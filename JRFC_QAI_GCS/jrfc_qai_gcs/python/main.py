# JRFCQAI GCS PYTHON CODE for WEB GCS utility
from collections import deque
import math
import struct
import threading
import time
from datetime import datetime, UTC

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel
import uvicorn

from arduino.app_peripherals.camera import Camera
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from arduino.app_utils import *

time.sleep(12)
# ==============================================================================
# FASTAPI INSTANCE & REQUEST MODELS
# ==============================================================================
app = FastAPI()


class Waypoint(BaseModel):
    lat: float
    lng: float


class RouteRequest(BaseModel):
    waypoints: list[Waypoint]


class CommandButtonRequest(BaseModel):
    btn_id: str


class AIFlightToggleRequest(BaseModel):
    enabled: bool


# ==============================================================================
# COMMAND BUTTON MESSAGES MAP
# ==============================================================================
BUTTON_MESSAGES = {
    "ET.....": "🛫 Executing Auto Takeoff Sequence",
    "TF.....": "🔼 Moving Forward",
    "EU.....": "🔺 Increasing Altitude",
    "TL.....": "◀️ Rolling Left",
    "EH.....": "⚓ Holding Position",
    "TR.....": "▶️ Rolling Right",
    "EL.....": "🛬 Executing Auto Landing Sequence",
    "TB.....": "🔽 Moving Backward",
    "ED.....": "🔻 Decreasing Altitude",
    "EX.....": "⚙️ Auxiliary Switch 1 Activated",
    "EY.....": "⚙️ Auxiliary Switch 2 Activated",
    "EZ.....": "⚙️ Auxiliary Switch 3 Activated",
}


# ==============================================================================
# OBJECT DETECTION, AI FLIGHT STATE & DEBUG CONSOLE LOGGING
# ==============================================================================
camera = Camera(resolution=(640, 480), fps=5)
video_detector = VideoObjectDetection(camera=camera, confidence=0.5, debounce_sec=3.0)

LAST_DETECTION_TIME = 0.0
DETECTION_INTERVAL = 2  # Max 5 checks per second

# Toggle state for AI Flight mode
AI_FLIGHT_ENABLED = False

# Ring buffer to store the last 50 detection log strings
DETECTION_LOGS = deque(maxlen=50)


def send_detections_to_console(detections: dict):
    global LAST_DETECTION_TIME
    now = time.time()

    if now - LAST_DETECTION_TIME < DETECTION_INTERVAL:
        return
    LAST_DETECTION_TIME = now

    if detections:
        summary = []
        has_person = False

        for label, instances in detections.items():
            if label.lower() == "person" and len(instances) > 0:
                has_person = True

            for item in instances:
                conf = item.get("confidence", 0.0)
                box = item.get("bounding_box_xyxy")
                if box:
                    xmin, ymin, xmax, ymax = box
                    cx = int((xmin + xmax) / 2)
                    cy = int((ymin + ymax) / 2)
                else:
                    cx, cy = 0, 0
                print("Center coordinates:", cx, cy)
                summary.append(f"{label} ({int(conf * 100)}%)")

        if summary:
            log_entry = f"🎯 Detections : {', '.join(summary)}"
            print(log_entry)
            DETECTION_LOGS.append(log_entry)

        # AI Flight feature logic
        if AI_FLIGHT_ENABLED and has_person:
            msg = "AI Flight Active: Sending Drone MOVE UP Command "
            print(msg)
            DETECTION_LOGS.append(msg)
            send_command_button_packet("EU.....")


video_detector.on_detect_all(send_detections_to_console)


# ==============================================================================
# GLOBAL TELEMETRY STATE & DICTIONARIES
# ==============================================================================
TELEMETRY_STATE = {
    "status": "WAITING",
    "error": "Normal",
    "flight_mode": "Not Connected",
    "flight_mode_code": 0,
    "battery_voltage": 0.0,
    "battery_bar_level": 0,
    "temperature": 0.0,
    "roll": 0,
    "pitch": 0,
    "start": 0,
    "altitude": 0.0,
    "max_altitude": 0.0,
    "throttle": 0,
    "heading": 0,
    "heading_lock": 0,
    "satellites": 0,
    "fix_type": 0,
    "drone_lat": 23.259021795549522,
    "drone_lng": 72.65170727804829,
    "home_lat": 0.0,
    "home_lng": 0.0,
    "home_set": False,
    "ground_distance": 0.0,
    "los_distance": 0.0,
    "setting_1": 0.0,
    "setting_2": 0.0,
    "setting_3": 0.0,
    "setting_4": 0.0,
    "setting_5": 0.0,
    "setting_6": 0.0,
    "last_update": 0.0,
}

FLIGHT_MODES = {
    1: "1-Stabilize Mode",
    2: "2-Altitude Hold",
    3: "3-GPS Hold",
    4: "4-Preparing RTH",
    5: "5-RTH Ascending",
    6: "6-RTH Navigation",
    7: "7-RTH Descending",
    8: "8-RTH Complete",
    9: "9-Fly to Waypoint",
}

ERRORS = {
    0: "Normal",
    1: "Low Battery",
    2: "Main Loop Overrun",
    3: "Exceeded Accelerometer Calibration Angle",
    4: "GPS Watchdog Timeout",
    5: "Manual Take-off Out of Range",
    6: "Take-off Not Detected",
    7: "Take-off Throttle Out of Range",
}

# ==============================================================================
# WAYPOINT CONTROL STATE
# ==============================================================================
waypoint_queue = []
waypoint_active = False
waypoint_status_msg = "Idle"


# ==============================================================================
# BRIDGE TELEMETRY PARSER (40-BYTE FRAME)
# ==============================================================================
def stm_tx_tel(raw_bytes: bytes):
    global TELEMETRY_STATE

    if len(raw_bytes) != 40:
        TELEMETRY_STATE["status"] = f"ERR: Bad length ({len(raw_bytes)}B)"
        return

    if raw_bytes[0] != ord("J") or raw_bytes[1] != ord("B"):
        TELEMETRY_STATE["status"] = "ERR: Header Mismatch"
        return

    calc_checksum = 0
    for b in raw_bytes[:39]:
        calc_checksum ^= b

    if calc_checksum != raw_bytes[39]:
        TELEMETRY_STATE["status"] = "ERR: Checksum Fail"
        return

    (
        header,
        err_code,
        mode_code,
        vbat_raw,
        temp_raw,
        roll_raw,
        pitch_raw,
        start_val,
        alt_raw,
        throttle_raw,
        yaw_raw,
        h_lock,
        sats,
        fix,
        lat_raw,
        lon_raw,
        s1_raw,
        s2_raw,
        s3_raw,
        s4_raw,
        s5_raw,
        s6_raw,
    ) = struct.unpack("<2sBBBhBBBhhhBBBiihhhhhh", raw_bytes[:39])

    lat = lat_raw / 1000000.0
    lon = lon_raw / 1000000.0
    altitude = (alt_raw - 10000) / 120.0

    TELEMETRY_STATE["status"] = "LIVE STREAM"
    TELEMETRY_STATE["error"] = ERRORS.get(err_code, f"Err {err_code}")
    TELEMETRY_STATE["flight_mode"] = FLIGHT_MODES.get(mode_code, f"Mode {mode_code}")
    TELEMETRY_STATE["flight_mode_code"] = mode_code
    TELEMETRY_STATE["battery_voltage"] = round(vbat_raw / 10.0, 2)
    TELEMETRY_STATE["battery_bar_level"] = vbat_raw
    TELEMETRY_STATE["temperature"] = round(temp_raw / 340.0 + 36.5, 1)
    TELEMETRY_STATE["roll"] = roll_raw - 100
    TELEMETRY_STATE["pitch"] = pitch_raw - 100
    TELEMETRY_STATE["start"] = start_val
    TELEMETRY_STATE["altitude"] = round(altitude, 2)

    if altitude > TELEMETRY_STATE["max_altitude"]:
        TELEMETRY_STATE["max_altitude"] = round(altitude, 2)

    TELEMETRY_STATE["throttle"] = throttle_raw
    TELEMETRY_STATE["heading"] = yaw_raw
    TELEMETRY_STATE["heading_lock"] = h_lock
    TELEMETRY_STATE["satellites"] = sats
    TELEMETRY_STATE["fix_type"] = fix
    TELEMETRY_STATE["drone_lat"] = lat
    TELEMETRY_STATE["drone_lng"] = lon

    TELEMETRY_STATE["setting_1"] = float(s1_raw)
    TELEMETRY_STATE["setting_2"] = float(s2_raw)
    TELEMETRY_STATE["setting_3"] = float(s3_raw)
    TELEMETRY_STATE["setting_4"] = float(s4_raw)
    TELEMETRY_STATE["setting_5"] = round(s5_raw / 8.5, 2)
    TELEMETRY_STATE["setting_6"] = float(s6_raw)
    TELEMETRY_STATE["last_update"] = time.time()

    if not TELEMETRY_STATE["home_set"] and sats > 4 and start_val == 2:
        TELEMETRY_STATE["home_set"] = True
        TELEMETRY_STATE["home_lat"] = lat
        TELEMETRY_STATE["home_lng"] = lon
    elif TELEMETRY_STATE["home_set"] and start_val == 0:
        TELEMETRY_STATE["home_set"] = False

    if TELEMETRY_STATE["home_set"]:
        d_lat = (lat_raw - int(TELEMETRY_STATE["home_lat"] * 1e6)) * 0.111
        d_lon = (lon_raw - int(TELEMETRY_STATE["home_lng"] * 1e6)) * (
            math.cos((lat_raw / 1000000.0) * 0.017453) * 0.111
        )
        ground_dist = math.sqrt(math.pow(d_lat, 2) + math.pow(d_lon, 2))
        los_dist = math.sqrt(math.pow(ground_dist, 2) + math.pow(altitude, 2))

        TELEMETRY_STATE["ground_distance"] = round(ground_dist, 1)
        TELEMETRY_STATE["los_distance"] = round(los_dist, 1)
    else:
        TELEMETRY_STATE["ground_distance"] = 0.0
        TELEMETRY_STATE["los_distance"] = 0.0


Bridge.provide("stm_tx_tel", stm_tx_tel)


def send_waypoint_packet(lat: float, lng: float):
    """Packs 12-byte binary waypoint command and transmits over Arduino Bridge."""
    click_lat = int(round(lat * 1e6))
    click_lon = int(round(lng * 1e6))

    send_buffer = bytearray(struct.pack("<2sii1s", b"WP", click_lat, click_lon, b"-"))

    check_byte = 0
    for b in send_buffer[:11]:
        check_byte ^= b

    send_buffer.append(check_byte)
    Bridge.notify("stm_rx_tel", bytes(send_buffer))


def send_command_button_packet(btn_id: str):
    """Packs 12-byte binary command packet starting with 'CMD' and transmits over Bridge."""
    btn_bytes = btn_id.upper().ljust(7)[:7].encode("ascii")

    send_buffer = bytearray(struct.pack("<3s7s1s", b"CMD", btn_bytes, b"-"))

    check_byte = 0
    for b in send_buffer:
        check_byte ^= b

    send_buffer.append(check_byte)

    Bridge.notify("stm_rx_tel", bytes(send_buffer))


# ==============================================================================
# SEQUENTIAL WAYPOINT STATE MACHINE THREAD
# ==============================================================================
def waypoint_sequence_worker():
    """State Machine: Transmits 1 WP, waits for acceptance, waits for arrival, then moves to next WP."""
    global waypoint_queue, waypoint_active, waypoint_status_msg

    max_retries = 10

    while True:
        if waypoint_active and len(waypoint_queue) > 0:
            total_points = len(waypoint_queue)

            for index, wp in enumerate(waypoint_queue, start=1):
                if not waypoint_active:
                    break

                waypoint_status_msg = f"Sending WP {index}/{total_points}..."
                accepted = False

                for try_count in range(1, max_retries + 1):
                    if not waypoint_active:
                        break

                    send_waypoint_packet(wp["lat"], wp["lng"])
                    time.sleep(0.5)

                    if TELEMETRY_STATE["flight_mode_code"] == 9:
                        accepted = True
                        break

                if not accepted:
                    waypoint_status_msg = f"Failed WP {index}: Drone non-responsive."
                    waypoint_active = False
                    break

                waypoint_status_msg = f"Flying to WP {index}/{total_points}..."
                while waypoint_active and TELEMETRY_STATE["flight_mode_code"] == 9:
                    time.sleep(0.2)

                if waypoint_active and TELEMETRY_STATE["flight_mode_code"] == 3:
                    waypoint_status_msg = f"Reached WP {index}/{total_points}"
                    time.sleep(1.0)
                elif waypoint_active:
                    waypoint_status_msg = (
                        f"Aborted: Drone mode changed to {TELEMETRY_STATE['flight_mode']}"
                    )
                    waypoint_active = False
                    break

            if waypoint_active:
                waypoint_status_msg = "Route Completed Successfully!"
                waypoint_active = False
                waypoint_queue = []

        time.sleep(0.2)


threading.Thread(target=waypoint_sequence_worker, daemon=True).start()


# ==============================================================================
# FASTAPI ENDPOINTS & WEB UI WITH MAP & VIDEO STREAM
# ==============================================================================
@app.get("/api/data")
def get_telemetry():
    response = dict(TELEMETRY_STATE)
    response["wp_active"] = waypoint_active
    response["wp_status_msg"] = waypoint_status_msg
    response["detection_logs"] = list(DETECTION_LOGS)
    response["ai_flight_enabled"] = AI_FLIGHT_ENABLED
    return JSONResponse(content=response)


@app.post("/api/ai_flight")
def toggle_ai_flight(req: AIFlightToggleRequest):
    global AI_FLIGHT_ENABLED
    AI_FLIGHT_ENABLED = req.enabled
    status_str = "ENABLED" if AI_FLIGHT_ENABLED else "DISABLED"
    msg = f"🤖 AI Flight Mode {status_str}"
    DETECTION_LOGS.append(msg)
    print(msg)
    return {"status": "success", "ai_flight_enabled": AI_FLIGHT_ENABLED}


@app.post("/api/waypoint/start")
def start_route(data: RouteRequest):
    global waypoint_queue, waypoint_active, waypoint_status_msg

    if waypoint_active:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": "Route sequence already active!"},
        )

    if not data.waypoints:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": "No waypoints provided!"},
        )

    waypoint_queue = [{"lat": wp.lat, "lng": wp.lng} for wp in data.waypoints]
    waypoint_active = True
    waypoint_status_msg = "Starting route sequence..."

    return {
        "status": "success",
        "message": f"Route started with {len(waypoint_queue)} points",
    }


@app.post("/api/waypoint/stop")
def stop_route():
    global waypoint_active, waypoint_status_msg
    waypoint_active = False
    waypoint_status_msg = "Route cancelled by user."
    return {"status": "success", "message": "Route execution stopped"}


@app.post("/api/command")
def execute_command(req: CommandButtonRequest):
    send_command_button_packet(req.btn_id)

    # Retrieve mapped button message or use default format
    msg = BUTTON_MESSAGES.get(req.btn_id, f"Button {req.btn_id} Pressed")

    # Append raw message directly to DETECTION_LOGS ring buffer
    DETECTION_LOGS.append(msg)

    print("Command Sent:", req.btn_id, "| Console Log:", msg)
    return {"status": "success", "command_sent": req.btn_id}


@app.get("/", response_class=HTMLResponse)
def web_interface():
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>JRFCQAI Ground Control Station</title>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
            body { display: flex; height: 100vh; background-color: #1a1a1a; color: #fff; overflow: hidden; font-size: 14px; }
            
            /* Custom Width 3-Column Layout (25% | 35% | 40%) */
            .column-left { flex: 0 0 25%; width: 25%; background: #242424; padding: 10px; display: flex; flex-direction: column; gap: 8px; overflow-y: auto; height: 100vh; }
            .column-center { flex: 0 0 35%; width: 35%; background: #242424; padding: 10px; display: flex; flex-direction: column; gap: 8px; overflow-y: auto; height: 100vh; border-left: 1px solid #333; border-right: 1px solid #333; }
            .column-right { flex: 0 0 40%; width: 40%; height: 100vh; display: flex; flex-direction: column; padding: 10px; gap: 8px; background: #1f1f1f; overflow: hidden; }

            /* Map Container and Map element */
            .map-container { position: relative; width: 100%; height: 63%; }
            #map { width: 100%; height: 100%; border-radius: 6px; border: 1px solid #3d3d3d; }

            /* Recenter Drone Floating Overlay Button */
            #recenterBtn {
                position: absolute;
                top: 10px;
                right: 10px;
                z-index: 1000;
                width: auto;
                padding: 6px 12px;
                background: rgba(36, 36, 36, 0.85);
                color: #00ff88;
                border: 1px solid #00ff88;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                cursor: pointer;
                transition: 0.2s;
            }
            #recenterBtn:hover {
                background: #00ff88;
                color: #000;
            }

            .card { background: #2f2f2f; border-radius: 6px; padding: 10px; border: 1px solid #3d3d3d; }
            .card h3 { font-size: 16px; color: #00bcd4; margin-bottom: 6px; border-bottom: 1px solid #444; padding-bottom: 4px; text-transform: uppercase; }
            .card-header-flex { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; border-bottom: 1px solid #444; padding-bottom: 4px; }
            .card-header-flex h3 { border-bottom: none; padding-bottom: 0; margin-bottom: 0; }
            .row { display: flex; justify-content: space-between; margin-bottom: 6px; font-size: 14px; }
            .val { font-weight: bold; color: #4caf50; }
            
            button { width: 100%; padding: 10px 6px; border-radius: 4px; border: none; font-weight: bold; cursor: pointer; transition: 0.2s; font-size: 14px; }
            button.btn-action { background: #ff9800; color: white; }
            button.btn-action:hover { background: #e68a00; }
            button.btn-danger { background: #f44336; color: white; }
            button.btn-danger:hover { background: #d32f2f; }

            /* Toggle Switch Styling */
            .toggle-btn {
                width: auto;
                padding: 5px 12px;
                font-size: 12px;
                border-radius: 14px;
                background: #555;
                color: #fff;
                border: 1px solid #777;
                cursor: pointer;
                transition: 0.3s;
                text-transform: uppercase;
            }
            .toggle-btn.active {
                background: #00ff88;
                color: #000;
                border-color: #00ff88;
                box-shadow: 0 0 8px rgba(0, 255, 136, 0.6);
            }

            .battery-bar { height: 10px; background: #444; border-radius: 4px; overflow: hidden; margin-top: 4px; }
            .battery-fill { height: 100%; width: 0%; background: #4caf50; transition: width 0.3s; }
            .settings-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; font-size: 14px; }

            /* Live Stream Video Frame */
            .video-container { 
                position: relative; 
                width: 100%; 
                height: 364px; 
                background: #000; 
                border-radius: 4px; 
                overflow: hidden !important; 
                border: 1px solid #3d3d3d; 
                display: flex; 
                justify-content: center; 
                align-items: center; 
            }
            .video-container iframe { 
                width: 100%; 
                height: 100%; 
                border: 0; 
                outline: none; 
                display: block; 
                overflow: hidden !important; 
                background: #fff; 
            }
            .video-badge { position: absolute; top: 6px; left: 6px; z-index: 10; background: rgba(0, 255, 136, 0.9); color: #000; padding: 3px 7px; font-size: 12px; font-weight: bold; border-radius: 3px; text-transform: uppercase; }

            /* Side-by-Side Waypoint Button Grid */
            .wp-btn-group { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; margin-top: 4px; }

            /* Command Matrix (3x4) */
            .grid-3x4 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
            .btn-cmd { background: #3a3a4a; color: #00ff88; border: 1px solid #00ff88; padding: 4px 0; font-size: 14px; font-weight: bold; border-radius: 4px; cursor: pointer; text-align: center; }
            .btn-cmd:hover { background: #00ff88; color: #000; }
            .btn-cmd:active { transform: scale(0.95); }

            /* Debug Console Output Styling */
            .debug-console {
                background: #111;
                color: #00ff88;
                font-family: 'Courier New', Courier, monospace;
                font-size: 14px;
                padding: 8px;
                border-radius: 4px;
                border: 1px solid #333;
                height: 90px;
                overflow-y: auto;
                white-space: pre-wrap;
                line-height: 1.3;
            }

            /* Footer Card Styling */
            .info-card { background: #252525; padding: 8px 10px; font-size: 13px; color: #aaa; line-height: 1.4; border: 1px solid #333; }
            .info-card p { margin-bottom: 3px; }
            .info-card a { color: #00bcd4; text-decoration: none; }
            .info-card a:hover { text-decoration: underline; }

            /* Drone Heading Arrow Icon Styling */
            .drone-arrow-icon {
                transition: transform 0.3s ease-in-out;
            }
        </style>
    </head>
    <body>
        <!-- COLUMN 1: SYSTEM STATUS & TELEMETRY (25% WIDTH) -->
        <div class="column-left">
            <h2 style="margin-bottom: 6px; font-family: 'Segoe UI', sans-serif; line-height: 1.3;">
                <span style="font-size: 40px; color: #00ff88; font-weight: bold; display: block;">JRFCQAI GCS</span>
                <span style="font-size: 25px; color: #00bcd4; font-weight: 600; display: block; margin-bottom: 8px;">Arduino UNO Q based AI UAV</span>
                <span style="font-size: 20px; color: #ffb74d; font-weight: 400; display: block;"> By - Ravi Butani (not PhD)</span>
                <span style="font-size: 20px; color: #ffb74d; font-weight: 400; display: block; margin-bottom: 4px;"> Credits - Joop Brokking</span>
            </h2>
            <div class="card">
                <h3>System Status</h3>
                <div class="row"><span>Link Status:</span><span id="status" class="val">--</span></div>
                <div class="row"><span>Flight Mode:</span><span id="flight_mode" class="val">--</span></div>
                <div class="row"><span>Start Flag:</span><span id="start_flag" class="val">0</span></div>
                <div class="row"><span>Error Message:</span><span id="error" class="val">--</span></div>
                <div class="row"><span>Satellites:</span><span id="satellites" class="val">0</span></div>
                <div class="row"><span>Battery Voltage:</span><span id="battery_voltage" class="val">0.0 V</span></div>
                <div class="battery-bar"><div id="battery_fill" class="battery-fill"></div></div>
            </div>

            <div class="card">
                <h3>Telemetry Data</h3>
                <div class="row"><span>Pitch / Roll:</span><span id="pitch_roll" class="val">0° / 0°</span></div>
                <div class="row"><span>Heading:</span><span id="heading" class="val">0°</span></div>
                <div class="row"><span>Throttle:</span><span id="throttle" class="val">0</span></div>
                <div class="row"><span>Altitude (Rel / Max):</span><span id="altitude" class="val">0 m / 0 m</span></div>
                <div class="row"><span>LOS Distance:</span><span id="los_distance" class="val">0 m</span></div>
                <div class="row"><span>Temperature:</span><span id="temperature" class="val">0 °C</span></div>
            </div>

            <div class="card">
                <h3>Watch Key Variables</h3>
                <div class="settings-grid">
                    <div>V1-CH1: <span id="setting_1" class="val">0</span></div>
                    <div>V2-CH2: <span id="setting_2" class="val">0</span></div>
                    <div>V3-CH3: <span id="setting_3" class="val">0</span></div>
                    <div>V4-CH4: <span id="setting_4" class="val">0</span></div>
                    <div>V5-ALTSP: <span id="setting_5" class="val">0.0</span></div>
                    <div>V6: <span id="setting_6" class="val">0</span></div>
                </div>
            </div>
        </div>

        <!-- COLUMN 2: CAMERA, COMMAND BUTTONS & DEBUG CONSOLE (35% WIDTH) -->
        <div class="column-center">
            <div class="card">
                <h3>Live UNO Q Stream</h3>
                <div class="video-container">
                    <iframe id="dynamicIframe" title="Video stream" scrolling="no" style="overflow:hidden;"></iframe>
                </div>
            </div>

            <div class="card">
                <div class="card-header-flex">
                    <h3>Command Buttons</h3>
                    <button id="aiFlightToggle" class="toggle-btn" onclick="toggleAIFlight()">AI Flight: OFF</button>
                </div>
                <div class="grid-3x4" id="cmdGrid"></div>
            </div>

            <div class="card">
                <h3>Debug Console</h3>
                <div class="debug-console" id="debugConsole">Waiting for detections...</div>
            </div>
        </div>

        <!-- COLUMN 3: MAP DISPLAY & WAYPOINT CONTROLS (40% WIDTH) -->
        <div class="column-right">
            <div class="map-container">
                <div id="map"></div>
                <button id="recenterBtn" onclick="recenterOnDrone()">🎯 Recenter Drone</button>
            </div>

            <!-- Waypoint Controls Card (Height Reduced) -->
            <div class="card" style="padding: 8px 10px;">
                <h3 style="font-size: 16px; margin-bottom: 4px;">Waypoint Controls</h3>
                <p style="font-size: 14px; color: #aaa; margin-bottom: 4px;">Click on map to mark target route</p>
                <div class="row" style="font-size: 14px; margin-bottom: 4px;"><span>WP Status:</span><span id="wp_status_msg" class="val" style="color:#ffb74d;">Idle</span></div>
                <div class="wp-btn-group">
                    <button class="btn-action" style="padding: 6px; font-size: 14px;" onclick="startSequentialRoute()">Execute Route</button>
                    <button class="btn-danger" style="padding: 6px; font-size: 14px;" onclick="stopRoute()">Cancel Route</button>
                    <button class="btn-action" style="background:#607d8b; padding: 6px; font-size: 14px;" onclick="clearWaypoints()">Clear Points</button>
                </div>
            </div>

        <!-- Copyright & Info Card -->
        <div class="card info-card">
            <p>
                <strong>JRFCQAI Arduino UNO Q based AI UAV STACK</strong><br>
                <strong>PHILOSOPHY & LICENSE:</strong> PUBLIC DOMAIN / UNLICENSE — Free for all human knowledge.<br>
                Provided "as is" without warranty of any kind. Use at your own risk.
            </p>
            <p><strong>Developed By:</strong> Ravi Butani | <strong>Credits:</strong> Inspired by and dedicated to Joop Brokking (<a href="http://www.brokking.net/" target="_blank">YMFC Project</a>)</p>
            <p><strong>Support & Contact:</strong> <a href="mailto:ravi.butani03@gmail.com">ravi.butani03@gmail.com</a> | Revision 1.0.2</p>
        </div>
        </div>

        <script>
            let aiFlightActive = false;

            window.addEventListener('DOMContentLoaded', () => {
                const iframe = document.getElementById('dynamicIframe');
                iframe.src = `http://${window.location.hostname}:4912/embed`;

                const btnLabels = [
                    "🛫 TAKEOFF", "🔼 FRONT", "🔺 UP",
                    "◀️ LEFT",    "⚓ HOLD",  "▶️ RIGHT",
                    "🛬 LAND",    "🔽 BACK",  "🔻 DOWN",
                    "⚙️ AUX1",    "⚙️ AUX2",  "⚙️ AUX3"
                ];
                
                const btnCommands = [
                    "ET.....",  "TF.....", "EU.....", 
                    "TL.....",  "EH.....", "TR.....", 
                    "EL.....",  "TB.....", "ED.....", 
                    "EX.....",  "EY.....", "EZ....."
                ];
                
                const cmdGrid = document.getElementById('cmdGrid');
                for (let i = 0; i < 12; i++) {
                    const btn = document.createElement('button');
                    btn.className = 'btn-cmd';
                    btn.innerText = btnLabels[i] || `BTN${i + 1}`;
                    
                    const cmdToSend = btnCommands[i] || `BTN${i + 1}`;
                    btn.onclick = () => triggerButtonCommand(cmdToSend);
                    
                    cmdGrid.appendChild(btn);
                }
            });

            async function triggerButtonCommand(btnId) {
                await fetch('/api/command', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ btn_id: btnId })
                });
            }

            async function toggleAIFlight() {
                aiFlightActive = !aiFlightActive;
                await fetch('/api/ai_flight', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ enabled: aiFlightActive })
                });

                const btn = document.getElementById('aiFlightToggle');
                if (aiFlightActive) {
                    btn.innerText = "AI Flight: ON";
                    btn.classList.add('active');
                } else {
                    btn.innerText = "AI Flight: OFF";
                    btn.classList.remove('active');
                }
            }

            let map = L.map('map').setView([23.259021795549522, 72.65170727804829], 20);
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
                maxNativeZoom: 19,
                maxZoom: 22,
                attribution: 'Esri World Imagery'
            }).addTo(map);

            // Function to generate rotated SVG arrow Icon for the drone
            function createDroneArrowIcon(headingAngle = 0) {
                const svgString = `
                    <svg width="36" height="36" viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg" style="transform: rotate(${headingAngle}deg); transform-origin: center center; filter: drop-shadow(0px 0px 4px rgba(0,0,0,0.8));">
                        <polygon points="18,2 32,32 18,24 4,32" fill="#ff3333" stroke="#ffffff" stroke-width="2.5" stroke-linejoin="round"/>
                        <circle cx="18" cy="18" r="3" fill="#ffffff"/>
                    </svg>
                `;
                return L.divIcon({
                    html: svgString,
                    className: 'drone-arrow-icon',
                    iconSize: [36, 36],
                    iconAnchor: [18, 18],
                    popupAnchor: [0, -18]
                });
            }

            let homeIcon = L.icon({
                iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
                shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                iconSize: [25, 41],
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            });

            let droneMarker = L.marker([23.259021795549522, 72.65170727804829], { icon: createDroneArrowIcon(0) })
                .addTo(map)
                .bindPopup("<b>Drone Position</b>");

            let homeMarker = null;
            let waypoints = [];
            let waypointMarkers = [];
            let routePolyline = L.polyline([], {color: 'red'}).addTo(map);

            function recenterOnDrone() {
                let currentLatLng = droneMarker.getLatLng();
                if (currentLatLng.lat !== 0 && currentLatLng.lng !== 0) {
                    map.panTo(currentLatLng);
                } else {
                    alert("No valid drone GPS coordinates received yet!");
                }
            }

            map.on('click', function(e) {
                let lat = e.latlng.lat;
                let lng = e.latlng.lng;
                waypoints.push({lat: lat, lng: lng});
                
                let marker = L.marker([lat, lng]).addTo(map).bindPopup("Waypoint " + waypoints.length);
                waypointMarkers.push(marker);
                routePolyline.addLatLng([lat, lng]);
            });

            function clearWaypoints() {
                waypoints = [];
                waypointMarkers.forEach(m => map.removeLayer(m));
                waypointMarkers = [];
                routePolyline.setLatLngs([]);
            }

            async function startSequentialRoute() {
                if (waypoints.length === 0) return alert("Click map to add waypoints first!");
                let res = await fetch('/api/waypoint/start', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ waypoints: waypoints })
                });
                let data = await res.json();
                if (data.status === "error") {
                    alert(data.message);
                }
            }

            async function stopRoute() {
                await fetch('/api/waypoint/stop', { method: 'POST' });
            }

            setInterval(async () => {
                try {
                    let res = await fetch('/api/data');
                    let t = await res.json();

                    document.getElementById('status').innerText = t.status;
                    document.getElementById('flight_mode').innerText = t.flight_mode;
                    document.getElementById('start_flag').innerText = (t.start === 2) ? "Armed" : "Disarmed";
                    document.getElementById('error').innerText = t.error;
                    document.getElementById('satellites').innerText = t.satellites;
                    document.getElementById('battery_voltage').innerText = t.battery_voltage + " V";
                    document.getElementById('pitch_roll').innerText = t.pitch + "° / " + t.roll + "°";
                    document.getElementById('heading').innerText = t.heading + "°";
                    document.getElementById('throttle').innerText = t.throttle;
                    document.getElementById('altitude').innerText = t.altitude + "m / " + t.max_altitude + "m";
                    document.getElementById('los_distance').innerText = t.los_distance + " m";
                    document.getElementById('temperature').innerText = t.temperature + " °C";

                    document.getElementById('setting_1').innerText = t.setting_1;
                    document.getElementById('setting_2').innerText = t.setting_2;
                    document.getElementById('setting_3').innerText = t.setting_3;
                    document.getElementById('setting_4').innerText = t.setting_4;
                    document.getElementById('setting_5').innerText = t.setting_5.toFixed(2) + " m";
                    document.getElementById('setting_6').innerText = t.setting_6;

                    document.getElementById('wp_status_msg').innerText = t.wp_status_msg;

                    // Sync UI toggle button state with server
                    if (t.ai_flight_enabled !== undefined && t.ai_flight_enabled !== aiFlightActive) {
                        aiFlightActive = t.ai_flight_enabled;
                        const btn = document.getElementById('aiFlightToggle');
                        if (aiFlightActive) {
                            btn.innerText = "AI Flight: ON";
                            btn.classList.add('active');
                        } else {
                            btn.innerText = "AI Flight: OFF";
                            btn.classList.remove('active');
                        }
                    }

                    let batPct = Math.max(0, Math.min(100, (t.battery_bar_level - 85) * 2.5));
                    document.getElementById('battery_fill').style.width = batPct + "%";

                    if (t.detection_logs && t.detection_logs.length > 0) {
                        const consoleElem = document.getElementById('debugConsole');
                        consoleElem.innerText = t.detection_logs.join('\\n');
                        consoleElem.scrollTop = consoleElem.scrollHeight;
                    }

                    if (t.drone_lat !== 0 && t.drone_lng !== 0) {
                        droneMarker.setLatLng([t.drone_lat, t.drone_lng]);
                    }

                    // Dynamically update arrow orientation according to drone heading angle
                    let droneHeading = t.heading || 0;
                    //droneMarker.setIcon(createDroneArrowIcon(droneHeading));
                    // ADD THIS LINE INSTEAD:
                    if (droneMarker._icon) droneMarker._icon.firstElementChild.style.transform = `rotate(${droneHeading}deg)`;

                    if (t.home_set && t.home_lat !== 0 && t.home_lng !== 0) {
                        if (!homeMarker) {
                            homeMarker = L.marker([t.home_lat, t.home_lng], { icon: homeIcon })
                                .addTo(map)
                                .bindPopup("<b>Home Position</b>");
                        } else {
                            homeMarker.setLatLng([t.home_lat, t.home_lng]);
                        }
                    } else if (!t.home_set && homeMarker) {
                        map.removeLayer(homeMarker);
                        homeMarker = null;
                    }
                } catch(e) {}
            }, 200);
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)


# ==============================================================================
# MAIN APPLICATION LOOP
# ==============================================================================
def user_loop():
    def run_web_server():
        uvicorn.run(app, host="0.0.0.0", port=7000, log_level="warning")

    server_thread = threading.Thread(target=run_web_server, daemon=True)
    server_thread.start()

    while True:
        time.sleep(1)


App.run(user_loop=user_loop)
