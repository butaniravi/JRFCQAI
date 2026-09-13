# 🚁 JRFCQAI GCS Web Utility (Arduino UNO-Q Applab Project)

A high-performance, real-time Ground Control Station (GCS) web interface designed for telemetry monitoring, autonomous AI-driven flight, interactive mapping, and drone command control.
<!-- SCREENSHOT PLACEHOLDER: MAIN DASHBOARD OVERVIEW -->
![JRFCQAI GCS Dashboard Overview](path/to/dashboard-overview.png)
*Figure 1: Main GCS Web Dashboard Overview*
---

## 📑 Table of Contents
- [✨ Key Features](#-key-features)
  - [📊 Real-Time Telemetry & System Monitoring](#-real-time-telemetry--system-monitoring)
  - [🗺️ Interactive Mapping & Waypoint Navigation](#️-interactive-mapping--waypoint-navigation)
  - [👁️ Computer Vision & Autonomous AI Flight](#️-computer-vision--autonomous-ai-flight)
  - [🎮 Manual Control Matrix & REST API](#-manual-control-matrix--rest-api)
- [🛠️ Tech Stack](#️-tech-stack)

---

## ✨ Key Features

### 📊 Real-Time Telemetry & System Monitoring
* **📡 40-Byte Bridge Frame Parsing:** Decodes incoming binary telemetry frames (`stm_tx_tel`) with header matching (`JB`) and XOR checksum validation.
* **🟢 Live Flight Status:** Real-time display of flight modes (*Stabilize*, *Altitude Hold*, *GPS Hold*, *RTH*, *Waypoints*), armed/disarmed state, satellite count, and GPS fix type.
* **📐 Flight Metrics & Diagnostics:** Tracks pitch, roll, yaw angles, relative/max altitude, throttle percentage, system temperature, and line-of-sight/ground distance.
* **⚡ Power & Error Tracking:** Monitors real-time battery voltage and charge levels while flagging active system errors (low battery, sensor anomalies, watchdog timeouts).

---

### 🗺️ Interactive Mapping & Waypoint Navigation
* **🗺️ GIS Leaflet Map:** High-resolution satellite view featuring real-time drone position markers, heading-aligned directional icons, and home position coordinates.
* **📍 Click-to-Point Planning:** Drop route waypoints directly on the interactive map connected by auto-drawing path polylines.
* **⚙️ Automated Waypoint Engine:** Executes multi-point autonomous routes via a background state machine using packed 12-byte binary command frames (`WP`) with handshake retry logic.

---

### 👁️ Computer Vision & Autonomous AI Flight
* **📹 Live Video Feed:** Integrates an onboard camera stream directly into the UI dashboard (640x480 resolution).
* **🎯 Object Detection Pipeline:** Detects targets, calculates confidence scores, and determines bounding-box center coordinates in real time.
* **🤖 Target-Triggered AI Flight:** Toggleable AI flight mode that automatically dispatches altitude and positioning commands (`EU.....`) upon detecting specific targets (e.g., human detection).
* **🖥️ Live Console Output:** Real-time 50-entry dark-mode scrolling debug log for system events, AI detections, and outgoing commands.

---

### 🎮 Manual Control Matrix & REST API
* **🕹️ 3x4 Command Grid:** Dispatches 12 manual 12-byte packed binary commands (`CMD`) for Takeoff, Landing, Position Hold, directional pitch/roll/yaw adjustments, and Auxiliary switches (`AUX1–3`).
* **🚀 FastAPI Backend:** Multi-threaded web server powered by FastAPI/Uvicorn exposing asynchronous endpoints (`/api/data`, `/api/command`, `/api/waypoint/*`, `/api/ai_flight`) for continuous dashboard synchronization.

---

## 🛠️ Tech Stack

| Component | Technology |
| :--- | :--- |
| **Backend Framework** | FastAPI / Uvicorn (Python) |
| **Frontend Mapping** | Leaflet.js |
| **Protocol** | Custom 40-Byte & 12-Byte Packed Binary Telemetry (`JB` / `WP` / `CMD`) |
| **Video & CV Stream** | OpenCV / Custom AI Inference Engine |




