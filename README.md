# 🚀 JRFCQAI: Dual-Brain Physical AI Flight Controller & Ground Station Developed on Arduino UNO Q

[![Track: Robotics & Interactive AI](https://img.shields.io/badge/Track-Robotics%20%26%20Interactive%20AI-blue)](https://github.com)
[![Board: Arduino UNO Q](https://img.shields.io/badge/Board-Arduino%20UNO%20Q-teal)](https://github.com)
[![Status: Open Source](https://img.shields.io/badge/Status-Open--Source-brightgreen)](https://github.com)

**Developer:** Ravi Butani  
**Track:** Robotics & Interactive AI  
**Board:** Arduino UNO Q (Qualcomm® Dragonwing™ QRB2210 + STM32U585)  
**Project Status:** Open-Source Software & Hardware Framework  

---

## 📌 Project Overview

While most projects utilizing the newly launched **Arduino UNO Q** explore basic ground robotics, **JRFCQAI** pushes the platform to its limits: transforming a single alpha-stage UNO Q board into a full-fledged **Flight Computer + Flight Controller** combo for autonomous UAVs.

By leveraging the **Dual-Brain Architecture** of the UNO Q, this framework splits time-critical flight stabilization and high-level Edge AI processing across two dedicated processors on a single PCB:

1. **STM32U585 Microcontroller (MCU):** Runs a deterministic, 250Hz PID loop over Zephyr RTOS for real-time sensor fusion, motor control, and flight stabilization.
2. **Qualcomm® Dragonwing™ MPU:** Runs Debian Linux, processing live USB webcam video via YOLO Light Object Detection inside an Arduino App Lab environment. It acts as a local Web Ground Station streaming live telemetry, AI detections, and direct UI controls to any browser on the local Wi-Fi network.

---

## 🛠️ Hardware Architecture & Sensor Interfaces

* **Flight Controller Core:** Arduino UNO Q (Powered directly via VIN with a $1000\mu\text{F}$ 50V filter capacitor from a 3S 6000mAh LiPo).
* **Sensors (I2C1 QWIIC Port @ 400kHz):**
  * **MPU6050:** 6-DOF Gyroscope / Accelerometer
  * **MS5611:** Barometric Pressure / Altitude sensor
  * **HMC5883:** 3-Axis Magnetometer / Compass
* **GPS & Receiver:**
  * **u-blox NEO-M8N GPS:** Connected via UART3 (115200 baud)
  * **Flysky FS-iA6B 10CH Receiver:** iBus interface on UART1 (115200 baud)
* **Actuators & Monitoring:**
  * **4x BLDC ESCs:** Driven at 350Hz on Hardware PWM Pins (`D3`, `D5`, `D6`, `D9`)
  * **Battery Telemetry:** Voltage monitoring on Pin `A0` via a 10K/1K resistor voltage divider
  * **Status Indicators:** RGB Status LEDs on `D11`, `D12`, `D13`
* **Vision System:** Direct USB Webcam connected to the UNO Q Type-C port, powered externally via a dedicated 5V Buck Converter.
---

## 📹 Media & Video Demonstration

Watch the full build breakdown and flight demo on YouTube:  
▶️ [**JRFCQAI Video Playlist**](https://www.youtube.com/watch?u1FR_EWiBVY&list=PLfxRMvKGemR4)

---

## 💻 Quick Start & Software Setup

### 1. Flight Controller Firmware (MCU)
1. Open `UNOQ_JRFCQAI_MAIN` in the **Arduino IDE**.
2. Select your **Arduino UNO Q** board target.
3. Flash the sketch to the board to initialize the STM32U585 real-time flight loop.

### 2. Ground Station & AI System (MPU)
1. Launch **Arduino AppLab**.
2. Import the `JRFC_QAI_GCS` Python project.
3. Run the application to start the web ground station and AI detection engine.

> **💡 Hardware Note (Camera Troubleshooting):**  
> If your USB webcam does not initialize upon booting the Ground Station you need to enable USB Host mode on UNO Q, open the **Arduino UNO Q Shell** in AppLab and execute the following commands to force USB Host Mode:
> ```bash
> sudo -i
> echo host > /sys/kernel/debug/usb/4e00000.usb/mode
> ```

---

## 💡 Key Engineering Challenges & Technical Breakthroughs

1. **Low-Weight, Cost-Effective Power Routing:** Solved the USB Host power limitation on boot without using heavy, expensive USB-C Power Delivery hubs by routing webcam power to a lightweight 5V 500mA Buck Converter and triggering USB Host mode via startup scripts.
2. **Zephyr RTOS Kernel Hacks for PWM:** Overcame default 500Hz PWM constraints by hacking the underlying Zephyr RTOS layer to modify default output down to 350Hz, enabling precise ESC and servo synchronization.
3. **Serial Buffer Expansion for GPS Data:** Prevented packet loss on the standard 64-byte UART buffer by patching Zephyr RTOS to allocate a 512-byte UART RX buffer for loss-free u-blox GPS parsing.
4. **Optimized 250Hz PID Execution Loop:** Implemented `k_yield()` and `k_sleepus()` inside the main flight loop to guarantee a fixed 4ms (250Hz) execution cycle with ~3ms of spare processing headroom per loop.
5. **Zero-Blocking RPC Bridge Communication:** Replaced blocking `bridge.provide()` calls (which introduce 5–7ms delays in flight loop timing) with asynchronous `bridge.notify()` fire-and-forget RPC calls for non-blocking telemetry sync between the STM32 flight core and Qualcomm Linux host.
6. **Core Architecture Migration:** Ported and expanded the YMFC-32 Autonomous framework from legacy STM32F103 bare-metal registers to the modern STM32U5 platform running Zephyr RTOS, replacing PPM-SUM receiver input with native iBus support.

---

## 📊 Current Capabilities & Roadmap

- [x] Auto-Level Flight Mode *(Tested)*
- [x] Altitude Hold Mode *(Tested)*
- [x] GPS Position Hold Mode *(Tested)*
- [x] Automatic Takeoff & Landing *(Tested)*
- [x] Failsafe System *(RTH on GPS signal / Slow descent altitude hold on signal loss - Tested)*
- [x] Real-Time YOLO Object Detection AI *(Tested)*
- [x] Browser-Based Ground Station & Live Video Feed *(Tested)*
- [ ] Return to Home (RTH) Execution *(Implemented, pending field test)*
- [ ] Autonomous Waypoint Navigation *(Implemented, pending field test)*
- [ ] Framework expansion for Hexcopter & Fixed-Wing platforms


---

## 🌐 Open-Source Impact & Community Contribution

JRFCQAI is released as a fully open-source framework—including hardware schematics, Zephyr patch scripts, Arduino sketches, and Python App Lab code—to democratize advanced edge-AI robotics on the UNO Q platform. The goal is to provide the global maker community with a tested blueprint for complex, real-time autonomous systems without relying on high-cost commercial flight controllers.

---

## 🙏 Credits & Acknowledgments

Special thanks and full attribution to **Joop Broking**, creator of the original **YMFC-32 Autonomous Flight Controller**. His pioneering open-source work provided the fundamental mathematical foundation for the flight stabilization algorithms ported into this project.

---

## ⚠️ Disclaimer

*This is an experimental, open-source educational project developed for testing and research purposes. Unmanned Aerial Vehicles (UAVs) can cause physical injury or property damage if mishandled. This framework is provided **"AS IS"**, without warranty or guarantee of any kind. Operate experimental hardware at your own risk, follow local aviation regulations, and adhere to strict safety protocols during testing.*
