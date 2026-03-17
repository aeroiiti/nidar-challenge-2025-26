# 🚁 Autonomous Disaster Management Drone System

## 📌 Overview

This repository contains the complete software architecture, autonomous flight logic, and computer vision pipelines developed by **Team RAPTOR (ID: N250781)** for the **NIDAR Challenge 2025–26**.

Our solution is a **dual-drone autonomous system** designed for rapid **survey, reconnaissance, and precision payload delivery** in disaster-stricken environments.

---

## 🎯 Problem Statement

During severe flooding in a coastal region, residents may become stranded on rooftops without access to essential supplies.

### Mission Objectives

* Autonomously scan a **30-hectare disaster zone**
* Detect and geotag survivors using real-time vision
* Deploy a delivery drone to **precisely drop 200g survival kits**

---

## 🧠 System Architecture

```text
Scout Drone (Detection + Geotagging)
        ↓  (LoRa / Serial Communication)
Geotag Coordinates (lat, lon)
        ↓
Delivery Drone (Planning + Navigation + AI Alignment)
        ↓
Precision Payload Deployment
```

---
## 📂 Project Structure

```bash
NIDAR/
├── README.md
├── .gitignore
│
├── delivery_drone/
│   ├── main.py
│   ├── behaviours/
│   ├── models/
│   │   └── yolov8s_new.hef
│   └── data/
│       └── received_geotags.json
│
├── scout_drone/
│   ├── scout.py
│   ├── geolocation.py
│   ├── models/
│   │   └── yolov8s_visdrone.pt
│   └── mission_files/
│       └── cricketground_full.kml
│
└── docs/
    ├── architecture.png
    └── system_design.md
```

---

## 🚀 Key Features

### 🛰️ Scout Drone (Reconnaissance & Geotagging)

#### 🗺️ Automated Flight Path Generation
- Generates a lawnmower grid pattern from KML boundaries  
- Ensures complete area coverage with minimal overlap  

---

#### 👁️ Real-Time Survivor Detection
- YOLOv8s (VisDrone dataset) with BoT-SORT tracking  
- Adaptive confidence scaling for distant targets  
- Robust detection with minimal false positives  

---

#### 📍 Smart Geotagging
- Trigger-based detection (bottom-frame zone)  
- Ensures near-vertical alignment for accurate GPS tagging  
- Filters duplicate detections using spatial thresholds  

---

#### 📡 Communication Pipeline
- Transmits geotags in real-time via LoRa serial communication  
- Lightweight and reliable for long-range communication  

---

### 🚁 Delivery Drone (Planning & Payload Delivery)

#### 🧮 TSP-Based Path Optimization
- Groups geotags into batches based on payload capacity  
- Uses Traveling Salesman Problem (TSP) solver  
- Minimizes total flight distance and energy usage  

---

#### 🌳 Behavior Tree Autonomy
- Built using py_trees  
- Handles:
  - Mission execution  
  - Safety checks  
  - Fail-safe actions (RTL, payload checks)  

---

#### 🎯 Precision AI Alignment
- Powered by Hailo-8 AI Accelerator  
- Real-time visual servoing using pixel error  
- Dynamically adjusts drone position for accurate targeting  

---

#### 📦 Controlled Payload Deployment
- Servo-actuated payload release mechanism  
- Supports sequential and safe delivery  

---

#### ⚙️ Parallel Execution System
- Dual-threaded architecture:
  - Control thread (flight logic)  
  - Communication thread (geotag reception)  
- Uses shared JSON blackboard for synchronization

---
## 🚁 System Components

---

### 🔹 Scout Drone (Quadcopter)

#### Hardware

* Custom PLA + Carbon Fiber Frame
* 920KV BLDC Motors + 1045 Propellers
* Pixhawk 6C Flight Controller
* NVIDIA Jetson Orin Nano (AI compute)
* F9P GPS Module
* LoRa Communication Module
* USB / SIYI Camera

#### Software Architecture

* Multi-threaded execution:

  * Flight path generation
  * Detection & tracking
  * Geotag transmission

---

### 🔹 Delivery Drone (Hexacopter)

#### Hardware

* Custom Hexacopter Frame
* 22000mAh 4S LiPo Battery
* Pixhawk Cube Orange+
* Raspberry Pi 5 + Hailo AI Hat
* Here 3+ GNSS
* SiK Telemetry Module
* Raspberry Pi AI Camera

#### Software Architecture

* Dual-threaded system:

  * **Control Thread** → Behavior Tree execution
  * **Communication Thread** → LoRa geotag reception

* Shared JSON-based **Blackboard System**

---

## 🧪 Tech Stack

### Flight Control & Autonomy

* DroneKit Python
* MAVLink (pymavlink)
* py_trees (Behavior Trees)

### Computer Vision

* OpenCV
* Ultralytics YOLOv8
* Hailo GStreamer Pipeline

### Hardware & Communication

* NVIDIA JetPack
* Raspberry Pi OS
* LoRa (pyserial)

### Design & Validation

* ANSYS (Static Structural Analysis)

---

## ⚙️ Installation & Setup

### 🔽 Clone Repository

```bash
git clone https://github.com/aeroiit/nidar-challenge-2025-26.git
cd nidar-challenge-2025-26
```

---

### 🛰️ Scout Drone Setup (Jetson Orin Nano)

```bash
cd scout_drone
pip install -r requirements.txt
```

> Ensure YOLO weights are placed in:

```
scout_drone/models/
```

---

### 📦 Delivery Drone Setup (Raspberry Pi 5)

```bash
cd delivery_drone
pip install -r requirements.txt
```

> Ensure:

* Hailo runtime installed
* GStreamer plugins configured
* `.hef` model placed in:

```
delivery_drone/models/
```

---

## ▶️ Usage

---

### 🛰️ Run Scout Drone

```bash
cd scout_drone
python3 scout.py mission_files/PATH_TO_KML_FILE.kml
```

> Automatically:

* Generates path
* Detects survivors
* Sends geotags

---

### 🚁 Run Delivery Drone

```bash
cd delivery_drone
python3 main.py
```

> Automatically:

* Receives geotags
* Plans optimal route
* Aligns and drops payload

---

## 🧩 Key Modules

* **main.py (Delivery)**
  Behavior Tree execution and mission control

* **scout.py (Scout)**
  Flight path generation and navigation

* **geolocation.py (Scout)**
  Detection, tracking, and geotagging

* **drone_allign.py (Delivery)**
  Hailo-based visual alignment using pixel error

* **path_planner.py (Delivery)**
  TSP-based route optimization

* **geotag_read.py (Delivery)**
  LoRa communication + ACK protocol

* **deploy_action.py (Delivery)**
  MAVLink servo control for payload release

---

## 🛡️ Safety & Reliability

* Return-to-Launch (RTL) on:

  * Low battery
  * Payload depletion

* Duplicate geotag filtering

* Stable detection verification

* Robust communication with ACK protocol

---

## 🔮 Future Improvements

* Multi-target prioritization using confidence scoring
* Swarm coordination between multiple scout drones
* Improved control using PID-based alignment
* Real-time mission monitoring dashboard

---

## 👨‍💻 Team RAPTOR

Developed for the **NIDAR Disaster Management Challenge (2025–26)**

---

## 📜 License

This project is intended for academic and research purposes.
