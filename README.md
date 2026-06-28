# Portfolio — T. Hauser

Computer Science student. This repository contains projects in robotics, autonomous navigation, and full-stack development.

---

## Autonomous Wheelchair Navigation System

> C++ · Eigen · PCL · OpenCV · Octomap · YOLOv8 · Webots · SQLite

Full autonomous navigation system for a wheelchair robot, simulated in Webots. Runs a closed control loop at **100Hz** across 6 sensor types.

**What it does:**
- Fuses GPS, IMU, LiDAR, Camera, Radar, and Encoder data using **EKF** and **Factor Graph** (sliding window optimization) for real-time localization
- Plans global routes with **A\*** on an OSM graph and replans locally with **D\*Lite** (incremental, 100ms cycle)
- Builds a live **3D Octomap** + 2D costmap from LiDAR scans for obstacle avoidance
- Tracks dynamic objects with **Hungarian Algorithm** + **ICP** across a multi-threaded Producer-Consumer pipeline (11 workers)

**[/](.) — C++ navigation core** | **[/AdminApp](AdminApp) — React + Node.js monitoring dashboard**

---

## AdminApp — Sensor Monitoring Dashboard

> React · Node.js · Express · SQLite · Recharts · Vite

Web dashboard for monitoring sensor frames collected during simulation.

- Live frame browser with per-sensor filtering
- Confidence visualization per detection
- REST API backed by SQLite frame database
