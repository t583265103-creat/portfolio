# Portfolio — T. Hauser

Computer Science student. This repository contains projects in robotics, autonomous navigation, and full-stack development.

---

## Autonomous Wheelchair Navigation System

> C++ · Eigen · PCL · OpenCV · Octomap · YOLOv8 · Webots · React · Node.js · SQLite

Full autonomous navigation system for a wheelchair robot, simulated in Webots. Includes a real-time navigation core running at **100Hz** and a web dashboard for monitoring sensor data.

- Fused GPS, IMU, LiDAR, Camera, Radar, and Encoder data using **EKF** and **Factor Graph** (sliding window optimization) for real-time localization
- Planned global routes with **A\*** on an OSM graph and replanned locally with **D\*Lite** (incremental, 100ms cycle)
- Built a live **3D Octomap** + 2D costmap from LiDAR scans for obstacle avoidance, and tracked dynamic objects with **Hungarian Algorithm** + **ICP**
- Designed a multi-threaded Producer-Consumer architecture (11 workers) and a **React + Node.js** monitoring dashboard backed by SQLite
