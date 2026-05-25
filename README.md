# 🚁 Drone Simulation HUD v3

> A real-time drone flight simulator with a **military-grade HUD interface**, built in Python with PyQt6.

---

## ✨ Overview

This project is a **complete drone simulation environment** combining:
- 🧠 Physics-based flight model  
- 🎮 PID flight control system  
- 🖥️ Advanced avionics-style HUD  
- 📡 Real-time telemetry visualization  
- ⚙️ Modular architecture (simulation + UI + control)

It is designed for learning, experimentation, and engineering visualization of drone dynamics.

---

## 🧰 Tech Stack

- 🐍 Python 3.10+
- 🖼️ PyQt6 (GUI / HUD rendering)
- 🧮 Custom physics engine
- 🎯 PID control system
- 📊 Real-time state smoothing (EMA filters)

---

## 🚀 Features

### 🧠 Flight Simulation
- Full 3D drone state (position, velocity, attitude)
- Realistic motion integration
- Battery consumption model
- Multi-mode flight system:
  - Ground 🟫
  - Takeoff 🟡
  - Flight 🟢
  - Landing 🟠
  - Emergency 🔴

---

### 🖥️ Advanced HUD (v3 Upgrade)

#### 🎯 Flight Display
- Artificial horizon (roll / pitch)
- Flight Path Marker (FPM)
- Glide slope indicator (landing assist – 5°)
- Scanline HUD effect for realism

#### 📡 Navigation & Radar
- 10m / 20m / 30m radar rings
- Safety zone detection (>25m)
- Drone trajectory trail
- Velocity vector + yaw direction

#### 📊 Telemetry Panel
- Altitude, speed, vertical speed
- Roll / Pitch / Yaw
- Target altitude
- Commanded velocity inputs

#### 🔋 Power System
- Battery percentage & voltage
- Critical battery blinking (<15%)
- Estimated remaining flight time
- Motor-by-motor power output

#### ⚡ Flight Intelligence
- Peak speed tracking
- Hover detection system
- Real-time battery drain estimation
- Smooth sensor filtering (EMA)

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| `Z / S` | Increase / decrease altitude |
| `↑ ↓ ← →` | Move drone |
| `Q / D` | Yaw rotation |
| `T` | Takeoff |
| `L` | Landing |
| `SPACE` | Emergency stop |
| `R` | Reset drone |
| `H` | Return home |
| `J` | Toggle day/night mode |
| `I` | Engineering overlay (HIL debug) |

---

## 🧱 Architecture
