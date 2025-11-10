# 🕷️ Darkberry Hexapod Project

**Darkberry** is a six-legged robotic spider built using a **Raspberry Pi** and a **DualShock 4 (DS4) controller**.  
It combines **robotics, computer vision, and AI** to simulate the natural movements and behaviors of a real spider.  
The goal is to create an autonomous hexapod capable of **environmental analysis, object detection, and self-navigation** without human control.

---

## 🧠 Project Overview

Darkberry is designed to move and act like a living organism — capable of walking, turning, performing gestures, and streaming live visuals.  
In its early phase, it was remotely controlled using a PS4 controller. The ongoing development aims to give it **independent intelligence** using onboard sensors and AI models.

### Key Features
- 🦿 **6-Leg Walking Mechanism** – Smooth, stable gait controlled via servo motors.
- 🎮 **DS4 Controller Support** – Manual control for walking, turning, and gestures.
- 📷 **Camera Integration** – Capture photos and videos through the Raspberry Pi camera.
- 🧭 **Sensor Fusion (planned)** – For obstacle detection and autonomous pathfinding.
- 🧠 **AI Integration (in progress)** – Object recognition, decision-making, and behavior patterns.
- 🔋 **Portable Design** – Powered by Li-ion battery pack for untethered operation.

---

## ⚙️ Hardware Components

| Component | Description |
|------------|-------------|
| **Raspberry Pi 4 Model B** | Main control unit (brain) |
| **DS4 (DualShock 4) Controller** | Wireless control via Bluetooth |
| **16-Channel PWM Servo Driver (PCA9685)** | Controls leg servo motors |
| **Servo Motors (MG90S / SG90)** | 3 per leg × 6 legs = 18 servos |
| **Pi Camera Module** | Captures video and images |
| **Battery Pack (5V/2A)** | Power source |
| **Frame** | Custom 3D-printed spider chassis |

---

## 🧩 Software Stack

| Layer | Technology |
|--------|-------------|
| **OS** | Raspberry Pi OS (Lite or Desktop) |
| **Programming Language** | Python 3 |
| **Control Interface** | `pygame`, `evdev`, or `inputs` for DS4 |
| **Servo Control** | `Adafruit_PCA9685` |
| **Computer Vision** | `OpenCV`, `TensorFlow Lite` (for AI) |
| **Networking** | Flask / SocketIO for remote web control (optional) |

---

## 🚀 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/<your-username>/darkberry-hexapod.git
cd darkberry-hexapod
