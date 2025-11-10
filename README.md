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
# Darkberry Hexapod 🕷️

## 2. Install Dependencies

```bash
sudo apt update && sudo apt install python3-pip python3-opencv
pip3 install adafruit-circuitpython-pca9685 pygame flask
```

## 3. Connect Hardware

- Connect all 18 servos to PCA9685.
- Connect PCA9685 to Raspberry Pi GPIO (SDA, SCL, VCC, GND).
- Pair the DualShock 4 controller via Bluetooth.

## 4. Run the Controller Script

```bash
python3 darkberry_controller.py
```

## 5. (Optional) Run Camera Stream

```bash
python3 camera_stream.py
```

---

## 🧠 Future Goals

- [ ] Integrate ultrasonic or LiDAR sensors for environment mapping.
- [ ] Develop AI-based obstacle avoidance using computer vision.
- [ ] Implement gait optimization using reinforcement learning.
- [ ] Add spider-like behavior simulation (e.g., hunting, reacting to movement).
- [ ] Design a web dashboard for monitoring and controlling the robot remotely.

---

## 📸 Media

You can include photos, build logs, and videos of the hexapod here:

```
/media
  ├── images/
  ├── videos/
```

**Example:** *(coming soon)*

---

## 🧑‍💻 Author

**Mausam Bahar Barbhuiya**

🎓 MCA, Sikkim Manipal Institute of Technology  
💡 Robotics | AI | Embedded Systems | Raspberry Pi Enthusiast  
📷 Independent Music & Video Director

---

## 📄 License

This project is licensed under the **MIT License** — feel free to use, modify, and build upon it with credit.

---

## ⭐ Acknowledgments

- Raspberry Pi Foundation
- Adafruit Industries
- OpenCV Community
- Sony DualShock 4 SDK Developers
- Inspiration from natural spider locomotion and biomechanics research

---

*"The Darkberry Hexapod walks not just with legs — but with curiosity."* 🕷️

---

## 🎬 Auto-generating GIF Preview

### Option 1: Manual Conversion (Recommended)

Use `ffmpeg` to convert your video to a GIF:

```bash
ffmpeg -i media/videos/demo.mp4 -vf "fps=10,scale=640:-1:flags=lanczos" -loop 0 media/demo.gif
```

Then embed it in your README:

```markdown
![Darkberry Hexapod Demo](media/demo.gif)
```

### Option 2: GitHub Actions Workflow (Advanced)

Create `.github/workflows/generate-gif.yml`:

```yaml
name: Generate Demo GIF

on:
  push:
    paths:
      - 'media/videos/*.mp4'

jobs:
  generate-gif:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Install ffmpeg
        run: sudo apt-get install -y ffmpeg
      
      - name: Convert video to GIF
        run: |
          ffmpeg -i media/videos/demo.mp4 \
            -vf "fps=10,scale=640:-1:flags=lanczos" \
            -loop 0 media/demo.gif
      
      - name: Commit GIF
        run: |
          git config --local user.email "action@github.com"
          git config --local user.name "GitHub Action"
          git add media/demo.gif
          git commit -m "Auto-generate demo GIF" || echo "No changes"
          git push
```

This will automatically generate a GIF whenever you push a new video to `/media/videos/`.
