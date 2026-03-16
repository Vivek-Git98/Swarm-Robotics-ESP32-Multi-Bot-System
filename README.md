# 🤖 Swarm Robotics — ESP32 Multi-Bot System

> Three-robot swarm where two slave bots autonomously follow the master bot's path,  
> with real-time wireless coordination via ESP-NOW and a Python PC control GUI.

---

## 📸 Demo

<!-- Replace with your actual photo/video paths after uploading -->
| Bot Hardware | Formation in Action |
|:---:|:---:|
| ![Bot Photo](media/photos/bot_overview.jpg) | ![Formation](media/photos/formation.jpg) |

🎥 **[Watch Demo Video](media/videos/swarm_demo.mp4)**

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────┐
│              PC  (Python GUI)                   │
│    Manual control  ◄──► Serial USB              │
└────────────────────┬────────────────────────────┘
                     │ USB Serial
                     ▼
         ┌─────────────────────┐
         │   Master Bot (1)    │  ← You drive this
         │   Records path      │
         └────────┬────────────┘
                  │ ESP-NOW (2.4 GHz)
          ┌───────┴───────┐
          ▼               ▼
  ┌──────────────┐  ┌──────────────┐
  │  Slave Bot 2 │  │  Slave Bot 3 │
  │  Follows     │  │  Follows     │
  │  waypoints   │  │  waypoints   │
  └──────────────┘  └──────────────┘
```

Slaves maintain a **waypoint delay** so Bot 2 follows Bot 1's trail by ~2 waypoints,  
and Bot 3 follows 4 waypoints behind — keeping them evenly spaced.

---

## 🔧 Hardware Per Bot

| Component | Spec / Part |
|---|---|
| Microcontroller | ESP32 (30-pin DevKit) |
| Motor Driver | L298N or TB6612FNG |
| Drive Motors | DC encoder motors × 2 |
| Ultrasonic Sensors | HC-SR04 × 2 (front + back) |
| Servo Motors | SG90 × 2 (one per sensor) |
| Magnetometer | QMC5883L (I²C) |
| Power | 5V LiPo + LM2596 Buck Converter |
| Chassis | Custom 2-wheel differential drive |

---

## 📌 Wiring / Pin Map

| Signal | ESP32 Pin |
|---|---|
| Motor EN_A (PWM) | GPIO 27 |
| Motor IN1 / IN2 | GPIO 14 / 32 |
| Motor EN_B (PWM) | GPIO 13 |
| Motor IN3 / IN4 | GPIO 4 / 15 |
| Encoder Left A | GPIO 34 |
| Encoder Right A | GPIO 36 |
| Ultrasonic Front TRIG/ECHO | GPIO 5 / 18 |
| Ultrasonic Back TRIG/ECHO | GPIO 25 / 26 |
| Servo Front | GPIO 33 |
| Servo Back | GPIO 12 |
| Magnetometer SDA/SCL | GPIO 21 / 22 |

---

## 📁 Repository Structure

```
swarm-robotics/
├── firmware/
│   ├── master/
│   │   └── master.ino          ← Flash to Bot 1
│   └── slave/
│       └── slave.ino           ← Flash to Bot 2 & 3 (change BOT_ID)
├── gui/
│   └── swarm_control.py        ← Python PC control panel
├── docs/
│   └── GITHUB_GUIDE.md         ← How to upload this repo
├── media/
│   ├── photos/                 ← Put your bot photos here
│   └── videos/                 ← Put your demo video here
├── .gitignore
└── README.md
```

---

## 🚀 Quick Start

### 1. Flash the firmware

**Dependencies (Arduino IDE Library Manager):**
- `ESP32Servo`
- `QMC5883LCompass`
- Board: `esp32` by Espressif (Board Manager)

**Steps:**
1. Open `firmware/master/master.ino` in Arduino IDE
2. Update the **MAC addresses** at the top to match your bots
3. Select board: `ESP32 Dev Module`, upload to Bot 1
4. Open `firmware/slave/slave.ino`, set `#define BOT_ID 2`, upload to Bot 2
5. Change `#define BOT_ID 3` and `#define FOLLOW_DELAY 4`, upload to Bot 3

#### Finding your bot's MAC address
Flash this one-liner to each ESP32 before uploading the main firmware:
```cpp
#include <WiFi.h>
void setup() { Serial.begin(115200); WiFi.mode(WIFI_STA); Serial.println(WiFi.macAddress()); }
void loop() {}
```

### 2. Run the Python GUI

```bash
pip install pyserial
python gui/swarm_control.py
```

1. Connect Master Bot (Bot 1) to PC via USB
2. Select COM port in the GUI → click **CONNECT**
3. Use **WASD** keys or on-screen buttons to drive
4. Click **AUTO-FOLLOW** — slaves will begin following the master's path

---

## 🎮 GUI Controls

| Key / Button | Action |
|---|---|
| W / ▲ | Forward |
| S / ▼ | Backward |
| A / ◄ | Turn Left |
| D / ► | Turn Right |
| Space / ■ | Stop |
| AUTO-FOLLOW | Slaves begin path following |
| MANUAL | Return to joystick control |
| Speed slider | 60 – 255 PWM |

---

## ⚠️ Calibration

Before running, calibrate the **QMC5883L magnetometer** for your environment:

1. Upload compass calibration sketch and rotate the bot slowly in all axes
2. Note min/max for X, Y, Z
3. Update in firmware:
```cpp
compass.setCalibration(xMin, xMax, yMin, yMax, zMin, zMax);
```

---

## 🐛 Known Limitations

- Odometry drifts over long distances (no absolute positioning)
- ESP-NOW range ~50–100m line-of-sight; walls reduce it
- Emergency stop uses a brief blocking delay — will be replaced with non-blocking in v2

---

## 👤 Author

**[Your Name]**  
B.Tech / Department — [Your College]  
[Your LinkedIn / email]

---

## 📄 License

MIT License — see [LICENSE](LICENSE)
