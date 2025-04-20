# 🌍 Environment Surveillance Bot

An IoT-based, semi-autonomous surveillance bot designed to monitor environmental conditions like temperature, humidity, gas concentration, and obstacles—streaming real-time data and live video over Wi-Fi.

---

## 📷 Demo Preview

> Live sensor data + bot control via WebSocket  
> ESP32-CAM video streaming  
> Mobile & desktop friendly dashboard  

---

## 🛠️ Features

- Real-time temperature & humidity monitoring (DHT11)
- Gas leak/smoke detection (MQ2 sensor)
- Obstacle detection with ultrasonic sensors
- Motion control (forward, backward, left, right, stop)
- MPU6050-based orientation sensing
- Wi-Fi enabled bot control & data streaming
- Live video feed using ESP32-CAM
- Remote dashboard via WebSocket + HTML/CSS/JS frontend

---

## 🔋 Hardware Components

| Component           | Quantity | Description                                   |
|--------------------|----------|-----------------------------------------------|
| ESP32 Dev Board    | 1        | Main controller for sensors and motor control |
| ESP32-CAM Module   | 1        | For live video surveillance                   |
| L298N Motor Driver | 1        | Controls 2 DC motors                          |
| DHT11 Sensor       | 1        | Measures temperature & humidity               |
| MQ2 Gas Sensor     | 1        | Detects gas leakage / smoke                   |
| Ultrasonic Sensors | 3        | Obstacle detection (Front, Left, Right)       |
| MPU6050            | 1        | Measures tilt and orientation                 |
| 12V Battery        | 1        | Powers motors and ESP                         |
| Chassis + Wheels   | 1 set    | Mobile base of the bot                        |

---

## 🔌 Circuit Connections

### ESP32 Pin Mapping

| Sensor/Module       | ESP32 GPIO Pin |
|---------------------|----------------|
| DHT11               | GPIO 33        |
| MQ2 Analog Output   | GPIO 32        |
| Ultrasonic Front    | TRIG: GPIO 12, ECHO: GPIO 13 |
| Ultrasonic Left     | TRIG: GPIO 14, ECHO: GPIO 27 |
| Ultrasonic Right    | TRIG: GPIO 26, ECHO: GPIO 25 |
| L298N Motor IN1/IN2 | GPIO 16, GPIO 17 |
| L298N Motor IN3/IN4 | GPIO 5, GPIO 18  |
| L298N ENA/ENB       | GPIO 4, GPIO 19  |
| ESP32-CAM Video     | Runs separately, connects via WiFi |
| MPU6050 I2C         | SDA: GPIO 21, SCL: GPIO 22 |

> ⚠️ Use logic level shifters or voltage dividers where necessary.

---

## 📂 Project Structure

```bash
📁 Environment-Surveillance-Bot
├── 📁 ESP32-Code
│   ├── bot_controller.ino         # Main control code for ESP32
│   └── includes...                # Sensor libraries
├── 📁 ESP32-CAM
│   └── esp32-cam-stream.ino       # ESP32-CAM video streaming code
├── 📁 Web-Dashboard
│   └── index.html                 # HTML/CSS/JS for control panel
├── schematic.png                  # Circuit diagram
└── README.md                      # This file
