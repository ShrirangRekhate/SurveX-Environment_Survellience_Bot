# 🌍 SureveX: Environment Surveillance Bot

An IoT-based, semi-autonomous surveillance bot designed to monitor environmental conditions like temperature, humidity, gas concentration, and obstacles—streaming real-time data and live video over Wi-Fi.

---

## 📷 Demo Preview

> ✅ Live sensor data + bot control via WebSocket  
> 🎥 ESP32-CAM video streaming  
> 📱 Mobile & 💻 desktop friendly dashboard  

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
```
## Working

The **Environmental Surveillance Bot** monitors various environmental parameters (e.g., temperature, humidity, air quality) and sends the data to a web dashboard in real-time. The bot uses an **ESP32** or **ESP32-CAM** for Wi-Fi connectivity, sensor data collection, and WebSocket communication. The bot also integrates a camera module for live surveillance.

### Overview of the Process:

1. **Set up the Environment**:
   - Clone the repository from GitHub.
   - Install the necessary libraries in the Arduino IDE or PlatformIO (e.g., `DHT`, `MQ2`, `WebSockets`).
   - Set up your WebSocket server for communication and ensure it is ready to receive data from the bot.

2. **Hardware Setup**:
   - Connect the sensors (DHT22 for temperature/humidity, MQ2 for gas detection, etc.) to the ESP32/ESP32-CAM as per the wiring instructions.
   - Optionally, connect the ESP32-CAM for live video surveillance.

3. **Modify Configuration**:
   - Update the **Wi-Fi credentials** in the code (`ssid`, `password`) to connect the ESP32 to your network.
   - Modify the **WebSocket server address** and port in the code to point to your WebSocket server.

4. **Upload the Code**:
   - Upload the code to your ESP32/ESP32-CAM via the Arduino IDE or PlatformIO.

5. **Monitoring**:
   - Once the ESP32 connects to Wi-Fi, it starts reading data from the sensors and sends the data to the WebSocket server.
   - The WebSocket server forwards the data to the connected clients (e.g., a web dashboard) in real-time.

6. **Camera and Video Streaming**:
   - The ESP32-CAM can stream live footage to your server or WebSocket server. If you're storing video, modify the code to upload camera snapshots or streams to your server or platform.

### Code Flow:
- **Wi-Fi Connection**: The ESP32 connects to the internet using the provided credentials.
- **Sensor Data Collection**: The code continuously reads data from the DHT22 and MQ2 sensors.
- **WebSocket Communication**: The data is sent to the WebSocket server, where it is forwarded to any connected clients in real-time.
- **Camera Integration (Optional)**: The ESP32-CAM captures video frames and streams them if configured.

### Customization:
- **Alert System**: Set thresholds for sensor data to trigger alerts, such as high CO levels, and send notifications via WebSockets to the connected clients.
- **Web Dashboard**: Integrate WebSocket data with a custom web dashboard to visualize the sensor readings in real-time.

---
## 👨‍💻 Author

# Shrirang Rekhate

Electronics and Telecommunication Engineering

Shri Guru Gobind Singhji Institute of Engineering and Technology, Nanded

GitHub: @ShrirangRekhate
