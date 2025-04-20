#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
//#include <MQUnifiedsensor.h>
#include <MQ2.h>
#include <Bonezegei_DHT11.h>

// ==================== WiFi Credentials ====================
#define WIFI_SSID        "$HREE"
#define WIFI_PASSWORD    "12345678900"

// ==================== Server Instances ====================
WebServer httpServer(80);                // HTTP server (optional)
WebSocketsServer webSocket(81);         // WebSocket server on port 81

// ==================== Sensor & Module Instances ====================
MPU6050 mpu6050(Wire);                  // MPU6050 sensor via I2C
#define MQ2_PIN              32
#define MQ2_BOARD            "ESP32"
#define MQ2_TYPE             "MQ-2"
#define VOLTAGE_RESOLUTION   3.3
#define ADC_RESOLUTION       12
#define RATIO_CLEAN_AIR      9.83
//MQUnifiedsensor MQ2(MQ2_BOARD, VOLTAGE_RESOLUTION, ADC_RESOLUTION, MQ2_PIN, MQ2_TYPE);
MQ2 MQ2(MQ2_PIN);

#define DHTPIN 33
Bonezegei_DHT11 dht(DHTPIN);

// ==================== Ultrasonic Sensor Pins ====================
const int trigFront = 12, echoFront = 13;
const int trigLeft = 14, echoLeft = 27;
const int trigRight = 26, echoRight = 25;

// ==================== Motor Driver Pins (L298N) ====================
const int IN1 = 16, IN2 = 17, IN3 = 5, IN4 = 18;
const int ENA = 4, ENB = 19;

// PWM configuration
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1
#define PWM_FREQ       1000
#define PWM_RES        8

int motorSpeed = 80; // Default speed

// ==================== Timing ====================
unsigned long prevSensorMillis = 0;
const unsigned long sensorInterval = 1000;  // 1 second interval

// ==================== Data Structures ====================
struct MPUData {
  float angleX, angleY, angleZ;
};
struct DHTData {
  float tempC, tempF;
  int humidity;
};
struct MQ2Data {
  float lpg, co, smoke;
};

// ==================== Function Prototypes ====================
MPUData getMPUData();
DHTData getDHTData();
MQ2Data readMQ2Sensor();
long measureDistance(int trigPin, int echoPin);
void setupMotorPWM();
void motorForward();
void motorBackward();
void motorLeft();
void motorRight();
void motorStop();
void calibrateMQ2();

// ==================== WebSocket Event Handler ====================
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected from %s\n", client_num, webSocket.remoteIP(client_num).toString().c_str());
      webSocket.sendTXT(client_num, "Connected to ESP32!");
      break;

    case WStype_TEXT: {
        String cmd = String((char*)payload).substring(0, length);
        Serial.printf("[%u] Received command: %s\n", client_num, cmd.c_str());

        if      (cmd == "forward")   motorForward();
        else if (cmd == "backward")  motorBackward();
        else if (cmd == "left")      motorLeft();
        else if (cmd == "right")     motorRight();
        else if (cmd == "stop")      motorStop();
        else if (cmd.startsWith("speed:")) {
          motorSpeed = cmd.substring(6).toInt();
          Serial.printf("Motor speed set to: %d\n", motorSpeed);
        }
        break;
      }

    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected\n", client_num);
      motorStop(); // Safety
      break;
  }
}

// ==================== HTTP Test Page ====================
void handleRoot() {
  httpServer.send(200, "text/html", "<html><body><h2>ESP32 WebSocket Server Running</h2></body></html>");
}

// ==================== Setup ====================
float errorX, errorY, errorZ;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize sensors
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  errorX = mpu6050.getAngleX();
  errorY = mpu6050.getAngleY();
  errorZ = mpu6050.getAngleZ();

  dht.begin();

  // Ultrasonic pins
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  // Motor driver pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  setupMotorPWM();
  motorStop();

  // WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  // HTTP & WebSocket server
  httpServer.on("/", handleRoot);
  httpServer.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // MQ-2 sensor setup
  /*Serial.println("MQ-2 Sensor Calibration");
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25); MQ2.setB(-2.222);
  MQ2.init();
  calibrateMQ2();
  MQ2.serialDebug(true);*/
MQ2.begin();

}

// ==================== Main Loop ====================
void loop() {
  httpServer.handleClient();
  webSocket.loop();

  if (millis() - prevSensorMillis >= sensorInterval) {
    prevSensorMillis = millis();

    MPUData mpu = getMPUData();
    MQ2Data mq2 = readMQ2Sensor();
    DHTData dhtData = getDHTData();
    long distFront = measureDistance(trigFront, echoFront);
    long distLeft  = measureDistance(trigLeft, echoLeft);
    long distRight = measureDistance(trigRight, echoRight);

    String json = "{";
    json += "\"mpu\":{\"angleX\":" + String(mpu.angleX, 2) +
            ",\"angleY\":" + String(mpu.angleY, 2) +
            ",\"angleZ\":" + String(mpu.angleZ, 2) + "},";
    json += "\"mq2\":{\"lpg\":" + String(mq2.lpg, 2) +
            ",\"co\":" + String(mq2.co, 2) +
            ",\"smoke\":" + String(mq2.smoke, 2) + "},";
    json += "\"dht\":{\"tempC\":" + String(dhtData.tempC, 2) +
            ",\"tempF\":" + String(dhtData.tempF, 2) +
            ",\"humidity\":" + String(dhtData.humidity) + "},";
    json += "\"ultrasonic\":{\"front\":" + String(distFront) +
            ",\"left\":" + String(distLeft) +
            ",\"right\":" + String(distRight) + "}";
    json += "}";

    webSocket.broadcastTXT(json);
    if(distFront<10){motorStop();}
  }
}
  
// ==================== Sensor Functions ====================
MPUData getMPUData() {
  mpu6050.update();
  return {
    mpu6050.getAngleX() - errorX,
    mpu6050.getAngleY() - errorY,
    mpu6050.getAngleZ() - errorZ
  };
}

DHTData getDHTData() {
  if (dht.getData()) {
    return {
      dht.getTemperature(),
      dht.getTemperature(true),
      dht.getHumidity()
    };
  }
  return { 0, 0, 0 };
}

MQ2Data readMQ2Sensor() {
float lpg = MQ2.readLPG();
float co = MQ2.readCO();
float smoke = MQ2.readSmoke();
return { lpg, co, smoke };

}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ==================== Motor Control ====================
void setupMotorPWM() {
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  Serial.println("PWM channels initialized");
}

void motorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, motorSpeed);
  ledcWrite(PWM_CHANNEL_B, motorSpeed);
  //Serial.println("MOTOR: FORWARD");
}

void motorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_A, motorSpeed);
  ledcWrite(PWM_CHANNEL_B, motorSpeed);
  //Serial.println("MOTOR: BACKWARD");
}

void motorLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_A, motorSpeed);
  ledcWrite(PWM_CHANNEL_B, motorSpeed);
  //Serial.println("MOTOR: LEFT");
}

void motorRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, motorSpeed);
  ledcWrite(PWM_CHANNEL_B, motorSpeed);
  //Serial.println("MOTOR: RIGHT");
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  // Serial.println("MOTOR: STOP");
}

// ==================== MQ-2 Calibration ====================
/*void calibrateMQ2() {
  Serial.print("Calibrating MQ-2 sensor, please wait");
  float calcR0 = 0;
  for (int i = 0; i < 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RATIO_CLEAN_AIR);
    Serial.print(".");
    delay(1000);
  }
  calcR0 /= 10;
  MQ2.setR0(calcR0);
  Serial.println(" Calibration done!");

  if (isinf(calcR0)) {
    Serial.println("Warning: R0 is infinite. Check wiring.");
    while (true);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: R0 is zero. Check wiring.");
    while (true);
  }
}*/
