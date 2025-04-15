#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_VL53L0X.h>
#include <HMC5883L.h>
#include <OV2640.h>
#include <driver/adc.h>
#include <esp32-hal.h>

// Zigbee library (placeholder; use EByte E18 or CC2530 driver)
#include "Zigbee.h"

// Pin definitions
#define MOTOR1_PWM 12  // GPIO12-15 for motors
#define MOTOR2_PWM 13
#define MOTOR3_PWM 14
#define MOTOR4_PWM 15
#define ZIGBEE_TX 17   // Zigbee UART
#define ZIGBEE_RX 16
#define CAMERA_SDA 21  // I2C
#define CAMERA_SCL 22
#define BATTERY_ADC ADC1_CHANNEL_0  // GPIO0

// Constants
#define MASTER_ID 0x01  // M1 (M2: 0x02, GCS: 0x00, S1–S4: 0x11–0x14)
#define PAN_ID 0x1234   // Zigbee network
#define CHANNEL 15      // Zigbee channel
#define BAUD_RATE 115200
#define MAX_THRUST 180  // grams
#define DRONE_WEIGHT 65 // grams
#define BATTERY_CAP 1000 // mAh
#define VOLTAGE_FULL 4.35 // LiHV
#define VOLTAGE_LOW 3.5   // Cutoff
#define SLAVE_COUNT 4   // S1–S4

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox;
HMC5883L hmc;
OV2640 cam;

// Zigbee object
Zigbee zigbee(ZIGBEE_TX, ZIGBEE_RX);

// Motor PWM
const int motorChannels[4] = {0, 1, 2, 3};
const int motorPins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};

// PID constants
float Kp = 2.0, Ki = 0.05, Kd = 1.0;
float rollError = 0, pitchError = 0, yawError = 0;
float rollSum = 0, pitchSum = 0, yawSum = 0;
float lastRollError = 0, lastPitchError = 0, lastYawError = 0;
float targetRoll = 0, targetPitch = 0, targetYaw = 0;
float targetAltitude = 0;

// State variables
bool isFlying = false;
bool videoOn = false;
float batteryVoltage = 4.2;
float altitude = 0;
float distance = 0;
float heading = 0;
float roll = 0, pitch = 0, yaw = 0;
uint32_t lastSensorTime = 0;
uint32_t lastVideoTime = 0;
uint32_t lastAggregateTime = 0;

// Message structures
struct Command {
  uint8_t action; // 0: takeoff, 1: land, 2: waypoint, 3: hover, 4: video_on, 5: video_off
  float x, y, z;
  float speed;
  float yaw;
  uint8_t drone_id;
} cmd;

struct SensorData {
  uint8_t drone_id;
  float distance;
  float roll, pitch, yaw;
  float heading;
  float altitude;
  float battery;
} sensorData[SLAVE_COUNT + 1]; // M1 + S1–S4

void setup() {
  // Serial
  Serial.begin(BAUD_RATE);

  // I2C
  Wire.begin(CAMERA_SDA, CAMERA_SCL);

  // Sensors
  if (!mpu.begin()) { Serial.println("MPU6050 failed"); while (1); }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!bmp.begin()) { Serial.println("BMP280 failed"); while (1); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  if (!lox.begin()) { Serial.println("VL53L0X failed"); while (1); }
  lox.setMeasurementTimingBudget(20000); // 5 Hz

  if (!hmc.begin()) { Serial.println("QMC5883L failed"); while (1); }

  // Camera
  cam.begin();
  cam.setResolution(OV2640_320x240);
  cam.setCompression(OV2640_MJPEG);

  // Motors
  for (int i = 0; i < 4; i++) {
    ledcSetup(motorChannels[i], 1000, 8);
    ledcAttachPin(motorPins[i], motorChannels[i]);
    ledcWrite(motorChannels[i], 0);
  }

  // Zigbee
  zigbee.begin(BAUD_RATE);
  zigbee.setChannel(CHANNEL);
  zigbee.setPanId(PAN_ID);
  zigbee.startCoordinator(MASTER_ID);

  // ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BATTERY_ADC, ADC_ATTEN_DB_11);

  // Calibrate
  calibrateSensors();

  // Initialize master data
  sensorData[0].drone_id = MASTER_ID;

  Serial.println("Master M1 ready");
}

void loop() {
  // Read own sensors (~2 Hz)
  if (millis() - lastSensorTime >= 500) {
    readSensors();
    lastSensorTime = millis();
  }

  // Aggregate slave data (~1 Hz)
  if (millis() - lastAggregateTime >= 1000) {
    aggregateSensorData();
    sendAggregatedData();
    lastAggregateTime = millis();
  }

  // Check Zigbee for commands/ACKs
  if (zigbee.available()) {
    processZigbee();
  }

  // Stream video if ON (~5 FPS)
  if (videoOn && millis() - lastVideoTime >= 200) {
    sendVideoFrame();
    lastVideoTime = millis();
  }

  // Update flight
  if (isFlying) {
    updatePID();
    adjustMotors();
  }

  // Battery
  updateBattery();

  // Coordinate swarm
  coordinateSwarm();
}

void calibrateSensors() {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax += a.acceleration.x;
    ay += a.acceleration.y;
    az += a.acceleration.z;
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;
    delay(10);
  }
  ax /= 100; ay /= 100; az /= 100; gx /= 100; gy /= 100; gz /= 100;
}

void readSensors() {
  // MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  yaw += g.gyro.z * 0.02;

  // QMC5883L
  Vector mag = hmc.readNormalize();
  heading = atan2(mag.YAxis, mag.XAxis) * 180 / PI;
  if (heading < 0) heading += 360;

  // BMP280
  altitude = bmp.readAltitude(1013.25);

  // VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  distance = measure.RangeMilliMeter / 1000.0;

  // Store
  sensorData[0].distance = distance;
  sensorData[0].roll = roll;
  sensorData[0].pitch = pitch;
  sensorData[0].yaw = yaw;
  sensorData[0].heading = heading;
  sensorData[0].altitude = altitude;
  sensorData[0].battery = batteryVoltage;
}

void processZigbee() {
  uint8_t buffer[10240];
  uint16_t len = zigbee.receive(buffer, sizeof(buffer));
  if (len == 0) return;

  // Command from GCS
  if (buffer[0] == 0xBB && (buffer[1] == MASTER_ID || buffer[1] >= 0x11)) {
    if (buffer[1] == MASTER_ID) {
      cmd.action = buffer[3];
      memcpy(&cmd.x, &buffer[4], 4);
      memcpy(&cmd.y, &buffer[8], 4);
      memcpy(&cmd.z, &buffer[12], 4);
      memcpy(&cmd.speed, &buffer[16], 4);
      memcpy(&cmd.yaw, &buffer[20], 4);
      cmd.drone_id = buffer[24];
      uint16_t crc = computeCRC(buffer, 29);
      if (buffer[29] != (crc & 0xFF)) return;

      // Process for self
      switch (cmd.action) {
        case 0: // Takeoff
          targetAltitude = cmd.z;
          isFlying = true;
          break;
        case 1: // Land
          targetAltitude = 0;
          isFlying = false;
          break;
        case 2: // Waypoint
          targetRoll = cmd.x;
          targetPitch = cmd.y;
          targetYaw = cmd.yaw;
          targetAltitude = cmd.z;
          isFlying = true;
          break;
        case 3: // Hover
          targetRoll = 0;
          targetPitch = 0;
          targetAltitude = altitude;
          isFlying = true;
          break;
        case 4: // Video ON
          if (cmd.drone_id == MASTER_ID) {
            videoOn = true;
            cam.start();
          }
          break;
        case 5: // Video OFF
          if (cmd.drone_id == MASTER_ID) {
            videoOn = false;
            cam.stop();
          }
          break;
      }

      // Send ACK
      uint8_t ack[10] = {0xCC, MASTER_ID, 0x00, cmd.action};
      zigbee.send(ack, 10, 0x00);
    }

    // Forward to slave
    if (buffer[1] >= 0x11 && buffer[1] <= 0x14) {
      zigbee.send(buffer, 30, buffer[1]);
    }
  }

  // Sensor data from slave
  if (buffer[0] == 0xAA && buffer[1] >= 0x11 && buffer[1] <= 0x14) {
    int idx = buffer[1] - 0x10; // S1=1, S2=2, etc.
    sensorData[idx].drone_id = buffer[1];
    memcpy(&sensorData[idx].distance, &buffer[3], 4);
    memcpy(&sensorData[idx].roll, &buffer[7], 4);
    memcpy(&sensorData[idx].pitch, &buffer[11], 4);
    memcpy(&sensorData[idx].yaw, &buffer[15], 4);
    memcpy(&sensorData[idx].heading, &buffer[19], 4);
    memcpy(&sensorData[idx].altitude, &buffer[23], 4);
    memcpy(&sensorData[idx].battery, &buffer[27], 4);
    uint16_t crc = computeCRC(buffer, 29);
    if (buffer[29] != (crc & 0xFF)) return;
  }

  // Video from slave
  if (buffer[0] == 0xDD && buffer[1] >= 0x11 && buffer[1] <= 0x14) {
    uint32_t frameSize;
    memcpy(&frameSize, &buffer[3], 4);
    if (frameSize > 10000) return;
    uint16_t crc = computeCRC(buffer, frameSize + 7);
    if (buffer[frameSize + 7] != (crc & 0xFF)) return;
    zigbee.send(buffer, frameSize + 8, 0x00); // Relay to GCS
  }

  // ACK from slave
  if (buffer[0] == 0xCC && buffer[1] >= 0x11 && buffer[1] <= 0x14) {
    // Log or process ACK (e.g., confirm S1 got command)
  }
}

void aggregateSensorData() {
  // Combine M1 + S1–S4 data (~100 bytes)
  // Already stored in sensorData[0–4]
}

void sendAggregatedData() {
  // Pack (~100 bytes)
  uint8_t buffer[100];
  buffer[0] = 0xEE; // Aggregated data
  buffer[1] = MASTER_ID;
  buffer[2] = 0x00; // GCS
  int offset = 3;
  for (int i = 0; i <= SLAVE_COUNT; i++) {
    buffer[offset] = sensorData[i].drone_id;
    memcpy(&buffer[offset + 1], &sensorData[i].distance, 4);
    memcpy(&buffer[offset + 5], &sensorData[i].roll, 4);
    memcpy(&buffer[offset + 9], &sensorData[i].pitch, 4);
    memcpy(&buffer[offset + 13], &sensorData[i].yaw, 4);
    memcpy(&buffer[offset + 17], &sensorData[i].heading, 4);
    memcpy(&buffer[offset + 21], &sensorData[i].altitude, 4);
    memcpy(&buffer[offset + 25], &sensorData[i].battery, 4);
    offset += 29;
  }
  uint16_t crc = computeCRC(buffer, offset);
  buffer[offset] = crc & 0xFF;

  // Send to GCS
  zigbee.send(buffer, offset + 1, 0x00);
}

void sendVideoFrame() {
  uint8_t* frame = cam.capture();
  uint32_t frameSize = cam.getFrameSize();
  if (frameSize > 10000) frameSize = 10000;

  uint8_t buffer[10240];
  buffer[0] = 0xDD;
  buffer[1] = MASTER_ID;
  buffer[2] = 0x00;
  memcpy(&buffer[3], &frameSize, 4);
  memcpy(&buffer[7], frame, frameSize);
  uint16_t crc = computeCRC(buffer, frameSize + 7);
  buffer[frameSize + 7] = crc & 0xFF;

  zigbee.send(buffer, frameSize + 8, 0x00);
}

void coordinateSwarm() {
  // Check for obstacles (e.g., distance < 0.3 m)
  for (int i = 0; i <= SLAVE_COUNT; i++) {
    if (sensorData[i].distance < 0.3 && sensorData[i].drone_id != 0) {
      // Send avoidance command
      uint8_t buffer[30];
      buffer[0] = 0xBB;
      buffer[1] = sensorData[i].drone_id;
      buffer[2] = MASTER_ID;
      buffer[3] = 2; // Waypoint (simplified avoidance)
      float x = sensorData[i].roll + 0.5; // Move right (example)
      memcpy(&buffer[4], &x, 4);
      float y = sensorData[i].pitch;
      memcpy(&buffer[8], &y, 4);
      float z = sensorData[i].altitude;
      memcpy(&buffer[12], &z, 4);
      float speed = 0.5;
      memcpy(&buffer[16], &speed, 4);
      float yaw = sensorData[i].yaw;
      memcpy(&buffer[20], &yaw, 4);
      buffer[24] = sensorData[i].drone_id;
      uint16_t crc = computeCRC(buffer, 29);
      buffer[29] = crc & 0xFF;

      zigbee.send(buffer, 30, sensorData[i].drone_id);
    }
  }
}

void updatePID() {
  rollError = targetRoll - roll;
  pitchError = targetPitch - pitch;
  yawError = targetYaw - yaw;
  float altError = targetAltitude - altitude;

  rollSum += rollError * 0.02;
  pitchSum += pitchError * 0.02;
  yawSum += yawError * 0.02;

  float rollDeriv = (rollError - lastRollError) / 0.02;
  float pitchDeriv = (pitchError - lastPitchError) / 0.02;
  float yawDeriv = (yawError - lastYawError) / 0.02;

  float rollOut = Kp * rollError + Ki * rollSum + Kd * rollDeriv;
  float pitchOut = Kp * pitchError + Ki * pitchSum + Kd * pitchDeriv;
  float yawOut = Kp * yawError + Ki * yawSum + Kd * yawDeriv;
  float altOut = Kp * altError;

  float baseThrottle = altOut * 255 / MAX_THRUST;
  float m1 = baseThrottle + rollOut - pitchOut + yawOut;
  float m2 = baseThrottle - rollOut - pitchOut - yawOut;
  float m3 = baseThrottle - rollOut + pitchOut + yawOut;
  float m4 = baseThrottle + rollOut + pitchOut - yawOut;

  m1 = constrain(m1, 0, 255);
  m2 = constrain(m2, 0, 255);
  m3 = constrain(m3, 0, 255);
  m4 = constrain(m4, 0, 255);

  ledcWrite(motorChannels[0], m1);
  ledcWrite(motorChannels[1], m2);
  ledcWrite(motorChannels[2], m3);
  ledcWrite(motorChannels[3], m4);

  lastRollError = rollError;
  lastPitchError = pitchError;
  lastYawError = yawError;
}

void adjustMotors() {
  if (!isFlying) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(motorChannels[i], 0);
    }
  }
}

void updateBattery() {
  int raw = adc1_get_raw(BATTERY_ADC);
  batteryVoltage = raw * (VOLTAGE_FULL / 4095.0);
  if (batteryVoltage < VOLTAGE_LOW) {
    isFlying = false;
    videoOn = false;
    cam.stop();
    for (int i = 0; i < 4; i++) {
      ledcWrite(motorChannels[i], 0);
    }
  }
  sensorData[0].battery = batteryVoltage;
}

uint16_t computeCRC(uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
