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
#define MOTOR1_PWM 12  // GPIO12-15 for 4 motors
#define MOTOR2_PWM 13
#define MOTOR3_PWM 14
#define MOTOR4_PWM 15
#define ZIGBEE_TX 17   // Zigbee UART (TX, RX)
#define ZIGBEE_RX 16
#define CAMERA_SDA 21  // Camera I2C (shared with sensors)
#define CAMERA_SCL 22
#define BATTERY_ADC ADC1_CHANNEL_0  // GPIO0 for battery voltage

// Constants
#define SLAVE_ID 0x11  // S1 (M1: 0x01, M2: 0x02, GCS: 0x00, S2: 0x12, etc.)
#define PAN_ID 0x1234  // Zigbee network ID
#define CHANNEL 15     // Zigbee channel
#define BAUD_RATE 115200
#define MAX_THRUST 180 // grams (4x 45 gm)
#define DRONE_WEIGHT 65 // grams
#define BATTERY_CAP 1000 // mAh
#define VOLTAGE_FULL 4.35 // LiHV
#define VOLTAGE_LOW 3.5   // Cutoff

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox;
HMC5883L hmc;
OV2640 cam;

// Zigbee object (placeholder)
Zigbee zigbee(ZIGBEE_TX, ZIGBEE_RX);

// Motor PWM channels
const int motorChannels[4] = {0, 1, 2, 3};
const int motorPins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};

// PID constants (tuned for stability)
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

// Message buffers
struct Command {
  uint8_t action; // 0: takeoff, 1: land, 2: waypoint, 3: hover, 4: video_on, 5: video_off
  float x, y, z;
  float speed;
  float yaw;
  uint8_t drone_id;
} cmd;

struct SensorData {
  float distance;
  float roll, pitch, yaw;
  float heading;
  float altitude;
  float battery;
} sensorData;

void setup() {
  // Initialize serial for debug
  Serial.begin(BAUD_RATE);

  // Initialize I2C
  Wire.begin(CAMERA_SDA, CAMERA_SCL);

  // Initialize sensors
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

  // Initialize camera
  cam.begin();
  cam.setResolution(OV2640_320x240);
  cam.setCompression(OV2640_MJPEG);

  // Initialize motors
  for (int i = 0; i < 4; i++) {
    ledcSetup(motorChannels[i], 1000, 8); // 1 kHz, 8-bit
    ledcAttachPin(motorPins[i], motorChannels[i]);
    ledcWrite(motorChannels[i], 0); // Off
  }

  // Initialize Zigbee
  zigbee.begin(BAUD_RATE);
  zigbee.setChannel(CHANNEL);
  zigbee.setPanId(PAN_ID);
  zigbee.joinNetwork(SLAVE_ID);

  // Initialize ADC for battery
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BATTERY_ADC, ADC_ATTEN_DB_11);

  // Calibrate sensors
  calibrateSensors();

  Serial.println("Slave S1 ready");
}

void loop() {
  // Read sensors (~2 Hz)
  if (millis() - lastSensorTime >= 500) {
    readSensors();
    sendSensorData();
    lastSensorTime = millis();
  }

  // Check Zigbee for commands
  if (zigbee.available()) {
    processCommand();
  }

  // Stream video if ON (~5 FPS)
  if (videoOn && millis() - lastVideoTime >= 200) {
    sendVideoFrame();
    lastVideoTime = millis();
  }

  // Update flight control
  if (isFlying) {
    updatePID();
    adjustMotors();
  }

  // Monitor battery
  updateBattery();
}

void calibrateSensors() {
  // MPU6050: Average 100 readings for offset
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
  // Store offsets (simplified, adjust in production)
  ax /= 100; ay /= 100; az /= 100; gx /= 100; gy /= 100; gz /= 100;
}

void readSensors() {
  // MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  yaw += g.gyro.z * 0.02; // Integrate gyro (~50 Hz loop)

  // QMC5883L
  Vector mag = hmc.readNormalize();
  heading = atan2(mag.YAxis, mag.XAxis) * 180 / PI;
  if (heading < 0) heading += 360;

  // BMP280
  altitude = bmp.readAltitude(1013.25); // Sea-level pressure

  // VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  distance = measure.RangeMilliMeter / 1000.0; // meters

  // Update sensor data
  sensorData.distance = distance;
  sensorData.roll = roll;
  sensorData.pitch = pitch;
  sensorData.yaw = yaw;
  sensorData.heading = heading;
  sensorData.altitude = altitude;
  sensorData.battery = batteryVoltage;
}

void sendSensorData() {
  // Pack data (~30 bytes)
  uint8_t buffer[30];
  buffer[0] = 0xAA; // Start byte
  buffer[1] = SLAVE_ID;
  buffer[2] = 0x00; // GCS ID
  memcpy(&buffer[3], &sensorData.distance, 4);
  memcpy(&buffer[7], &sensorData.roll, 4);
  memcpy(&buffer[11], &sensorData.pitch, 4);
  memcpy(&buffer[15], &sensorData.yaw, 4);
  memcpy(&buffer[19], &sensorData.heading, 4);
  memcpy(&buffer[23], &sensorData.altitude, 4);
  memcpy(&buffer[27], &sensorData.battery, 4);
  uint16_t crc = computeCRC(buffer, 29);
  buffer[29] = crc & 0xFF;

  // Send to master (e.g., M1 for S1)
  zigbee.send(buffer, 30, 0x01);
}

void processCommand() {
  uint8_t buffer[30];
  if (zigbee.receive(buffer, 30)) {
    if (buffer[0] != 0xBB || buffer[1] != SLAVE_ID) return; // Validate
    cmd.action = buffer[3];
    memcpy(&cmd.x, &buffer[4], 4);
    memcpy(&cmd.y, &buffer[8], 4);
    memcpy(&cmd.z, &buffer[12], 4);
    memcpy(&cmd.speed, &buffer[16], 4);
    memcpy(&cmd.yaw, &buffer[20], 4);
    cmd.drone_id = buffer[24];

    // Verify CRC
    uint16_t crc = computeCRC(buffer, 29);
    if (buffer[29] != (crc & 0xFF)) return;

    // Process command
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
        targetRoll = cmd.x; // Simplified (x -> roll for demo)
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
        if (cmd.drone_id == SLAVE_ID) {
          videoOn = true;
          cam.start();
        }
        break;
      case 5: // Video OFF
        if (cmd.drone_id == SLAVE_ID) {
          videoOn = false;
          cam.stop();
        }
        break;
    }

    // Send ACK
    uint8_t ack[10] = {0xCC, SLAVE_ID, 0x01, cmd.action};
    zigbee.send(ack, 10, 0x01);
  }
}

void sendVideoFrame() {
  // Capture MJPEG frame (~10 KB)
  uint8_t* frame = cam.capture();
  uint32_t frameSize = cam.getFrameSize();
  if (frameSize > 10000) frameSize = 10000; // Cap at 10 KB

  // Packetize (~5–10 KB)
  uint8_t buffer[10240];
  buffer[0] = 0xDD; // Video start
  buffer[1] = SLAVE_ID;
  buffer[2] = 0x00; // GCS
  memcpy(&buffer[3], &frameSize, 4);
  memcpy(&buffer[7], frame, frameSize);
  uint16_t crc = computeCRC(buffer, frameSize + 7);
  buffer[frameSize + 7] = crc & 0xFF;

  // Send to GCS via master
  zigbee.send(buffer, frameSize + 8, 0x01);
}

void updatePID() {
  // Errors
  rollError = targetRoll - roll;
  pitchError = targetPitch - pitch;
  yawError = targetYaw - yaw;
  float altError = targetAltitude - altitude;

  // Integral
  rollSum += rollError * 0.02; // ~50 Hz
  pitchSum += pitchError * 0.02;
  yawSum += yawError * 0.02;

  // Derivative
  float rollDeriv = (rollError - lastRollError) / 0.02;
  float pitchDeriv = (pitchError - lastPitchError) / 0.02;
  float yawDeriv = (yawError - lastYawError) / 0.02;

  // PID outputs
  float rollOut = Kp * rollError + Ki * rollSum + Kd * rollDeriv;
  float pitchOut = Kp * pitchError + Ki * pitchSum + Kd * pitchDeriv;
  float yawOut = Kp * yawError + Ki * yawSum + Kd * yawDeriv;
  float altOut = Kp * altError; // Simplified altitude PID

  // Update motors
  float baseThrottle = altOut * 255 / MAX_THRUST; // Scale to PWM
  float m1 = baseThrottle + rollOut - pitchOut + yawOut;
  float m2 = baseThrottle - rollOut - pitchOut - yawOut;
  float m3 = baseThrottle - rollOut + pitchOut + yawOut;
  float m4 = baseThrottle + rollOut + pitchOut - yawOut;

  // Constrain
  m1 = constrain(m1, 0, 255);
  m2 = constrain(m2, 0, 255);
  m3 = constrain(m3, 0, 255);
  m4 = constrain(m4, 0, 255);

  // Apply
  ledcWrite(motorChannels[0], m1);
  ledcWrite(motorChannels[1], m2);
  ledcWrite(motorChannels[2], m3);
  ledcWrite(motorChannels[3], m4);

  // Store errors
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
  batteryVoltage = raw * (VOLTAGE_FULL / 4095.0); // 12-bit ADC
  if (batteryVoltage < VOLTAGE_LOW) {
    isFlying = false;
    videoOn = false;
    cam.stop();
    for (int i = 0; i < 4; i++) {
      ledcWrite(motorChannels[i], 0);
    }
  }
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
