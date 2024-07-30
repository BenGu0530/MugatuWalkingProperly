#include <DynamixelShield.h>
#include <Wire.h>  
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoBLE.h>

const uint8_t legID = 1;  
const float DXL_PROTOCOL_VERSION = 2.0;
const double DEG_2_RAD = 0.01745329251;
const float pi = 3.14159;

DynamixelShield dxl;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

int startOffset = 800;
int GOAL_SPEED = 32;
float t_ime;

float f_left = 1.4;
float f_right = 1.4;

int A_left = 450;  
int A_right = 450;

float current_time;
float prev_time;
float current_pos;
float current_actual_pos;

String dataString;

float linear_accel_x, linear_accel_y, linear_accel_z;
float yaw, pitch, roll;

float deltaAmp = 0;
const float regularAmp = 480.0;

BLEService customService("180C");
BLEStringCharacteristic messageCharacteristic("2A37", BLERead | BLENotify, 100);

double xPos = 0, yPos = 0, headingVel = 0;
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
const uint16_t PRINT_DELAY_MS = 500;
uint16_t printCount = 0;

const double ACCEL_VEL_TRANSITION =  0.1;

const uint32_t BAUDRATE = 57600;
const uint32_t NEW_BAUDRATE = 1000000;

// Dynamic array to store yaw values
float averageYaw = 0;

bool firstStep = true;

float previousAverageYaw = 0;
float deltaAverageYaw = 0;
float accumulatedAverageYaw = 0;

float sumYaw = 0;
float yawCounter = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  Serial.println("Initializing...");

  dxl.begin(NEW_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(legID);

  dxl.torqueOff(legID);
  dxl.setOperatingMode(legID, OP_POSITION);
  dxl.torqueOn(legID);

  startOffset = dxl.getPresentPosition(legID);
  dxl.setGoalPosition(legID, startOffset);

  current_time = millis() / 1000;

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bno.setExtCrystalUse(true);
  Serial.println("bno.setExtCrystalUse(true)");

  BLE.begin();
  BLE.setLocalName("MKR1010");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(messageCharacteristic);
  BLE.addService(customService);
  Serial.println("BLE.addService(customService);");

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  digitalWrite(LED_BUILTIN,HIGH);

  BLEDevice central = BLE.central();

  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  linear_accel_x = linearAccelData.acceleration.x;
  linear_accel_y = linearAccelData.acceleration.y;
  linear_accel_z = linearAccelData.acceleration.z;

  yaw = orientationData.orientation.x;
  pitch = orientationData.orientation.y;
  roll = orientationData.orientation.z;

  // Ensure yaw is within the valid range of 0 to 359 degrees
  if (yaw < 0 || yaw >= 360) {
    yaw = 0; // Reset outlier values to 0
  }

  // Recalibrate yaw: left is -179 to -1, right is 1 to 179
  if (yaw > 180 && yaw < 360) {
    yaw = yaw - 360;
  }


  headingVel = ACCEL_VEL_TRANSITION * linear_accel_x / cos(DEG_2_RAD * yaw);


  sumYaw += yaw;
  yawCounter += 1;

  digitalWrite(LED_BUILTIN, LOW);


  t_ime = millis();
  float t = t_ime / 1000;  
  float pos = getJointAngle(t);
  
  dxl.setGoalPosition(legID, pos);

  prev_time = current_time;
  current_time = t;
  current_pos = pos;
  current_actual_pos = dxl.getPresentPosition(legID);


  if (central){
    String message = String(current_time) + "," +
                     String(current_pos) + "," +
                     String(current_actual_pos) + "," +
                     String(linear_accel_x) + "," +
                     String(linear_accel_y) + "," +
                     String(linear_accel_z) + "," +
                     String(yaw) + "," +
                     String(pitch) + "," +
                     String(roll) + "," +
                     String(f_left) + "," +
                     String(f_right) + "," +
                     String(A_left) + "," +
                     String(A_right);

    messageCharacteristic.writeValue(message);

  }
}

float getJointAngle(float t) {
  float phi = 0;
  float w_left = 2 * pi * f_left;
  float w_right = 2 * pi * f_right;
  float p_right = 1 / f_right;
  float p_left = 1 / f_left;
  float period_total = 1 / (2 * f_left) + 1 / (2 * f_right);
  float pos = 0;

  static float tau = fmod(t, period_total);
  static float prevTau;
  static bool wasLeftSwing = false;

  tau = fmod(t, period_total);

  if (tau < p_right / 2) {
    if (wasLeftSwing) {
      flat_controller();
      sumYaw = 0;
      yawCounter = 0;
      wasLeftSwing = false;
    }
    pos = A_right * sin(w_right * tau + phi) + startOffset;
  } else if (p_right / 2 <= tau && tau <= p_right / 2 + p_left / 2) {
    if (!wasLeftSwing) {
      flat_controller();
      sumYaw = 0;
      yawCounter = 0;
      wasLeftSwing = true;
    }
    pos = A_left * sin(w_left * (tau - p_right / 2 + p_left / 2) + phi) + startOffset;
  }
  prevTau = tau;
  return pos;
}

void flat_controller() {
  averageYaw = sumYaw/yawCounter;
  
  deltaAmp = 80;

  A_left = regularAmp - deltaAmp;
  A_right = regularAmp + deltaAmp;

  
}