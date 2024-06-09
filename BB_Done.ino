#include <DynamixelShield.h>
#include <Wire.h>  // This library allows you to communicate with I2C devices.
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "I2Cdev.h"

int counter = 0;
float pi = 3.14159;
const uint8_t legID = 1;  // Default ID is 1
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

int startOffset = 800;  // Original 820
int GOAL_SPEED = 32;
float t_ime;

// Frequency for each leg
float f_left = 1.40;
float f_right = 1.40;

// Amplitude for sinusoid
float A_left = 450;  // Starting point at 440 (4096 clicks per revolution)
float A_right = 450;

const int chipSelect = SDCARD_SS_PIN;

String expDate = "Ju";
String fileName;
String expNum = "a";

// Vectors for experimental data
float current_time;
float prev_time;
float current_pos;
float current_velocity;
float current_actual_pos;
float current_current;
float current_PWM;

bool printHeaders = true;

String dataString;
File dataFile;

float ax, ay, az;
float gx, gy, gz;

float reasonable_alpha_z_range = 10.0;
float deltaAmp = 80;
float regularAmp = 450.0;
float smallAmp = regularAmp - deltaAmp;
float bigAmp = regularAmp + deltaAmp;

bool get_back_from_left;
bool get_back_from_right;
bool on_track;

bool ledFlashed = false;  // Add this at the beginning of your code

// Define a buffer structure to hold the data
struct WriteData {
  float current_time;
  float current_pos;
  float current_actual_pos;
  float current_velocity;
  float ax, ay, az;
  float gx, gy, gz;
  float A_left, A_right;
  float f_left, f_right;
};

const int bufferSize = 1;
WriteData dataBuffer[bufferSize];
int bufferIndex = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

  int trialNum = 0;
  do {
    trialNum++;
    fileName = expDate + expNum + (String)trialNum + ".txt";
  } while (SD.exists(fileName));

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(legID);

  dxl.torqueOff(legID);
  dxl.setOperatingMode(legID, OP_POSITION);
  dxl.torqueOn(legID);

  startOffset = dxl.getPresentPosition(legID);
  dxl.setGoalPosition(legID, startOffset);
  // delay(20);

  current_time = millis() / 1000;

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bno.setExtCrystalUse(true);

  // Open file and print headers
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println((String) "Amplitude Settings: " + A_right + ", " + A_left);
    dataFile.println((String) "Frequency Settings: " + f_right + "," + f_left);
    dataFile.println((String) "Bias Setting: " + startOffset);
    dataFile.println("================================================================");
    dataFile.println((String) "Time [s] " + "Pos [deg] " + "Actual Pos [deg] " + "Velocity [deg/s] " + "ax " + "ay " + "az " + "gx " + "gy " + "gz " + "amplitude_left " + "amplitude_right " + "frequency_left " + "frequency_right ");
    dataFile.close();
  } else {
    Serial.println((String) "error printing headers to " + fileName);
  }
}

void loop() {
  static bool setupComplete = false;
  static bool firstStep = true;  // Flag for the first step
  if (!setupComplete) {
    setupComplete = true;
    Serial.println("Setup complete, starting walking sequence.");
    delay(1000);  // Brief delay before starting the walking sequence
  }

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  printEvent(&orientationData);

  ax = accelerometerData.acceleration.x;
  ay = accelerometerData.acceleration.y;
  az = accelerometerData.acceleration.z;

  gx = orientationData.orientation.x;
  gy = orientationData.orientation.y;
  gz = orientationData.orientation.z;

  counter++;

  //we want to set up frame and collect data first, then start walking
  t_ime = millis();
  float t = t_ime / 1000;  // time in sec
  float pos = getJointAngle(t);

  dxl.setGoalPosition(legID, pos);

  prev_time = current_time;
  current_time = t;
  current_pos = pos;
  current_actual_pos = dxl.getPresentPosition(legID, UNIT_DEGREE);
  current_velocity = dxl.getPresentVelocity(legID, UNIT_RPM);
  current_current = dxl.getPresentCurrent(legID, UNIT_MILLI_AMPERE);
  current_PWM = dxl.getPresentPWM(legID, UNIT_PERCENT);

  WriteData datum;
  datum.current_time = current_time;
  datum.current_pos = current_pos;
  datum.current_actual_pos = current_actual_pos;
  datum.current_velocity = current_velocity;
  datum.ax = ax;
  datum.ay = ay;
  datum.az = az;
  datum.gx = gx;
  datum.gy = gy;
  datum.gz = gz;
  datum.A_left = A_left;
  datum.A_right = A_right;
  datum.f_left = f_left;
  datum.f_right = f_right;
  storeData(datum);

  flat_controller();
  

  if (firstStep) {
    A_left = 700;  // Set the left amplitude for the first step
    A_right = 700; // Set the right amplitude for the first step
    firstStep = false;
  }
}

void storeData(WriteData data) {
  dataBuffer[bufferIndex++] = data;

  // If buffer is full, write to SD card
  if (bufferIndex >= bufferSize) {
    writeBufferToSD();
    bufferIndex = 0;  // Reset buffer index
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

  tau = fmod(t, period_total);

  if (tau < p_right / 2) {
    pos = A_right * sin(w_right * tau + phi) + startOffset;
  } else if (p_right / 2 <= tau && tau <= p_right / 2 + p_left / 2) {
    pos = A_left * sin(w_left * (tau - p_right / 2 + p_left / 2) + phi) + startOffset;
  }
  prevTau = tau;
  return pos;
}

void flat_controller() {
  if (gx > 180 && gx < 360 - reasonable_alpha_z_range) {
    A_left = bigAmp + 15;
    A_right = smallAmp - 15;
  } else if (gx > reasonable_alpha_z_range && gx < 180) {
    A_left = smallAmp;
    A_right = bigAmp;
  } else {
    A_left = regularAmp;
    A_right = regularAmp;
  }
}

void writeBufferToSD() {
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < bufferIndex; i++) {
      dataFile.print(dataBuffer[i].current_time);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].current_pos);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].current_actual_pos);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].current_velocity);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].ax);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].ay);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].az);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].gx);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].gy);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].gz);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].A_left);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].A_right);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].f_left);
      dataFile.print(",");
      dataFile.println(dataBuffer[i].f_right);
    }
    dataFile.flush();  // Ensure data is written to the card
    dataFile.close();  // Close the file
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Error opening file for writing.");
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}