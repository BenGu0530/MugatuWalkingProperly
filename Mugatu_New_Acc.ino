#include <DynamixelShield.h>
#include <Wire.h>  // This library allows you to communicate with I2C devices.
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "I2Cdev.h"

float elapsedTime, currentTime, previousTime;

int counter = 0;
float pi = 3.14159;
const uint8_t legID = 1;  //Default ID is 1
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

//This namespace is required to use Control table item names
// using namespace ControlTableItem;

int startOffset = 800;  //Original 820
int GOAL_SPEED = 32;
float t_ime;

//Frequency for each leg
float f_left = 1.40;
float f_right = 1.40;

//Amplitude for sinusoid
float A_left = 450;  //Starting point at 440 (4096 clicks per revolution)
float A_right = 450;

const int chipSelect = SDCARD_SS_PIN;

String expDate = "M22";
String fileName;
String expNum = "a";

//Vectors for experimental data
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

// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Open serial communications and wait for port to open:
  // Serial.begin(57600);
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println("card initialized.");

  int trialNum = 0;
  do {
    trialNum++;
    fileName = expDate + expNum + (String)trialNum + ".txt";  //xFile name
  } while (SD.exists(fileName));

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  // dxl.begin(1000000);
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(legID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(legID);
  dxl.setOperatingMode(legID, OP_POSITION);
  dxl.torqueOn(legID);

  startOffset = dxl.getPresentPosition(legID);
  dxl.setGoalPosition(legID, startOffset);  //setting initial foot position
  delay(20);

  current_time = millis() / 1000;

  // dxl.setGoalPosition(legID, 300);

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  get_back_from_left = false;
  get_back_from_right = false;
  on_track = false;
}

void loop() {
  // Serial.println(dxl.getPresentPosition(legID));
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  printEvent(&gravityData);

  ax = accelerometerData.acceleration.x;
  ay = accelerometerData.acceleration.y;
  az = accelerometerData.acceleration.z;

  gx = orientationData.orientation.x;
  gy = orientationData.orientation.y;
  gz = orientationData.orientation.x;

  counter++;

  t_ime = millis();
  float t = t_ime / 1000;  //time in sec
  float pos = getJointAngle(t);

  //Updating position of dynamixel
  dxl.setGoalPosition(legID, pos);

  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

  //Update data vectors
  prev_time = current_time; // new
  current_time = t;
  current_pos = pos;
  current_actual_pos = dxl.getPresentPosition(legID, UNIT_DEGREE);
  current_velocity = dxl.getPresentVelocity(legID, UNIT_RPM);
  current_current = dxl.getPresentCurrent(legID, UNIT_MILLI_AMPERE);
  current_PWM = dxl.getPresentPWM(legID, UNIT_PERCENT);

  dataFile = SD.open(fileName, FILE_WRITE);
  //Print headers
  if (printHeaders) {
    printHeaders = false;

    if (dataFile) {
      dataFile.println((String) "Amplitude Settings: " + A_right + ", " + A_left);
      dataFile.println((String) "Frequency Settings: " + f_right + "," + f_left);
      dataFile.println((String) "Bias Setting: " + startOffset);
      dataFile.println("================================================================");
      dataFile.println((String) "Time [s] " + "Pos [deg] " + "Acutal Pos [deg] " + "Velocity [deg/s] " + "ax " + "ay " + "az " + "gx " + "gy " + "gz " + "amplitude_left " + "amplitude_right " + "frequency_left " + "frequency_right ");
      dataFile.close();
    } else {
      Serial.println((String) "error printing headers to " + fileName);
    }

  } else {
    //Write new data to SD Card
    if (dataFile) {
      dataString = (String)current_time + ", " + (String)current_pos + ", " + (String)current_actual_pos + ", " + (String)current_velocity + ", " + (String)ax + ", " + (String)ay + ", " + (String)az + ", " + (String)gx + ", " + (String)gy + ", " + (String)gz + ", " + (String)A_left + ", " + (String)A_right + ", " + (String)f_left + ", " + (String)f_right;
      dataFile.println(dataString);
      dataFile.close();
    } else {
      Serial.println((String) "error writing data to " + fileName);
    }
  }

  if (gx > 180 && gx < 360 - reasonable_alpha_z_range) {
    A_left  = bigAmp + 15;
    A_right = smallAmp - 15;
    get_back_from_left = true;
    on_track = false;
    // Serial.println(" gx " + (String)gx + " going where " + " going right " + (String)A_left + " , " + (String)A_right);
  } else if (gx > reasonable_alpha_z_range && gx < 180) {
    A_left = smallAmp;
    A_right = bigAmp;
    get_back_from_right = true;
    on_track = false;
    // Serial.println(" gx " + (String)gx + " going where " + " going left " + (String)A_left + " , " + (String)A_right);
  } else {
    A_left = regularAmp;
    A_right = regularAmp;
    on_track = true;
    // Serial.println(" gx " + (String)gx + " going where " + " going straight " + (String)A_left + " , " + (String)A_right);
  }

  if (on_track && gz > -2.0 && gz < 8.5){
    if (get_back_from_left){
      A_left = smallAmp;
      A_right = bigAmp + 30;
      get_back_from_left = false;
      get_back_from_right = false;
      Serial.println("compensation scenario 1 " + (String)A_left + " , " + (String)A_right);

    } else if (get_back_from_right) {
      A_left = bigAmp + 30;
      A_right = smallAmp;
      get_back_from_left = false;
      get_back_from_right = false;
      Serial.println("compensation scenario 2 " + (String)A_left + " , " + (String)A_right);
    }
  }
  
  // Serial.println((String)A_left + " , " + (String)A_right + " , " + (String)gx + " , " + (String)gy + " , " + (String)gz);
}

float getJointAngle(float t) {
  //Defining constants for sinusoid
  float phi = 0;
  float w_left = 2 * pi * f_left;
  float w_right = 2 * pi * f_right;
  float p_right = 1 / f_right;
  float p_left = 1 / f_left;
  float period_total = 1 / (2 * f_left) + 1 / (2 * f_right);
  float pos = 0;

  static float tau = fmod(t, period_total);  //time within one period of sinusoid;
  static float prevTau;

  /*Calculating sinusoidal input*/
  tau = fmod(t, period_total);  //time within one period of sinusoid

  //First half
  if (tau < p_right / 2) {
    pos = A_right * sin(w_right * tau + phi) + startOffset;
  }
  //Second half
  else if (p_right / 2 <= tau && tau <= p_right / 2 + p_left / 2) {
    pos = A_left * sin(w_left * (tau - p_right / 2 + p_left / 2) + phi) + startOffset;
  }
  prevTau = tau;
  return pos;
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
