#include <DynamixelShield.h>
#include <Wire.h>  // This library allows you to communicate with I2C devices.
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

float elapsedTime, currentTime, previousTime;

int counter = 0;

float pi = 3.14159;
const uint8_t legID = 1;  //Default ID is 1
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;
MPU6050 mpu;

//This namespace is required to use Control table item names
// using namespace ControlTableItem;

int startOffset = 800;  //Original 820
int GOAL_SPEED = 32;
float t_ime;

//Frequency for each leg
float f_left = 1.4;
float f_right = 1.4;

//Amplitude for sinusoid
float A_left = 450;  //Starting point at 440 (4096 clicks per revolution)
float A_right = 450;

const int chipSelect = SDCARD_SS_PIN;

String expDate = "A4";
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

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// void dmpDataReady() {
//   mpuInterrupt = true;
// }

float ax, ay, az;
float gx, gy, gz;

bool prev_leg_left;    
bool curr_leg_left;    
bool switched_leg;
float alpha_z = 0.0; //cumulative Yaw angle
float sine_part = 1.0; // need to define this by the pos

float reasonable_alpha_z_range = 5.0;
float deltaAmp = 100;
float regularAmp = 450.0;
float smallAmp = regularAmp - deltaAmp;
float bigAmp = regularAmp + deltaAmp;




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

  delay(250);
  MPU_setup();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);

  prev_leg_left == true;    
  curr_leg_left == false;    
  alpha_z = -5.0;
}

void loop() {
  MPU_loop(0);
  // Serial.println(dxl.getPresentPosition(legID));

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
  current_time = t;
  current_pos = pos;
  current_actual_pos = dxl.getPresentPosition(legID, UNIT_DEGREE);
  current_velocity = dxl.getPresentVelocity(legID, UNIT_RPM);
  current_current = dxl.getPresentCurrent(legID, UNIT_MILLI_AMPERE);
  current_PWM = dxl.getPresentPWM(legID, UNIT_PERCENT);

  /// see gz and alpha_z
  Serial.println(" gz " + (String)gz + " alpha_z " + (String)alpha_z);

  dataFile = SD.open(fileName, FILE_WRITE);
  //Print headers
  if (printHeaders) {
    printHeaders = false;

    if (dataFile) {
      dataFile.println((String) "Amplitude Settings: " + A_right + ", " + A_left);
      dataFile.println((String) "Frequency Settings: " + f_right + "," + f_left);
      dataFile.println((String) "Bias Setting: " + startOffset);
      dataFile.println("================================================================");
      dataFile.println((String) "Time [s] " + "Pos [deg] " + "Acutal Pos [deg] " + "Velocity [deg/s] " + "ax " + "ay " + "az " + "gx " + "gy " + "gz " + "amplitude_left " + "amplitude_right " + "alpha_z " + "frequency_left " + "frequency_right " + "switched_leg ");
      dataFile.close();
    } else {
      Serial.println((String) "error printing headers to " + fileName);
    }

  } else {
    //Write new data to SD Card
    if (dataFile) {
      dataString = (String)current_time + ", " + (String)current_pos + ", " + (String)current_actual_pos + ", " + (String)current_velocity + ", " + (String)ax + ", " + (String)ay + ", " + (String)az + ", " + (String)gx + ", " + (String)gy + ", " + (String)gz + ", " + (String)A_left + ", " + (String)A_right + ", " + (String)alpha_z + ", " + (String)f_left + ", " + (String)f_right + ", " + (String)switched_leg;
      dataFile.println(dataString);
      dataFile.close();
    } else {
      Serial.println((String) "error writing data to " + fileName);
    }
  }

  Serial.println("sine part = " + (String)sine_part);

  prev_leg_left = curr_leg_left;
  if (sine_part > 0) {
    curr_leg_left = true;  // current status: left leg landing
    Serial.println("sine part > 0 >>>>>>>>>>>>>>" + (String)sine_part);
  } else {
    curr_leg_left = false;
    Serial.println("sine part < 0 <<<<<<<<<<<<<<" + (String)sine_part);

  }
  // if statement 2: check if there is a contact leg shift
  if (prev_leg_left  == curr_leg_left) {
      switched_leg = false;
  } else if (prev_leg_left  != curr_leg_left) {
      switched_leg = true;
  }

  // control part
  if (switched_leg){
    // if we switched leg
    Serial.println(" Swithced_leg in.......");

    if (curr_leg_left) { 
      //left leg has just landed and right leg is about to swing
        Serial.println(" Swithced_leg in and curr_leg_left in ........");

      if (alpha_z < -1 * reasonable_alpha_z_range) {
        //left leg landed and off the track too right
        //then we need a faster swing on left leg
        Serial.println(" Swithced_leg in and curr_leg_left in and we are too right.......");
        A_left = smallAmp;
        A_right = bigAmp;
      } else  if (alpha_z > reasonable_alpha_z_range) {
        //left leg landed and off the track too left
        //then we need a slower swing on left leg
        Serial.println(" Swithced_leg in and curr_leg_left in and we are too left.......");
        A_left  = bigAmp;
        A_right = smallAmp;
      } else if (alpha_z > -1 * reasonable_alpha_z_range && alpha_z < reasonable_alpha_z_range ) {
        A_right = regularAmp;
        A_left = regularAmp;
      }
    } else if (curr_leg_left == false) {
      // right leg has just landed and left leg is about to swing
      //didn't see any reason we need to determine this actually
      Serial.println(" Swithced_leg in and curr_leg_left not in.......");

      if (alpha_z < -1 * reasonable_alpha_z_range) {
        Serial.println(" Swithced_leg in and curr_leg_left not in and we are too right.......");
        //right leg landed and off the track too right
        //then we need a slower swing on right leg
        A_left = smallAmp;
        A_right = bigAmp;

      } else if (alpha_z > reasonable_alpha_z_range) {
        Serial.println(" Swithced_leg in and curr_leg_left not in and we are too left.......");
        //left leg landed and off the track too left
        //then we need a faster swing on right leg
        A_left  = bigAmp;
        A_right = smallAmp;
      } else if (alpha_z > -1 * reasonable_alpha_z_range && alpha_z < reasonable_alpha_z_range) {
        A_right = regularAmp;
        A_left = regularAmp;
      }
    }
    //always record alpha_z
    //might need an if statement here
    alpha_z += gz * (current_time - prev_time);
    switched_leg = false;
  } else if (switched_leg == false) {
    Serial.println(" Swithced_leg not in ........");
    //continue on the same leg
    //we recording alpha_z
    alpha_z += gz * (current_time - prev_time);
  }
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
    sine_part = 1;
  }
  //Second half
  else if (p_right / 2 <= tau && tau <= p_right / 2 + p_left / 2) {
    pos = A_left * sin(w_left * (tau - p_right / 2 + p_left / 2) + phi) + startOffset;
    sine_part = -1;
  }
  prevTau = tau;
  return pos;
}

void MPU_setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(100000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // while (!Serial)
  // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));

  mpu.initialize();

  // pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // // enable Arduino interrupt detection
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    // mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void MPU_loop(bool realAccel) {
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gx = ypr[0] * 180 / M_PI;
    gy = ypr[1] * 180 / M_PI;
    gz = ypr[2] * 180 / M_PI;

    if (realAccel) {
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      ax = aaReal.x;
      ay = aaReal.y;
      az = aaReal.z;
    } else {
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      ax = aaWorld.x;
      ay = aaWorld.y;
      az = aaWorld.z;
    }
  }
}