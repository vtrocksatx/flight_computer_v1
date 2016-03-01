/**
 * RockSat-X 2016 Flight Computer code.
 *
 * Version: 03/01/2016
 * Author: RockSat-X at Virginia Tech
 */

// Library inclusions
#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include "SFE_BMP180.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Pin definitions
#define SS 53 // Serial1 Select -> CS on LIS331
#define MOSI 51 // MasterOutSlaveIn -> SDI
#define MISO 50 // MasterInSlaveOut -> SDO
#define SCK 52 // Serial1 Clock -> SPC on LIS331
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define NASA_TIMER_EVENT_INPUT 31

// Constant definitions
#define SCALE 0.0007324; // approximate scale factor for full range (+/-24g)
#define OUTPUT_READABLE_YAWPITCHROLL
#define TRANSMIT_BEGIN_TIME 100
#define MAX_PACKET_SIZE 255

// Object declarations
SFE_BMP180 pressure;
MPU6050 mpu(0x68); // <-- use for AD0 high
Quaternion q; // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector

// Global variables
double xAcc, yAcc, zAcc;
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int globalDelay = 100;
long initTime;
long currentTime = 0;
bool launched = false;
bool valRead = false;
bool getTime = true;
int transmittedPacketCount = 0;
String dataPacketToTransmit;

/**
 * This fuction is run once at the beginning of the microcontroller's initialization sequence.
 * Its purpose is to peform all necessary setup operations.
 */
void setup() {
  Serial1.begin(19200); // OpenLog and RS-232 Serial output
  Serial.begin(9600); // E310 Serial output
  pinMode(NASA_TIMER_EVENT_INPUT, INPUT);
  gyroSetup();
  openLoggerSetup();
  accelerometerSetup();
  barometricSetup();
  Serial1.print("Time(sec)\tTemp(deg C)\tAbs Pres.(mb)\tYaw\tPitch\tRoll\tAccelX\tAccelY\tAccelZ");
  Serial1.println();
}

/**
 * This fuction is run repeatedly after the microcontroller's initialization sequence is complete.
 * Its purpose is to peform all data collection, processing, and storing operations as well as
 * handle all payload command and control.
 */
void loop() {
  if (!launched && digitalRead(NASA_TIMER_EVENT_INPUT) == HIGH) {
    launched = true;
    initTime = millis(); // Run once to get the elapsed time between boot up and launch
    Serial1.println("Launch Timer Event State Change: HIGH");
  }
  
  currentTime = millis() - initTime;
  Serial1.print(currentTime / 1000);
  Serial1.print(".");
  Serial1.print(currentTime % 1000);
  Serial1.print("\t\t");
  initTransmitPacket();
  barometricLoop();
  gyroLoop();
  accelerometerLoop();
  Serial1.println();
  
  // Begin passing data to E310 once antenna is deployed and E310 is booted
  if (launched && currentTime >= TRANSMIT_BEGIN_TIME) {
    
    if (dataPacketToTransmit.length() > MAX_PACKET_SIZE) {
      dataPacketToTransmit = dataPacketToTransmit.substring(0, MAX_PACKET_SIZE);
    }
    
    Serial.print(dataPacketToTransmit);
    Serial.println();
    transmittedPacketCount++;
  }
}

/********************************************************
 * Command and control specific functions               *
 ********************************************************/
 
 

/********************************************************
 * USRP E310 communication specific functions           *
 ********************************************************/

void initTransmitPacket() {
  dataPacketToTransmit = String("$,callsign,");
  dataPacketToTransmit += transmittedPacketCount;
  dataPacketToTransmit +=",";
  dataPacketToTransmit += currentTime / 1000.0;
  dataPacketToTransmit +=",";
}

/********************************************************
 * Datalogger specific functions                        *
 ********************************************************/

void openLoggerSetup() {
  pinMode(LED_PIN, OUTPUT);
  delay(1000); //Wait a second for OpenLog to init
  Serial1.println();
}

/********************************************************
 * Accelerometer specific functions                     *
 ********************************************************/

void accelerometerSetup() {
  // SPI setup
  pinMode(SS, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // SPI clock 1000Hz
  
  // Accelerometer setup
  byte addressByte = 0x20;
  byte ctrlRegByte = 0x37; // 00111111 : normal mode, 1000Hz, xyz enabled
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);
  delay(100);
  addressByte = 0x21;
  ctrlRegByte = 0x00; // High pass filter off
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);
  delay(100);
  addressByte = 0x23;
  ctrlRegByte = 0x30; // 00110000 : 24G (full scale)
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(addressByte);
  SPI.transfer(ctrlRegByte);
  delay(1);
  digitalWrite(SS, HIGH);
}

void readVal() {
  byte xAddressByteL = 0x28; // Low Byte of X value (the first data register)
  byte readBit = B10000000; // bit 0 (MSB) HIGH means read register
  byte incrementBit = B01000000; // bit 1 HIGH means keep incrementing registers
  byte dataByte = xAddressByteL | readBit | incrementBit;
  byte b0 = 0x0; // an empty byte, to increment to subsequent registers
  digitalWrite(SS, LOW); // SS must be LOW to communicate
  delay(1);
  SPI.transfer(dataByte); // request a read, starting at X low byte
  byte xL = SPI.transfer(b0); // get the low byte of X data
  byte xH = SPI.transfer(b0); // get the high byte of X data
  byte yL = SPI.transfer(b0); // get the low byte of Y data
  byte yH = SPI.transfer(b0); // get the high byte of Y data
  byte zL = SPI.transfer(b0); // get the low byte of Z data
  byte zH = SPI.transfer(b0); // get the high byte of Z data
  delay(1);
  digitalWrite(SS, HIGH);
  int xVal = (xL | (xH << 8));
  int yVal = (yL | (yH << 8));
  int zVal = (zL | (zH << 8));
  xAcc = xVal * SCALE;
  yAcc = yVal * SCALE;
  zAcc = zVal * SCALE;
}

void accelerometerLoop() {
  readVal(); // get acc values and put into global variables

  Serial1.print(xAcc, 2);
  Serial1.print("\t");
  Serial1.print(yAcc, 2);
  Serial1.print("\t");
  Serial1.print(zAcc, 2);
  Serial1.print("\t");
  
  dataPacketToTransmit += xAcc;
  dataPacketToTransmit += ",";
  dataPacketToTransmit += yAcc;
  dataPacketToTransmit += ",";
  dataPacketToTransmit += zAcc;

  delay(1);
}

/********************************************************
 * Barometer specific functions                         *
 ********************************************************/

void barometricSetup() {
  Serial1.println("BMP180 REBOOT");
  if (pressure.begin()) {
    Serial1.println("BMP180 init success");
  }
  else {
    Serial1.println("BMP180 init fail\n\n");
  }
}

void barometricLoop() {
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      Serial1.print(T, 2);
      Serial1.print("\t\t");
      dataPacketToTransmit += T;
      dataPacketToTransmit += ",";
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0) {
          Serial1.print(P, 2);
          Serial1.print("\t\t");
          dataPacketToTransmit += P;
          dataPacketToTransmit += ",";
        }
        else {
          Serial1.println("error retrieving pressure measurement\n");
        }
      }
      else {
        Serial1.println("error starting pressure measurement\n");
      }
    }
    else {
      Serial1.println("error retrieving temperature measurement\n");
    }
  }
  else {
    Serial1.println("error starting temperature measurement\n");
  }

  delay(globalDelay);
}

/********************************************************
 * Gyroscope specific functions                         *
 ********************************************************/

void dmpDataReady() {
  mpuInterrupt = true;
}

void gyroSetup() {
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
  
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  
  Fastwire::setup(400, true);
  
  #endif
  
  while (!Serial1); // wait for Leonardo enumeration, others continue immediately
  Serial1.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial1.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(100);
  mpu.setZGyroOffset(-60);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
  if (devStatus == 0) {
    Serial1.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial1.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial1.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    Serial1.print(F("DMP Initialization failed (code "));
    Serial1.print(devStatus);
    Serial1.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
}

void gyroLoop() {
  if (!dmpReady) {
    return;
  }
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial1.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial1.print(ypr[0] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[1] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[2] * 180 / M_PI);
    Serial1.print("\t");

    dataPacketToTransmit += (double)(ypr[0] * 180 / M_PI);
    dataPacketToTransmit += ",";
    dataPacketToTransmit += (double)(ypr[1] * 180 / M_PI);
    dataPacketToTransmit += ",";
    dataPacketToTransmit += (double)(ypr[2] * 180 / M_PI);
    dataPacketToTransmit += ",";

    #endif

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
