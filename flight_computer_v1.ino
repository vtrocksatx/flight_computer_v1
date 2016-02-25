#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define SS 53 // Serial1 Select -> CS on LIS331
#define MOSI 51 // MasterOutSlaveIn -> SDI
#define MISO 47 // MasterInSlaveOut -> SDO
#define SCK 49 // Serial1 Clock -> SPC on LIS331
#define SCALE 0.0007324; // approximate scale factor for full range (+/-24g)
#define ALTITUDE 634.0
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

double xAcc, yAcc, zAcc; // global acceleration values
SFE_BMP180 pressure;
int ledPin =  13; //Status LED connected to digital pin 13 - for openLogger
int nasaTimerPin = 31;
MPU6050 mpu(0x68); // <-- use for AD0 high
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int globalDelay = 100;
long time0;
long newTime = 0;
bool valRead = false;
bool getTime = true;
char ettusArray[256];
int ettusTransCount = 1;
char buffer[256];

void dmpDataReady() {
  mpuInterrupt = true;
}

void clearArray() {
  memset(ettusArray, '\0', sizeof ettusArray);
}

void convertToArray(double val) {
  if (val < 0) {
    strcat(ettusArray, "-");
  }
  val = abs(val);

  itoa(val, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  strncat(ettusArray, ".", countCharsLeft());
  itoa(((int)(val * 10)) % 10, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  itoa(((int)(val * 100)) % 10, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  strncat(ettusArray, ",", countCharsLeft());
}

int countCharsLeft() {
  int i = 0;
  while (ettusArray[i] != '\0') {
    i++;
  }
  return (255 - i);
}

void setup() {
  Serial1.begin(19200);
  Serial.begin(9600); // Ettus
  pinMode(nasaTimerPin, INPUT);
  gyroSetup();
  openLoggerSetup();
  accelerometerSetup();
  barometricSetup();
  Serial1.print("Time(sec)\tTemp(deg C)\tAbs Pres.(mb)\tYaw\tPitch\tRoll\tAccelX\tAccelY\tAccelZ");
  Serial1.println();
}

void loop() {
  while (valRead != true) {
    valRead = digitalRead(nasaTimerPin);
  }
  if (getTime == true) {
    time0 = millis();
    getTime = false;
  }
  newTime = millis() - time0;
  Serial1.print(newTime / 1000);
  Serial1.print(".");
  Serial1.print(newTime % 1000);
  Serial1.print("\t\t");
  ettusLoopHelper();
  barometricLoop();
  gyroLoop();
  accelerometerLoop();
  Serial.print(ettusArray);
  Serial.println();
  Serial1.println();
  ettusTransCount++;
}

void ettusLoopHelper() {
  clearArray();
  strcat(ettusArray, "$,callsign,");
  itoa(ettusTransCount, buffer, 10);
  strcat(ettusArray, buffer);
  memset(buffer, '\0', sizeof buffer);
  strcat(ettusArray, ",");
  itoa(newTime / 1000, buffer, 10);
  strcat(ettusArray, buffer);
  memset(buffer, '\0', sizeof buffer);
  strcat(ettusArray, ".");
  itoa(newTime % 1000, buffer, 10);
  strcat(ettusArray, buffer);
  memset(buffer, '\0', sizeof buffer);
  strcat(ettusArray, ",");
}

void openLoggerSetup() {
  pinMode(ledPin, OUTPUT);
  delay(1000); //Wait a second for OpenLog to init
  Serial1.println();
}

void accelerometerSetup() {
  SPI_SETUP();
  Accelerometer_Setup();
}

void accelerometerLoop() {
  readVal(); // get acc values and put into global variables

  Serial1.print(xAcc, 2);
  Serial1.print("\t");
  Serial1.print(yAcc, 2);
  Serial1.print("\t");
  Serial1.print(zAcc, 2);
  Serial1.print("\t");
  convertToArray(xAcc);
  convertToArray(yAcc);
  if (zAcc < 0) {
    strcat(ettusArray, "-");
  }
  zAcc = abs(zAcc);

  itoa(zAcc, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  strncat(ettusArray, ".", countCharsLeft());
  itoa(((int)(zAcc * 10)) % 10, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  itoa(((int)(zAcc * 100)) % 10, buffer, 10);
  strncat(ettusArray, buffer, countCharsLeft());
  memset(buffer, '\0', sizeof buffer);
  delay(1);
}

void barometricSetup() {
  Serial1.println("BMP180 REBOOT");
  if (pressure.begin())
    Serial1.println("BMP180 init success");
  else
  {
    Serial1.println("BMP180 init fail\n\n");
  }
}

void barometricLoop() {
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      Serial1.print(T, 2);
      Serial1.print("\t\t");
      convertToArray(T);
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          Serial1.print(P, 2);
          Serial1.print("\t\t");
          convertToArray(P);
        }
        else Serial1.println("error retrieving pressure measurement\n");
      }
      else Serial1.println("error starting pressure measurement\n");
    }
    else Serial1.println("error retrieving temperature measurement\n");
  }
  else Serial1.println("error starting temperature measurement\n");

  delay(globalDelay);  // Pause for 5 seconds.
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
  } else {
    Serial1.print(F("DMP Initialization failed (code "));
    Serial1.print(devStatus);
    Serial1.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
}

void gyroLoop() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial1.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial1.print("ypr\t");
    Serial1.print(ypr[0] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[1] * 180 / M_PI);
    Serial1.print("\t");
    Serial1.print(ypr[2] * 180 / M_PI);
    Serial1.print("\t");
    convertToArray((double)(ypr[0] * 180 / M_PI));
    convertToArray((double)(ypr[1] * 180 / M_PI));
    convertToArray((double)(ypr[2] * 180 / M_PI));

#endif

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void readVal()
{
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

void SPI_SETUP()
{
  pinMode(SS, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // SPI clock 1000Hz
}

void Accelerometer_Setup()
{
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
