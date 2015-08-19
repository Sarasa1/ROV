#include <Servo.h>
#include <String.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

/********************************GYRO AND ACCELEROMETER WIRING******************************
 * WIRING OF MPU-6050
 * 
 * VCC: +3.3V From Arduino (attach 0.1 microFarad capacitor to ground)
 * GND: GND
 * SCL: SCL
 * SDA: SDA
 * XDA: Not connected
 * XCL: Not connected
 * ADO: GND
 * INT: Digital Pin 2 on Arduino
 ********************************************************************************************/

/**********************************ARDUINO GLOBAL VARIABLES***********************************/
//Arduino drive variables:
Servo m0,m1,m2,m3,m4,m5;
const long maxMotorValue = 400;
const long avgMotorValue = 0;

//Raw values passed from the C++ program. Kept as seperate variables mainly for intuitiveness
int RT, LT, R3, L3, RB, LB, UpD, DownD, LeftD, RightD, A, B, X, Y, Start, Back, Verification;
int RBPrevState = 0;
long RX, RY, LX, LY;
int motorRange = 200;

//Leap motion controller values passed from the C++ program
int palmVX = 0, palmVY = 0, palmVZ = 0, palmPitch = 0, palmYaw = 0, palmRoll = 0, clawDistance = 0;
boolean usingLeap = false;

//Sonar Sensor Variables
const int sonarPin = 0;
const float sonarConstant = .47921;
float sonarDistance = 0;

//Pressure Sensor Variables
const int pressureSensorPin = 1;
const float pressureConstant = 1;
float pressure;

//Gyro&accelerometer variables
MPU6050 mpu;
int PitchCenter = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Variable to intialize orientation of axes
const int pitchOrient = 25;
const int rollOrient = -20;
const boolean pitch = true;
const boolean roll = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
/*********************************************************************************************/


/*****************************SETUP AND INTIALIZATION ****************************************/
void setup()
{
  InitializeESCs();
  InitializeGyroAndAcceleromter();
  Serial.begin(115200, SERIAL_8N1);  //Baud here must match baud on c++ side
  Serial.setTimeout(100);
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
}

void InitializeESCs()
{
  m0.attach(3); 
  m1.attach(4);
  m2.attach(5);
  m3.attach(6);
  m4.attach(7);
  m5.attach(8);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  m0.writeMicroseconds(1500);
  m1.writeMicroseconds(1500);
  m2.writeMicroseconds(1500);
  m3.writeMicroseconds(1500);
  m4.writeMicroseconds(1500);
  m5.writeMicroseconds(1500);

  delay(1000); 
}

void InitializeGyroAndAcceleromter()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  //supply gyro offsets here
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(-48);
  mpu.setZGyroOffset(-28);
  mpu.setZAccelOffset(5420);

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}
/*************************END SETUP AND INITIALIZATION*****************************/


/***********************************MAIN PROGRAM LOOP*****************************************/
void loop()
{
  if(Serial)
  {
    if(ReadInValues()== 0)
    {
      ReadGyroAndAccelerometer();
      WriteOutValues();

      if (DriveControl() == 1)
      {
        Move();
      }

      //      else if (DriveControl() == -1)
      //      {
      //        LeapMove();
      //      }
      //        sonarDistance = ReadAnalogSensor(sonarPin, sonarConstant);
      //        pressure = ReadAnalogSensor(pressureSensorPin, pressureConstant);

      if (Start == 1)
      {
        PitchCenter = zeroGyroAndAccelerometer();
      }

      if (LB == 1)
      {
        PitchControl((ypr[1]* 180/M_PI), PitchCenter, 5);
      }

    }

    else
    {
      StopROV();
    }    
  }

  else
  {
    StopROV();
  }   
}

/*********************************END MAIN PROGRAM LOOP*******************************/


/***********************************SERIAL COMMUNICATION*****************************************/
int ReadInValues()
{
  Serial.print(F("r"));
  RX = Serial.parseInt();
  RY = Serial.parseInt();
  LX = Serial.parseInt();
  LY = Serial.parseInt();
  RT = Serial.parseInt();
  LT = Serial.parseInt();
  R3 = Serial.parseInt();
  L3 = Serial.parseInt();
  RB = Serial.parseInt();
  LB = Serial.parseInt();
  UpD = Serial.parseInt();
  DownD = Serial.parseInt();
  LeftD = Serial.parseInt();
  RightD = Serial.parseInt();
  A = Serial.parseInt();
  B = Serial.parseInt();
  Y = Serial.parseInt();
  X = Serial.parseInt();
  Start = Serial.parseInt();
  Back = Serial.parseInt();
  Verification = Serial.parseInt();
  palmVX = Serial.parseInt();
  palmVY = Serial.parseInt();
  palmVZ = Serial.parseInt();
  palmPitch = Serial.parseInt();
  palmYaw = Serial.parseInt();
  palmRoll = Serial.parseInt();
  clawDistance = Serial.parseInt();

  if (Verification != 9)
  {
    return -1;
  }

  else
  {
    Serial.print(F("q"));
  }

  motorRange = 200;
  RX = map(RX, -32767, 32767, -(motorRange), motorRange);
  RY = map(RY, -32767, 32767, -(motorRange), motorRange);
  LX = map(LX, -32767, 32767, -(motorRange), motorRange);
  LY = map(LY, -32767, 32767, -(motorRange), motorRange);
  LT = map(LT, 0, 255, 0, motorRange);
  RT = map(RT, 0, 255, 0, -(motorRange));

  return 0;
}
void WriteOutValues()
{
  int i = 0;
  int sent = 0;
  String buffer[8];
//  for (i = 0; i<8; i++)
//  {
//    sent = random(1, 360);
//    buffer[i] = editForSend(sent, 3);
//  }
  
  for (i = 0; i<8; i++)
  {
    sent = 21250;
    Serial.print(editForSend(sent, 3));
  }
  
}

//value is the number to be sent, number of bytes is the number of bytes it should be sent in
String editForSend(int value, int numberOfBytes)
{
  String buffer = String(value);
  
  while (buffer.length() < numberOfBytes)
  {
    buffer.concat('x');
  }
  
  if (buffer.length() > numberOfBytes)
  {
    buffer.remove(numberOfBytes);
  }
  
  return buffer;
}


/***********************************END SERIAL COMMUNICATION*****************************************/

/***********************************DRIVE CODE*****************************************/
void writeMotor(Servo motor, int value)
{

  motor.writeMicroseconds((1500 + value));
}

int DriveControl()
{
  if (usingLeap == true && RBPrevState == 1 && RB == 0)
  {
    usingLeap = false;
  }

  else if (usingLeap == false && RBPrevState == 1 && RB == 0)
  {
    usingLeap = true;
  }

  RBPrevState = RB;

  if (usingLeap == false)
    return 1;

  else if (usingLeap == true)
    return -1;
}

void Move()  //digital write pushes water out, high sucks water in
{
  //Control for motors 0 and 1, thrusters on back of ROV
  if (LY != 0 && RX == 0)
  {
    writeMotor(m0, LY);
    writeMotor(m1, LY);
  }

  else if (LY == 0 && RX != 0)
  {
    writeMotor(m0, RX);
    writeMotor(m1, -(RX));
  }

  else if (LY != 0 && RX != 0)
  {
    writeMotor(m0, DirectionCorrection(LY, RX, '+'));
    writeMotor(m1, DirectionCorrection(LY, RX, '-'));
  } 
  // ROV isn't moving back thrusters
  else 
  {
    writeMotor(m0, 0);
    writeMotor(m1, 0);
  }

  //motor 2 and 3 control, RT is decend, LT is ascend
  if (LB == 0)
  {
    if (RY == 0 && (RT != 0  || LT != 0))
    {
      if (RT != 0 && LT == 0)
      {
        writeMotor(m2, RT);
        writeMotor(m3, RT);
      }

      else if (RT == 0 && LT != 0)
      {
        writeMotor(m2, LT);
        writeMotor(m3, LT);
      }

      else
      {
        writeMotor(m2, 0);
        writeMotor(m3, 0);
      }
    }

    else if (RY != 0 && RT == 0 && LT == 0)
    {
      writeMotor(m2, -(RY));
      writeMotor(m3, RY);
    }

    else if (RY != 0 && (RT != 0  || LT != 0))
    {
      if (RT != 0 && LT == 0)
      {
        writeMotor(m2, DirectionCorrection(RT, RY, '+'));
        writeMotor(m3, DirectionCorrection(RT, RY, '-'));
      }

      else if (RT == 0 && LT != 0)
      {
        writeMotor(m2,  DirectionCorrection(LT, RY, '-'));
        writeMotor(m3,  DirectionCorrection(LT, RY, '+'));
      }

      else
      {
        writeMotor(m2, 0);
        writeMotor(m3, 0);
      }
    }

    else
    {
      writeMotor(m2, 0);
      writeMotor(m3, 0);
    }
  }
  //motor 4 and 5 control
  writeMotor(m4, -(LX));
  writeMotor(m5, LX);

}

int DirectionCorrection(long primaryValue, long secondaryValue, char side)
{
  int correctedValue = 0;

  if (side == '+')
  {
    correctedValue = primaryValue/2 + primaryValue*secondaryValue/800;
  }

  else if (side == '-')
  {
    correctedValue = primaryValue/2 - primaryValue*secondaryValue/800;
  }

  return correctedValue;
}

void PitchControl(int currentPitch, int nominalPitch, int tollerance)
{
  int thrust;

  if (currentPitch > nominalPitch)
    thrust = map(currentPitch, nominalPitch, nominalPitch + 45, 75, 150);

  else if (currentPitch < nominalPitch)
    thrust = map(currentPitch, nominalPitch, nominalPitch - 45, 75, 150);


  if ((currentPitch <= (nominalPitch + tollerance)) && (currentPitch >= (nominalPitch - tollerance)))
  {
    writeMotor(m2, 0);
    writeMotor(m3, 0);
    digitalWrite(53, LOW);
    digitalWrite(52, HIGH);
    return;
  }

  else if (currentPitch > nominalPitch)
  {
    writeMotor(m2, -(thrust));
    writeMotor(m3, thrust);
  }

  else if (currentPitch < nominalPitch)
  {
    writeMotor(m2, thrust);
    writeMotor(m3, -(thrust));
  }

  digitalWrite(52, LOW);
  digitalWrite(53, HIGH);
  return;
}

void StopROV()
{
  m0.writeMicroseconds(1500);
  m1.writeMicroseconds(1500);
  m2.writeMicroseconds(1500);
  m3.writeMicroseconds(1500);
  m4.writeMicroseconds(1500);
  m5.writeMicroseconds(1500);
}
/**********************************************END DRIVE CODE*************************************************/

/****************************************SENSOR I/O********************************************/
void ReadGyroAndAccelerometer()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) 
  {
    if( mpuInterrupt ) 
    {
      break;
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
  }

  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
#endif
  }
}

void dmpDataReady() {
  mpuInterrupt = true;
}

int ReadAnalogSensor(int sensorPin, float calibrationConstant)
{
  float incomingValue = 0;
  incomingValue = analogRead(sensorPin);

  return (incomingValue*calibrationConstant);
}

long zeroGyroAndAccelerometer()
{
  int sample = 10;
  long sum = 0, average = 0;
  int i = 0;
  for (i=0; i < sample; i++)
  {
    ReadGyroAndAccelerometer();
    sum += (ypr[1]* 180/M_PI);
  }

  average = sum/sample;
  return average;
}
/********************************************END SENSOR I/O************************************/


