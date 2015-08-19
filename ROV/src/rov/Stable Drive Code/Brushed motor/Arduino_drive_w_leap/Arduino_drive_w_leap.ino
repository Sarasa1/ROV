#include<Wire.h>
#include<Math.h>
#include <Servo.h>

// raw values passed from the C++ program. Kept as seperate variables mainly for intuitiveness
int RT, LT, R3, L3, RB, LB, UpD, DownD, LeftD, RightD, A, B, X, Y, Start, Back, Verification;
int palmVX = 0, palmVY = 0, palmVZ = 0, palmPitch = 0, palmYaw = 0, palmRoll, clawDistance = 0;
const int maxMotorValue =  190;
boolean usingLeap = false;
int RBPrevState = 0;
int i;
int PWMPinLocation[] = {
  3,5,6,9,10,11};
int height = 0;
long RX, RY, LX, LY;
/*
Wiring Diagram for MPU6050 IMU -> Arduino:
 VCC -> 3.3V
 GND -> GND
 SDA -> SDA
 SCL -> SCL
 AD0 -> resitor -> GND
 */

//Gyro and accelerometer variables
const int MPU=0x68;  // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY, GyZ;
double Zpos = 0.0,pos = 0.0;

typedef struct{
  int PWMPin;
  int directionPin;
  int Power;
  int Direction;
} 
motorStruct;

motorStruct motor[6];

void setup()
{
  //Gyro/Accelerometer setup
  //  Wire.begin();
  //  Wire.beginTransmission(MPU);
  //  Wire.write(0x6B);  // PWR_MGMT_1 register
  //  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  //  Wire.endTransmission(true);

  Serial.begin(14400, SERIAL_8N1);  //Baud here must match baud on c++ side
  Serial.setTimeout(100);

  for (i = 0; i<= 6; i++)
  {
    pinMode(PWMPinLocation[i], OUTPUT);
    motor[i].PWMPin = PWMPinLocation[i];  //PWM pin for motor 0 will be 3, corresponnds with the PWM PinLocation array
    pinMode(30+2*i, OUTPUT);
    motor[i].directionPin = (2*i+30);  //Digital pin for motor 0 will be 32 and go up by one
  }
}

void loop()
{
  int i = 0;
  //ReadGyro();
  if(Serial.available() > 0)
  {
    if(ReadInValues()!= -1)
    {
      if (DriveControl() == 1)
      {
        Move();
      }

      else if (DriveControl() == -1)
      {
        LeapMove();
      }
    }

    else
    {
      for(i = 0; i < 6; i++)
      {
        analogWrite(motor[i].PWMPin, 0);
      }
    }
  }
  else
  {
    for(i = 0; i < 6; i++)
    {
      analogWrite(motor[i].PWMPin, 0);
    }
  }
}

int ReadInValues()
{
  Serial.print("r ");
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
    Serial.print("q ");
  }

  RX = map(RX, -32767, 32767, -(maxMotorValue), maxMotorValue);
  RY = map(RY, -32767, 32767, -(maxMotorValue), maxMotorValue);
  LX = map(LX, -32767, 32767, -(maxMotorValue), maxMotorValue);
  LY = map(LY, -32767, 32767, -(maxMotorValue), maxMotorValue);

  return 0;
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

void Move()  //STANDARD: digital write low pushes water out, high sucks in
{
  //Control for motors 0 and 1, back 2 thrusters
  if (LY != 0 && RX!= 0)
  {
    motor[0].Power = DirectionCorrection(0);
    motor[1].Power = DirectionCorrection(1);

    if (LY > 0)
    {
      digitalWrite(motor[0].directionPin, LOW);
      digitalWrite(motor[1].directionPin, LOW);
    }
    if (LY < 0)
    {
      digitalWrite(motor[0].directionPin, HIGH);
      digitalWrite(motor[1].directionPin, HIGH);
    }
    analogWrite(motor[0].PWMPin, abs(motor[0].Power));
    analogWrite(motor[1].PWMPin, abs(motor[1].Power));
  }

  else if (LY == 0 && RX != 0)
  {
    if(RX > 0)
    {
      digitalWrite(motor[0].directionPin, LOW);
      digitalWrite(motor[1].directionPin, HIGH);
    }

    if(RX < 0)
    {
      digitalWrite(motor[0].directionPin, HIGH);
      digitalWrite(motor[1].directionPin, LOW);
    }

    analogWrite(motor[0].PWMPin, abs(RX));
    analogWrite(motor[1].PWMPin, abs(RX));
  }

  else if (LY != 0 && RX  == 0)
  {
    if (LY > 0)
    {
      digitalWrite(motor[0].directionPin, LOW);
      digitalWrite(motor[1].directionPin, LOW);
    }
    if (LY < 0)
    {
      digitalWrite(motor[0].directionPin, HIGH);
      digitalWrite(motor[1].directionPin, HIGH);
    }

    analogWrite(motor[0].PWMPin, abs(LY));
    analogWrite(motor[1].PWMPin, abs(LY));
  }

  else {
    analogWrite(motor[0].PWMPin, 0);
    analogWrite(motor[1].PWMPin, 0);
  }



  //Control for side thrusters(4 and 5)
  if (LX >= 0)
  {
    digitalWrite(motor[4].directionPin, HIGH);
    digitalWrite(motor[5].directionPin, LOW);
  }

  if (LX < 0)
  {
    digitalWrite(motor[4].directionPin, LOW);
    digitalWrite(motor[5].directionPin, HIGH);
  }

  analogWrite(motor[4].PWMPin, abs(LX));
  analogWrite(motor[5].PWMPin, abs(LX));


  //Control for top thrusters(2 and 3)

  //linear movement
  if (LT > 0 && height <= maxMotorValue)
  {
    if(height < 0)
      height = 0;

    height += LT/10;

    if (height > LT)
      height = LT;
  }

  if(RT > 0 && height >= -(maxMotorValue))
  { 
    if(height > 0)
      height = 0;

    height -= RT/10;

    if (height < -RT)
      height = -RT;
  }

  if(LT == 0 && RT == 0)
  {
    height = 0;
  }

  if (height != 0 && RY != 0)
  {
    motor[2].Power = DirectionCorrection(2);
    motor[3].Power = DirectionCorrection(3);

    if(height > 0)
    {
      digitalWrite(motor[2].directionPin, LOW);
      digitalWrite(motor[3].directionPin, LOW);
    }

    if(height < 0)
    {
      digitalWrite(motor[2].directionPin, HIGH);
      digitalWrite(motor[3].directionPin, HIGH);
    }

    analogWrite(motor[2].PWMPin, abs(motor[2].Power));
    analogWrite(motor[3].PWMPin, abs(motor[3].Power));
  }

  else if (height == 0 && RY != 0)
  {
    if (RY > 0)
    {
      digitalWrite(motor[2].directionPin, LOW);
      digitalWrite(motor[3].directionPin, HIGH);
    }

    if (RY < 0)
    {
      digitalWrite(motor[2].directionPin, HIGH);
      digitalWrite(motor[3].directionPin, LOW);
    }

    analogWrite(motor[2].PWMPin, abs(RY));
    analogWrite(motor[3].PWMPin, abs(RY));
  }

  else if (height != 0 && RY == 0)
  {
    if(height > 0)
    {
      digitalWrite(motor[2].directionPin, LOW);
      digitalWrite(motor[3].directionPin, LOW);
    }

    if(height < 0)
    {
      digitalWrite(motor[2].directionPin, HIGH);
      digitalWrite(motor[3].directionPin, HIGH);
    }
    analogWrite(motor[2].PWMPin, abs(height));
    analogWrite(motor[3].PWMPin, abs(height));
  }

  else
  {
    analogWrite(motor[2].PWMPin, 0);
    analogWrite(motor[3].PWMPin, 0);
  }
}

void LeapMove()
{  
  int xVelocity = 0, yVelocity = 0, zVelocity = 0;
  xVelocity = map(palmVX, -300, 300, -(maxMotorValue)*.1, maxMotorValue*.1);
  constrain(xVelocity, -(maxMotorValue), maxMotorValue);

  yVelocity = map(palmVY, -300, 300, -(maxMotorValue)*.1, maxMotorValue*.1);
  constrain(yVelocity, -(maxMotorValue), maxMotorValue);

  zVelocity = map(palmVZ, -300, 300, -(maxMotorValue)*.1, maxMotorValue*.1);
  constrain(zVelocity, -(maxMotorValue), maxMotorValue);

  analogWrite(motor[0].PWMPin, abs(zVelocity));
  analogWrite(motor[1].PWMPin, abs(zVelocity));

  analogWrite(motor[2].PWMPin, abs(yVelocity));
  analogWrite(motor[3].PWMPin, abs(yVelocity));

  analogWrite(motor[4].PWMPin, abs(xVelocity));
  analogWrite(motor[5].PWMPin, abs(xVelocity));
}

int DirectionCorrection(int motor)
{
  int correctedValue = 0;

  if (motor == 0)
  {
    correctedValue = LY/2 + ((long)(LY*RX)/(2*maxMotorValue));
  }

  else if (motor == 1)
  {
    correctedValue = LY/2 - ((long)(LY*RX)/(2*maxMotorValue));
  }

  else if (motor == 2)
  {
    correctedValue = height/2 - ((long)(height*RY)/(2*maxMotorValue));
  }

  else if (motor == 3)
  {
    correctedValue = height/2 + ((long)(height*RY)/(2*maxMotorValue));
  }

  return correctedValue;
}

void SendInt(int val,long maxVal)
{
  int i = 0;
  String valStr = String(val);
  String maxValStr = String(maxVal);
  int sizeDif = maxValStr.length() - valStr.length();

  for (i = 0; i < sizeDif; i++)
    Serial.print("0");

  Serial.print(val);
  Serial.println(" ");
}

void ReadGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers

  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void SendData()
{
  long maxValue = 1234567;
  //Gyro Values:
  SendInt(AcX, maxValue);
  /*SendInt(AcY, maxValue);
   SendInt(AcZ, maxValue);
   SendInt(Tmp/340.00+36.53, maxValue);
   SendInt(GyX, maxValue);
   SendInt(GyY, maxValue);
   SendInt(GyZ, maxValue);*/
}







