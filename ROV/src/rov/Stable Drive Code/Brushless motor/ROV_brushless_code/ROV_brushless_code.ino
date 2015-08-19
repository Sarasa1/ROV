#include<Wire.h>
#include <Servo.h>

//Gyro and accelerometer:
/*****************************************************************
WIRING OF MPU-6050
VCC: +3.3V From Arduino (attach 0.1 microFarad capacitor to ground)
GND: GND
SCL: SCL
SDA: SDA
XDA: Not connected
XCL: Not connected
ADO: GND
INT: Digital Pin 2 on Arduino
******************************************************************/


//Arduino global variables:
Servo m0,m1,m2,m3,m4,m5;
long minMotorValue = -400;
long maxMotorValue = 400;
long avgMotorValue = 0;

//Raw values passed from the C++ program. Kept as seperate variables mainly for intuitiveness
int RT, LT, R3, L3, RB, LB, UpD, DownD, LeftD, RightD, A, B, X, Y, Start, Back, Verification;
int RBPrevState = 0;
long RX, RY, LX, LY;

//Leap motion controller values passed from the C++ program
int palmVX = 0, palmVY = 0, palmVZ = 0, palmPitch = 0, palmYaw = 0, palmRoll = 0, clawDistance = 0;
boolean usingLeap = false;

void setup()
{
  InitializeESCs();
  
  Serial.begin(115200, SERIAL_8N1);  //Baud here must match baud on c++ side
  Serial.setTimeout(100);
}

void loop()
{
  if(Serial.available() > 0)
  {
    if(ReadInValues()!= -1)
    {
      if (DriveControl() == 1)
      {
        Move();
      }

//      else if (DriveControl() == -1)
//      {
//        LeapMove();
//      }
    }

    else
    {
      StopROV();
    }    
  }
}

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

  RX = map(RX, -32767, 32767, -200, 200);
  RY = map(RY, -32767, 32767, -200, 200);
  LX = map(LX, -32767, 32767, -200, 200);
  LY = map(LY, -32767, 32767, -200, 200);
  LT = map(LT, 0, 255, 0, 200);
  RT = map(RT, 0, 255, 0, -200);
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
  //motor 4 and 5 control
  writeMotor(m4, -(LX));
  writeMotor(m5, LX);

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
 
  delay(1500); 
}

void writeMotor(Servo motor, int value)
{
  motor.writeMicroseconds((1500 + value));
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
