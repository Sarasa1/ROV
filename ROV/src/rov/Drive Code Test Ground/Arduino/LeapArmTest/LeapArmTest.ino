#include<Wire.h>
#include<Math.h>
#include <Servo.h>

// raw values passed from the C++ program. Kept as seperate variables mainly for intuitiveness
int RT, LT, R3, L3, RB, LB, UpD, DownD, LeftD, RightD, A, B, X, Y, Start, Back, Verification;
float palmVX = 0, palmVY = 0, palmVZ = 0, palmPitch = 0, palmYaw = 0, palmRoll = 0, clawDistance = 0;
boolean usingLeap = false;
int RBPrevState = 0;
int i;
int PWMPinLocation[] = {
  3,5,6,9,10,11};
int height = 0;
long RX, RY, LX, LY;

Servo claw, roll;

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
  Serial.begin(14400, SERIAL_8N1);  //Baud here must match baud on c++ side
  Serial.setTimeout(100);

//  for (i = 0; i<= 6; i++)
//  {
//    motor[i].PWMPin = PWMPinLocation[i];  //PWM pin for motor 0 will be 3, corresponnds with the PWM PinLocation array
//    motor[i].directionPin = (i+32);  //Digital pin for motor 0 will be 32 and go up by one
//  }
  
  claw.attach(26);
  //roll.attach(27);
}

void loop()
{
  if(Serial.available() > 0)
  {
    if(ReadInValues()!= -1)
    {
    ControlArm();
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
  palmVX = Serial.parseFloat();
  palmVY = Serial.parseFloat();
  palmVZ = Serial.parseFloat();
  palmPitch = Serial.parseFloat();
  palmYaw = Serial.parseFloat();
  palmRoll = Serial.parseFloat();
  clawDistance = Serial.parseFloat();

  if (Verification != 9)
  {
    return -1;
  }

  else
  {
    Serial.print("q ");
  }

  RX = map(RX, -32767, 32767, -255, 255);
  RY = map(RY, -32767, 32767, -255, 255);
  LX = map(LX, -32767, 32767, -255, 255);
  LY = map(LY, -32767, 32767, -255, 255);

  return 0;
}

void ControlArm()
{
  float clawControl = 0;
  float rollControl = 0;
  palmRoll = palmRoll + 180;
  //palmRoll = abs(palmRoll);
  clawControl = map(clawDistance, 40, 100, 5, 175);
  clawControl = constrain(clawControl, 20, 120);
  
  rollControl = map (palmRoll, 0, 360, 0, 175);
  rollControl = constrain(rollControl, 20, 120);
  
  claw.write(clawControl);
  //roll.write(rollControl);
}
