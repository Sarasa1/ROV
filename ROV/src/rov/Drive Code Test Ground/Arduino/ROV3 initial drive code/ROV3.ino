#include<Wire.h>
#include<Math.h>
#include <Servo.h>

// raw values passed from the C++ program. Kept as seperate variables mainly for intuitiveness
long motorCap = .5;
long leapCap = .10;

int RT, LT, R3, L3, RB, LB, UpD, DownD, LeftD, RightD, A, B, X, Y, Start, Back, Verification;
long RX, RY, LX, LY;
int palmVX = 0, palmVY = 0, palmVZ = 0, palmPitch = 0, palmYaw = 0, palmRoll, clawDistance = 0;
boolean usingLeap = false;

int RBPrevState = 0;
int i;
int height = 0;

Servo m0,m1,m2,m3,m4,m5;

//long minMotorValue = 1500 + (400 * motorCap);
//long maxMotorValue = 1500 - (400 * motorCap);
//long avgMotorValue = 1500;
long minMotorValue = 1700;
long maxMotorValue = 1300;
long avgMotorValue = 1500;
long minLeapValue = 1500;
long maxLeapValue = 1500;


void setup()
{
//  minMotorValue = 1500 + (400 * motorCap);
//  maxMotorValue = 1500 - (400 * motorCap);
//  minLeapValue = 1500 - (400 * leapCap);
//  maxLeapValue = 1500 + (400 * leapCap);
  
  InitializeESCs();
  
  Serial.begin(14400, SERIAL_8N1);  //Baud here must match baud on c++ side
  Serial.setTimeout(100);
}

void loop()
{
  int i = 0;
  
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
      StopROV();
    }    
  }
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

void StopROV()
{
 m0.writeMicroseconds(1500);
 m1.writeMicroseconds(1500);
 m2.writeMicroseconds(1500);
 m3.writeMicroseconds(1500);
 m4.writeMicroseconds(1500);
 m5.writeMicroseconds(1500);
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

  RX = map(RX, -32767, 32767, minMotorValue, maxMotorValue);
  RY = map(RY, -32767, 32767, minMotorValue, maxMotorValue);
  LX = map(LX, -32767, 32767, minMotorValue, maxMotorValue);
  LY = map(LY, -32767, 32767, minMotorValue, maxMotorValue);
  LT = map(LT, 0, 255, avgMotorValue, maxMotorValue);
  RT = map(RT, 0, 255, avgMotorValue, minMotorValue);
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

/*
 * Move
 *
 *
 *
 */
void Move()  //STANDARD:
{
  //Control for motors 0 and 1, back 2 thrusters
  // ROV is moving translationally or turning
//  if (LY != 0 || RX != 0)
//  {
//    m0.write((LY / 2) + (RX / 2));
//    m1.write((LY / 2) - (RX / 2));
//  }

  // ROV is moving translationally and not turning
  if (LY != 1500 && RX == 1500)
  {
    m0.writeMicroseconds(LY);
    m1.writeMicroseconds(LY);
  }
  
  else if (LY == 1500 && RX != 1500)
  {
    m0.writeMicroseconds(avgMotorValue + (RX - avgMotorValue));
    m1.writeMicroseconds(avgMotorValue - (RX - avgMotorValue));
  }
  
  else if (LY != 1500 && RX != 1500)
 {
   if (RX > 1500)
   {
     m0.writeMicroseconds(LY);
     m1.writeMicroseconds(RX-(((RX-1500)/200)*(LY-1500)));
   }
 } 
  // ROV isn't moving back thrusters
  else {
    m0.writeMicroseconds(1500);
    m1.writeMicroseconds(1500);
  }


  //Control for side thrusters(4 and 5)
  // ROV is moving left translationally
  if (LX >= maxMotorValue && LX < avgMotorValue)
  {
    m4.write(maxMotorValue - (LX - minMotorValue));
    m5.write(LX);
  }

  // ROV is moving right translationally
  else if (LX > avgMotorValue && LX <= minMotorValue)
  {
    m4.write(minMotorValue + (maxMotorValue - LX));
    m5.write(LX);
  }
  
  // ROV is not moving
  else if (LX == avgMotorValue) {
    m4.write(1500);
    m5.write(1500); 
  }



  // ROV is going up
  if (LT != 1500 && RT == 1500) {
    m2.writeMicroseconds(LT);
    m3.writeMicroseconds(LT);
  
  // ROV is going down
  } else if (LT == 1500 && RT != 1500) {
    m2.writeMicroseconds(RT); 
    m3.writeMicroseconds(RT);
  
  // ROV isn't going up or down
  } else if (LT == 1500 && RT == 1500) {
    m2.writeMicroseconds(1500);
    m3.writeMicroseconds(1500);
    
  } else {
    m2.writeMicroseconds(1500);
    m3.writeMicroseconds(1500);
  }
  
  

/*
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

  // ROV isn't ascending or descending
  if(LT == 0 && RT == 0)
  {
    height = 0;
  }

  if (height != 0 && RY != 0)
  {
    

    if(height > 0)
    {
      
    }

    if(height < 0)
    {
      
    }
  }

  else if (height == 0 && RY != 0)
  {
    if (RY > 0)
    {
      
    }

    if (RY < 0)
    {
      
    }
  }

  else if (height != 0 && RY == 0)
  {
    if(height > 0)
    {
      
    }

    if(height < 0)
    {
      
    }
  }

  else
  {
  }
  */
}

void LeapMove() // THIS NEED MORE LOGIC INVOLVED
{  
  int xVelocity = 0, yVelocity = 0, zVelocity = 0;
  xVelocity = map(palmVX, -300, 300, minLeapValue, maxLeapValue);
  constrain(xVelocity, minLeapValue, maxLeapValue);

  yVelocity = map(palmVY, -300, 300, minLeapValue, maxLeapValue);
  constrain(yVelocity, minLeapValue, maxLeapValue);

  zVelocity = map(palmVZ, -300, 300, minLeapValue, maxLeapValue);
  constrain(zVelocity, minLeapValue, maxLeapValue);

  m0.writeMicroseconds(zVelocity);
  m1.writeMicroseconds(zVelocity);
  m2.writeMicroseconds(yVelocity);
  m3.writeMicroseconds(yVelocity);
  m4.writeMicroseconds(xVelocity);
  m5.writeMicroseconds(xVelocity);
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








