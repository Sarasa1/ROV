#define NUM_AVG_VAL 25

// cal constant air = .01027
// cal constant water = .04854
int analogPin = 0;
float val = 0;
float avg = 0;
//float calConstant = 0.01027;
float calConstant = .047921;
int i = 0;
float prevVals[NUM_AVG_VAL];
float distance = 0;

void setup()
{
  Serial.begin(9600);
  Serial.print("Send any value to start test\n");
}

void loop()

{
  if (Serial.available() > 0) {
    val = analogRead(analogPin);
    Serial.print("Value: ");
    Serial.print(val);
    Serial.print(" mV");
    Serial.print("\t");
    distance = calConstant * val;
    Serial.print(distance);
    Serial.print(" meters");
    Serial.print("\t");
    prevVals[i] = distance;
    i++;
    avg = average(prevVals);
    Serial.print("Average is: ");
    Serial.print(avg);
    Serial.print(" meters\n");

    if (i == NUM_AVG_VAL) {
      i = 0;
    }
  }
}

float average(float prevVals[])
{
  float sum = 0;
  for (int i = 0; i < NUM_AVG_VAL; i++) {
    sum += prevVals[i];
  }
  
  return (sum / NUM_AVG_VAL);
}

