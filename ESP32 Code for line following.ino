#include <Wire.h>

#define I2C_SLAVE_ADDR 0x04 // Address 4

int calculatePID (float weightedAverage, float setpoint)
{
  int u;
  float error;
  float kp = 0.4, ki = 0.35, kd = 0.2;
  float proportional, integral, differential;
  
  error = setpoint - weightedAverage;

  proportional = kp * error;
  integral = ki * error * error / 2.00;
  differential = kd;

  u = proportional + integral + differential;

  return u;
}

void drivingMovement (int leftMotor_speed, int rightMotor_speed, int steeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((steeringAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(steeringAngle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission(); // stop transmitting 
}

void setup() {
  Serial.begin(9600);
  Wire.begin();                
  pinMode(15, INPUT); //WHITE
  pinMode(4, INPUT);  //YELLOW
  pinMode(2, INPUT);  //PURPLE
  pinMode(14, INPUT); //PINK
  pinMode(27, INPUT); //GREEN
  pinMode(26, INPUT); //BLUE
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.begin(1000);

  int sensorWhite = analogRead(15); //Read sensor data
  int sensorYellow = analogRead(4);
  int sensorPurple = analogRead(2);
  int sensorPink = analogRead(14);  
  int sensorGreen = analogRead(27);
  int sensorBlue = analogRead(26);

  //Map sensor readings
  sensorWhite = map(sensorWhite, 2506, 4095, 100, 0);
  sensorYellow = map(sensorYellow, 2788, 4095, 100, 0);
  sensorPurple = map(sensorPurple, 1956, 4095, 100, 0);
  sensorPink = map(sensorPink, 2345, 4095, 100, 0);
  sensorGreen = map(sensorGreen, 2340, 4095, 100, 0);
  sensorBlue = map(sensorBlue, 2561, 4095, 100, 0);

  Serial.print(sensorPink);
  Serial.print(" ");
  Serial.print(sensorWhite);
  Serial.print(" ");
  Serial.print(sensorPurple);
  Serial.print(" ");
  Serial.print(sensorYellow);
  Serial.print(" ");
  Serial.print(sensorGreen);
  Serial.print(" ");
  Serial.print(sensorBlue);
  Serial.print(" ");

  int pid;
  float wAvg;

  wAvg = (sensorPink * -1.00 + sensorWhite * -0.66 + sensorPurple * -0.33 + sensorYellow * 0.33 + sensorGreen * 0.66 + sensorBlue * 1.00) / 6;
  pid = calculatePID(wAvg, 0); //Reference point - setpoint = 0

  float angle, startAngle = 81, c = 0.65;
  int leftSpeed, rightSpeed, baseSpeed = 75;

  angle = pid + startAngle;
      
  if (angle < 75)
  {
    rightSpeed = pid * c + baseSpeed;
    leftSpeed = baseSpeed - (pid * c);
    drivingMovement(leftSpeed, rightSpeed, angle);
  }
  else if (angle > 87)
  {
    leftSpeed = pid * c + baseSpeed;
    rightSpeed = baseSpeed - (pid * c);
    drivingMovement(leftSpeed, rightSpeed, angle);
  }
  else
  {
    drivingMovement(baseSpeed, baseSpeed, startAngle);
  }
}
