#include <Wire.h>

#define MODE_PIN 4
bool useAngularMode = false;
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B //turn the gyro on
#define GYRO_ZOUT_H 0x47
#define GYRO_CONFIG 0x1B
#define TRIG 2
#define ECHO 3

void displaySpeed(float speedInKmh);
float getZRotation();
float getDistance();
void setupGyro();
float getTargetSpeedKmh();

void setup() {
  Serial.begin(19200);
  Wire.begin();
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  setupGyro();
}

void loop() {
  useAngularMode = digitalRead(MODE_PIN) == LOW;
  float speed = useAngularMode ? getTargetSpeedKmh_Angular() : getTargetSpeedKmh_Straight();
  
  // Round to 1 decimal place
  float roundedSpeed = round(fabs(speed) * 10) / 10.0;

  displaySpeed(roundedSpeed);

  delay(500); 
}


void displaySpeed(float speedInKmh) {
  for (int i = 0; i < 5; i++) {
    Serial.println();
  }
  Serial.print(speedInKmh);
  Serial.print(" km/h");
}

float getZRotation() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t gz = Wire.read() << 8 | Wire.read();  // Combine bytes
  float degreesPerSecond = gz / 131.0;
  return(degreesPerSecond);

}


  void setupGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Wake up MPU
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0);  // Set ±250 °/s range
  Wire.endTransmission(true);
}

float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.01715;
}

float getTargetSpeedKmh_Straight() {
  static float prevDist = getDistance();
  static unsigned long prevTime = millis();

  float currDist = getDistance();
  unsigned long currTime = millis();

  float deltaT = (currTime - prevTime) / 1000.0;
  float speedMps = (currDist - prevDist) / 100.0 / deltaT;
  float speedKmh = speedMps * 3.6;

  prevDist = currDist;
  prevTime = currTime;

  return speedKmh;
}

float getTargetSpeedKmh_Angular() {
  float angularDegPerSec = getZRotation();
  float angularRadPerSec = angularDegPerSec * 0.01745;
  float distanceMeters = getDistance() / 100.0;
  float speedMps = angularRadPerSec * distanceMeters;
  return speedMps * 3.6;
}
