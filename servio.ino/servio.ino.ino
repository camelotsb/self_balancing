#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>

#define pitchServoPin 3
#define rollServoPin 4

Servo pitchServo; // create servo object for pitch servo
Servo rollServo;  // create servo object for roll servo

int MPU6050_addr = 0x68;                   // I2C address of the MPU6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // variables to store MPU6050 sensor data
float pitchSetpoint = 0.0;                 // desired pitch angle setpoint
float rollSetpoint = 0.0;                  // desired roll angle setpoint
float pitchInput, rollInput;               // current pitch and roll angles read from the sensor
float pitchOutput, rollOutput;             // angular speed at which servos must rotate
float Kp = 0.00008;                         // proportional gain
float Ki = 0.00003;                        // integral gain
float Kd = 0.00003;                         // derivative gain
float pitchErrorSum = 0.0;                 // cumulative error sum for pitch
float rollErrorSum = 0.0;                  // cumulative error sum for roll
float lastPitchError = 0.0;                // previous error for pitch
float lastRollError = 0.0;                 // previous error for roll
unsigned long lastTime = 0;                // last time the loop was run
unsigned long currentTime = 0;             // current time
unsigned long delTime = 0;             // current time

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU6050)
  Wire.endTransmission(true);
  pitchServo.attach(pitchServoPin); // attach pitch servo to pin
  rollServo.attach(rollServoPin);   // attach roll servo to pin
  Serial.print("Initiated");

}

void loop()
{
  currentTime = millis(); // get current time
 
  if (currentTime - lastTime >= 5)
  { // run loop every 20 milliseconds (50 Hz)

    // read MPU6050 sensor data
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_addr, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read();     // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();     // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    // Calculate pitch and roll angles using sensor data
    pitchInput = atan2(AcY, sqrt(AcX * AcX + AcZ * AcZ)) * 180.0 / PI;
    rollInput = atan2(-AcX, AcZ) * 180.0 / PI;

    // Calculate errors
    float pitchError = pitchSetpoint - pitchInput;
    float rollError = rollSetpoint - rollInput;

    // Calculate error sum
    pitchErrorSum += (pitchError * (currentTime - lastTime));
    rollErrorSum += (rollError * (currentTime - lastTime));

    // Calculate error rate of change
    float pitchErrorRate = (pitchError - lastPitchError) / (currentTime - lastTime);
    float rollErrorRate = (rollError - lastRollError) / (currentTime - lastTime);

    // Calculate PID outputs
    pitchOutput = Kp * pitchError + Ki * pitchErrorSum + Kd * pitchErrorRate;
    rollOutput = Kp * rollError + Ki * rollErrorSum + Kd * rollErrorRate;

    // Convert angular speed to angle for servos
    pitchOutput = pitchOutput * (180.0 / PI) * (currentTime - lastTime);
    rollOutput = rollOutput * (180.0 / PI) * (currentTime - lastTime);
    pitchOutput = max(min(pitchOutput,180),-90);
    rollOutput = max(min(rollOutput,180),-90);
    Serial.print("Roll: ");
    Serial.println(rollOutput + 90);
    // Apply PID outputs to servos
    pitchServo.write(pitchOutput + 90);
    rollServo.write(rollOutput + 90);

    // Store previous errors for next iteration
    lastPitchError = pitchError;
    lastRollError = rollError;
    lastTime = currentTime;
  }

  // Update current time
  currentTime = millis();

  // Print current pitch and roll to serial monitor

  // Wait a short period before repeating the loop
  delay(2);
}
