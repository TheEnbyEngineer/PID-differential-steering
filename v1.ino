/*
External libraries
https://www.arduino.cc/reference/en/libraries/mpu6050_light/
https://www.arduino.cc/reference/en/libraries/pid/
*/

// Raspberry Pi PICO I2C Connections GP4: SDA   GP5: SCL
// Move counter-clockwise: Positive displacement

#include "Wire.h"
#include <MPU6050_light.h>
#include <PID_v1.h>

// Variables that does not need to be tuned
MPU6050 mpu(Wire);
unsigned long timer = 0;
double yawSetpoint = 0, yawInput = 0, yawOutput = 0;
bool doneYaw = true;

// Specify the links and initial tuning parameters (Yaw)
double yawProportional = 0.1, yawIntegral = 0.1, yawDerivative = 0.01, tolerance = 1;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawProportional, yawIntegral, yawDerivative, DIRECT);


void setup() {
  // Starting the hardware
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BUILTIN,OUTPUT);
  // Left motor
  pinMode(27, OUTPUT); // Forward
  pinMode(26, OUTPUT); // Back
  // Right motor
  pinMode(21, OUTPUT); // Forward
  pinMode(20, OUTPUT); // Back

  // Initalizing PID
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-255, 255);

  Serial.println("Press enter to start");
  while (!Serial.available()) {}

  // Initalizing the MPU6050
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  // Calibrating
  Serial.println(F("Calculating offsets, do not move the aircraft"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  digitalWrite(LED_BUILTIN,HIGH);

  // Setpoint
  if (doneYaw) {
    Serial.print("Setpoint (in angle): ");
    while (!Serial.available()) {} // Wait for user input
    yawSetpoint = Serial.readStringUntil('\n').toInt();
    doneYaw = false;
  }

  if ((millis()-timer)>10) { // Main control loop
    // Feedback
    yawInput = mpu.getAngleZ();

    // Compute PID
    yawPID.Compute();

    // Output
    if (yawOutput > 1) {
      analogWrite(27, map(yawOutput, 0, 255, 100, 255));
      analogWrite(26, 0);
      analogWrite(21, 0);
      analogWrite(20, map(yawOutput, 0, 255, 100, 255));
    }
    else if (yawOutput < -1) {
      analogWrite(27, 0);
      analogWrite(26, map(yawOutput*-1, 0, 255, 100, 255));
      analogWrite(21, map(yawOutput*-1, 0, 255, 100, 255));
      analogWrite(20, 0);
    }
    else {
      analogWrite(26, 0);
      analogWrite(27, 0);
      analogWrite(21, 0);
      analogWrite(20, 0);
    }

    // Shutdown when target is reached
    if (((yawInput - yawSetpoint) < tolerance) and ((yawInput - yawSetpoint) > tolerance*-1)) {
      yawSetpoint = yawInput; // Once the position is within tolerance, the setpoint would be set to the same value as the feedback position, even if it is not the same, to effective "shutdown" the controller
      doneYaw = true;
      analogWrite(26, 0);
      analogWrite(27, 0);
      analogWrite(21, 0);
      analogWrite(20, 0);
    }

    // PID output debug
    Serial.print("Target yaw: ");
    Serial.print(yawSetpoint);
    Serial.print(" Current Yaw: ");
    Serial.print(mpu.getAngleZ());
	  Serial.print(" Steering power: ");
    Serial.println(yawOutput);

	  timer = millis();
  }
}
