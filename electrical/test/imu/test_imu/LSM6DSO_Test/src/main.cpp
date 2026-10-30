#include <Wire.h>

#include "SparkFunLSM6DSO.h"

LSM6DSO myIMU;

void setup() {
  Serial.begin(115200);
  Serial.println("=== NEW DIAGNOSTIC VERSION ===");
  delay(500);

  Wire.begin(21, 22);

  if (!myIMU.begin(0x6B, Wire)) {
    Serial.println("IMU not detected!");
    while (1) {
      delay(1000);
    }
  }

  myIMU.initialize(BASIC_SETTINGS);

  Serial.println("LSM6DSO detected!");
}

void loop() {
  Serial.print("Accel (g): ");
  Serial.print(myIMU.readFloatAccelX(), 3);
  Serial.print(", ");
  Serial.print(myIMU.readFloatAccelY(), 3);
  Serial.print(", ");
  Serial.println(myIMU.readFloatAccelZ(), 3);

  Serial.print("Gyro (deg/s): ");
  Serial.print(myIMU.readFloatGyroX(), 3);
  Serial.print(", ");
  Serial.print(myIMU.readFloatGyroY(), 3);
  Serial.print(", ");
  Serial.println(myIMU.readFloatGyroZ(), 3);

  Serial.println("---");
  delay(500);
}
