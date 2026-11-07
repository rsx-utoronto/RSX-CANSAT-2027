#include <Arduino.h>
#include <Wire.h>

const uint8_t MMC5983MA = 0x30;
const uint8_t REG_OUT_0 = 0x00;     
const uint8_t REG_CONTROL_0 = 0x09; 
const uint8_t REG_CONTROL_1 = 0x0A; 
const uint8_t REG_CONTROL_2 = 0x0B; 

struct RawAxes {
  uint32_t x;
  uint32_t y;
  uint32_t z;
};

// Take and read a measurement
bool captureRawData(RawAxes &data) {
  // Take a measurement
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_CONTROL_0);
  Wire.write(0x01); 
  Wire.endTransmission();
  delay(12); 

  // Start reading output
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_OUT_0);
  Wire.endTransmission(false);

  if (Wire.requestFrom(MMC5983MA, (uint8_t)7) == 7) {
    uint8_t x0 = Wire.read();
    uint8_t x1 = Wire.read();
    uint8_t y0 = Wire.read();
    uint8_t y1 = Wire.read();
    uint8_t z0 = Wire.read();
    uint8_t z1 = Wire.read();
    uint8_t xyz2 = Wire.read();

    // Convert to 18-bit
    data.x = ((uint32_t)x0 << 10) | ((uint32_t)x1 << 2) | ((xyz2 >> 6) & 0x03);
    data.y = ((uint32_t)y0 << 10) | ((uint32_t)y1 << 2) | ((xyz2 >> 4) & 0x03);
    data.z = ((uint32_t)z0 << 10) | ((uint32_t)z1 << 2) | ((xyz2 >> 2) & 0x03);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22);
  Wire.setClock(400000); 
  delay(150); 

  // Software Reset
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_CONTROL_1);
  Wire.write(0x80); 
  Wire.endTransmission();
  delay(50); 

  // 18-bit mode
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_CONTROL_2);
  Wire.write(0x03); 
  Wire.endTransmission();
  delay(10);
}

void loop() {  
  RawAxes setPhase = {0, 0, 0};
  RawAxes resetPhase = {0, 0, 0};

  // SET pulse
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_CONTROL_0);
  Wire.write(0x08); 
  Wire.endTransmission();
  delay(15);

  // RESET pulse
  Wire.beginTransmission(MMC5983MA);
  Wire.write(REG_CONTROL_0);
  Wire.write(0x10); 
  Wire.endTransmission();
  delay(15);

  //offset
  double cleanX = ((double)setPhase.x - (double)resetPhase.x) / 2.0;
  double cleanY = ((double)setPhase.y - (double)resetPhase.y) / 2.0;
  double cleanZ = ((double)setPhase.z - (double)resetPhase.z) / 2.0;

  // range is +-8
  double gaussX = (cleanX / 131072.0) * 8.0;
  double gaussY = (cleanY / 131072.0) * 8.0;
  double gaussZ = (cleanZ / 131072.0) * 8.0;
  double magnitude = sqrt(gaussX*gaussX + gaussY*gaussY + gaussZ*gaussZ);

  Serial.print("X: "); Serial.print(gaussX, 5);
  Serial.print("\tY: "); Serial.print(gaussY, 5);
  Serial.print("\tZ: "); Serial.print(gaussZ, 5);
  Serial.print("\tTotal Field (G): "); Serial.println(magnitude, 5);

  delay(400);
}


// y offset first test: start 0.43774 end -0.40405
// y offeset second test: start 0.24896 end 1.09625