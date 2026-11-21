/*
 * ESP32 + SparkFun MAX-M10S (UART) using the SparkFun u-blox GNSS Arduino Library.
 *
 * Reads and prints: GNSS_TIME, GNSS_LATITUDE, GNSS_LONGITUDE, GNSS_ALT, GNSS_SATS
 *
 * Wiring (UART). The library TALKS to the module, so BOTH data wires are required:
 *   Module TX  -> ESP32 GPIO26 (GNSS_RX_PIN)
 *   Module RX  -> ESP32 GPIO25 (GNSS_TX_PIN)
 *   3V3 -> 3V3, GND -> GND
 *
 * Serial Monitor: 115200 baud. Test outdoors with a clear view of the sky.
 */

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

static const int GNSS_RX_PIN = 26;  // ESP32 RX <- module TX
static const int GNSS_TX_PIN = 25;  // ESP32 TX -> module RX

// MAX-M10S defaults to 38400; 9600 is also tried in case it was changed.
static const uint32_t BAUDS_TO_TRY[] = {38400, 9600, 115200};

SFE_UBLOX_GNSS myGNSS;
bool gnssOk = false;

static bool connectGnss() {
  for (uint32_t baud : BAUDS_TO_TRY) {
    Serial.printf("Trying GNSS at %lu baud...\n", (unsigned long)baud);
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    delay(200);
    if (myGNSS.begin(Serial1)) {
      Serial.printf("GNSS connected at %lu baud.\n", (unsigned long)baud);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MAX-M10S GNSS test (SparkFun u-blox library)");

  gnssOk = connectGnss();
  if (!gnssOk) {
    Serial.println("GNSS not detected. Check BOTH wires (module TX->GPIO26, module RX->GPIO25),");
    Serial.println("3V3, GND, and that the breakout is set to UART. Will keep retrying.");
    return;
  }

  myGNSS.setUART1Output(COM_TYPE_UBX);  // binary only, no NMEA clutter
  myGNSS.setNavigationFrequency(1);     // 1 Hz
  myGNSS.setAutoPVT(true);              // module pushes position/time automatically
}

void loop() {
  if (!gnssOk) {
    delay(2000);
    gnssOk = connectGnss();
    if (gnssOk) {
      myGNSS.setUART1Output(COM_TYPE_UBX);
      myGNSS.setNavigationFrequency(1);
      myGNSS.setAutoPVT(true);
    }
    return;
  }

  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  // Returns true only when a new PVT message arrived since the last call.
  if (!myGNSS.getPVT()) {
    Serial.println("No new data from module (check wiring/power).");
    return;
  }

  // GNSS_SATS: satellites used in the position solution
  Serial.printf("GNSS_SATS: %d\n", myGNSS.getSIV());

  // GNSS_TIME: UTC time from the GNSS
  if (myGNSS.getTimeValid()) {
    Serial.printf("GNSS_TIME: %02d:%02d:%02d UTC", myGNSS.getHour(), myGNSS.getMinute(),
                  myGNSS.getSecond());
    if (myGNSS.getDateValid())
      Serial.printf("  (date %04d-%02d-%02d)", myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());
    Serial.println();
  } else {
    Serial.println("GNSS_TIME: (not valid yet)");
  }

  // Fix type: 0 = none, 2 = 2D, 3 = 3D
  uint8_t fix = myGNSS.getFixType();
  if (fix < 2) {
    Serial.println("No position fix yet (go outdoors and wait a few minutes).");
    Serial.println("-----");
    return;
  }

  // Library returns degrees * 1e-7 and altitude in millimetres
  Serial.printf("GNSS_LATITUDE: %.7f\n", myGNSS.getLatitude() / 1e7);
  Serial.printf("GNSS_LONGITUDE: %.7f\n", myGNSS.getLongitude() / 1e7);
  Serial.printf("GNSS_ALT: %.1f m\n", myGNSS.getAltitudeMSL() / 1000.0);
  Serial.println("-----");
}