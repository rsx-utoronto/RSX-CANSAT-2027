
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BMP5xx bmp;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting BMP581...");

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!bmp.begin(0x47, &Wire) &&
        !bmp.begin(0x46, &Wire))
    {
        Serial.println("BMP581 not found!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("BMP581 connected!");

    // Put sensor in standby before configuration
    bmp.setPowerMode(BMP5XX_POWERMODE_STANDBY);

    bool t = bmp.setTemperatureOversampling(
        BMP5XX_OVERSAMPLING_2X);

    bool p = bmp.setPressureOversampling(
        BMP5XX_OVERSAMPLING_16X);

    bool o = bmp.setOutputDataRate(
        BMP5XX_ODR_10_HZ);

    bool e = bmp.enablePressure(true);

    bool m = bmp.setPowerMode(
        BMP5XX_POWERMODE_NORMAL);

    Serial.print("Temperature config: ");
    Serial.println(t ? "OK" : "FAILED");

    Serial.print("Pressure config: ");
    Serial.println(p ? "OK" : "FAILED");

    Serial.print("ODR config: ");
    Serial.println(o ? "OK" : "FAILED");

    Serial.print("Pressure enable: ");
    Serial.println(e ? "OK" : "FAILED");

    Serial.print("Power mode config: ");
    Serial.println(m ? "OK" : "FAILED");

    Serial.print("Current power mode: ");
    Serial.println((int)bmp.getPowerMode());

    Serial.println("Setup complete!");
}

void loop()
{
    Serial.println("Trying to read BMP581...");

    if (bmp.performReading())
    {
        Serial.print("Temperature: ");
        Serial.print(bmp.temperature);
        Serial.println(" C");

        Serial.print("Pressure: ");
        Serial.print(bmp.pressure);
        Serial.println(" hPa");
    }
    else
    {
        Serial.println("Reading failed!");
    }

    Serial.println("--------------------");

    delay(1000);
}
