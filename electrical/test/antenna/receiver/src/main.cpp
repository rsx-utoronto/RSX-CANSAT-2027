#include <esp_now.h>
#include <WiFi.h>



typedef struct struct_message {

    char deviceName[32];

    int sensorValue;

    float temperature;

    bool status;

    unsigned long timestamp;

} struct_message;



struct_message incomingMessage;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(incomingMessage)) return;
    memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));

    int rssi = info->rx_ctrl->rssi;   // in dBm

    Serial.println("=== Message Received ===");
    Serial.printf("Device: %s\n", incomingMessage.deviceName);
    Serial.printf("Sensor: %d\n", incomingMessage.sensorValue);
    Serial.printf("Temperature: %.2f°C\n", incomingMessage.temperature);
    Serial.printf("Status: %s\n", incomingMessage.status ? "Active" : "Inactive");
    Serial.printf("Timestamp: %lu\n", incomingMessage.timestamp);
    Serial.printf("RSSI: %d dBm\n", rssi);
    Serial.println("========================");
}


void setup() {

    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    Serial.println(WiFi.macAddress());

    

    if (esp_now_init() != ESP_OK) {

        Serial.println("Error initializing ESP-NOW");

        return;

    }

    

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Ready");

}



void loop() {

    delay(250);

}