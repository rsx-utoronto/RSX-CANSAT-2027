#include <esp_now.h>

#include <WiFi.h>



// Replace with receiver's MAC Address

uint8_t receiverAddress[] = {0x94, 0xB5, 0x55, 0x26, 0x27, 0x34};



typedef struct struct_message {

    char deviceName[32];

    int sensorValue;

    float temperature;

    bool status;

    unsigned long timestamp;

} struct_message;



struct_message outgoingMessage;

esp_now_peer_info_t peerInfo;


void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {

    Serial.print("Last Packet Send Status: ");

    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}



void setup() {

    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    

    if (esp_now_init() != ESP_OK) {

        Serial.println("Error initializing ESP-NOW");

        return;

    }

    

    esp_now_register_send_cb(OnDataSent);

    

    memcpy(peerInfo.peer_addr, receiverAddress, 6);

    peerInfo.channel = 0;

    peerInfo.encrypt = false;

    

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {

        Serial.println("Failed to add peer");

        return;

    }

}



void loop() {

    strcpy(outgoingMessage.deviceName, "Sensor Node 1");

    outgoingMessage.sensorValue = random(0, 100);

    outgoingMessage.temperature = random(200, 300) / 10.0;

    outgoingMessage.status = true;

    outgoingMessage.timestamp = millis();

    

    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));

    

    if (result == ESP_OK) {

        Serial.println("Sent successfully");

    } else {

        Serial.println("Error sending data");

    }

    

    delay(250);

}