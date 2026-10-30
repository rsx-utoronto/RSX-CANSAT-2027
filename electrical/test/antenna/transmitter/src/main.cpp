#include <esp_now.h>

#include <WiFi.h>




uint8_t receiverAddress[] = {0x8C, 0x94, 0xDF, 0x57, 0x46, 0x4C};


typedef struct struct_message {

    char deviceName[32];

    int packetNumber;

    unsigned long timestamp;

} struct_message;

int totalPackets = 100;
int packetNumber = 0;

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

    
    delay(5000);
    Serial.println("Sending packets!");

}



void loop() {
    if (packetNumber >= totalPackets) return;

    strcpy(outgoingMessage.deviceName, "Transmitterrr");

    outgoingMessage.packetNumber = packetNumber++; // lolz

    outgoingMessage.timestamp = millis();

    

    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));

    

    if (result == ESP_OK) {

        Serial.println("Sent successfully");

    } else {

        Serial.println("Error sending data");

    }

    

    delay(250);

}