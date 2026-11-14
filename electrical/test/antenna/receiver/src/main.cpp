#include <esp_now.h>
#include <WiFi.h>



typedef struct struct_message {

    char deviceName[32];

    int packetNumber;

    unsigned long timestamp;

} struct_message;

int freq = 4; // Hz
int totalPackets = 100;
int packetsReceived = 0;
unsigned long firstPacketTimestamp;
int rssiSum = 0;
bool logged = false;
struct_message incomingMessage;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(incomingMessage)) return;
    memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));

    int rssi = info->rx_ctrl->rssi;   // in dBm

    if (packetsReceived == 0) {
        firstPacketTimestamp = millis();
    }

    packetsReceived++;
    rssiSum += rssi;

    Serial.printf("%d,%lu,%d\n", incomingMessage.packetNumber, incomingMessage.timestamp, rssi);

}


void setup() {

    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.println("Mac Address");
    Serial.println(WiFi.macAddress());

    

    if (esp_now_init() != ESP_OK) {

        Serial.println("Error initializing ESP-NOW");

        return;

    }

    

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Ready");

}



void loop() {
    int timeElapsed = millis() - firstPacketTimestamp;
    int totalTransmitTime = 1000 * ((double) totalPackets / freq);
    if (!logged && packetsReceived > 0 && timeElapsed >= totalTransmitTime + 1) {
        int packetsLost = totalPackets - packetsReceived;
        double rssiAvg = (double) rssiSum / packetsReceived; 
        Serial.printf("Packets lost: %d\n", packetsLost);
        Serial.printf("% Packets lost: %f\n", (double) packetsLost / totalPackets);
        Serial.printf("Average RSSI: %f", rssiAvg);
        logged = true;
    }
}