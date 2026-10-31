#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

uint8_t receiverAddress[] = {0x68, 0x09, 0x47, 0x07, 0xCF, 0x70};

#define PACKETS_PER_MODE 60
#define SEND_INTERVAL_MS 300 // Increased from 100 to 300ms to stop overloading the receiver queue
#define WIFI_CHANNEL 11      // Changed from 1 to 11 to get away from crowded WiFi router signals

#define MODE_STANDARD 0
#define MODE_LR 1

typedef struct struct_message
{
  char deviceName[32];
  int sensorValue;
  float temperature;
  bool status;
  unsigned long timestamp;
  uint8_t mode;
  uint16_t seq;
  uint16_t total;
} struct_message;

struct_message outgoingMessage;
esp_now_peer_info_t peerInfo = {};

volatile uint8_t currentMode = MODE_STANDARD;
uint32_t packetsSent[2] = {0, 0};
volatile uint32_t macAckFailed[2] = {0, 0};

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS)
  {
    macAckFailed[currentMode]++;
  }
}

void setProtocolForMode(uint8_t mode)
{
  if (mode == MODE_LR)
  {
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  }
  else
  {
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  }
}

void runPhase(uint8_t mode)
{
  currentMode = mode;
  setProtocolForMode(mode);
  delay(500);

  Serial.printf("\n--- Sending %d packets in %s mode ---\n",
                PACKETS_PER_MODE, mode == MODE_LR ? "LONG RANGE" : "STANDARD");

  for (uint16_t seq = 0; seq < PACKETS_PER_MODE; seq++)
  {
    strcpy(outgoingMessage.deviceName, "Sensor Node 1");
    outgoingMessage.sensorValue = random(0, 100);
    outgoingMessage.temperature = random(200, 300) / 10.0;
    outgoingMessage.status = true;
    outgoingMessage.timestamp = millis();
    outgoingMessage.mode = mode;
    outgoingMessage.seq = seq;
    outgoingMessage.total = PACKETS_PER_MODE;

    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
    if (result == ESP_OK)
    {
      packetsSent[mode]++;
    }
    else
    {
      Serial.printf("Error sending packet %u\n", seq);
    }
    delay(SEND_INTERVAL_MS);
  }

  delay(1000);
  Serial.printf("%s phase done: sent=%u, MAC ACK failures=%u\n",
                mode == MODE_LR ? "LR" : "Standard",
                (unsigned)packetsSent[mode], (unsigned)macAckFailed[mode]);
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Set the station mode and spin up the interface
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin(); // Forces network interface initialization to prevent 00:00:00:00:00:00

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

  // PRINT SENDER MAC ADDRESS HERE
  Serial.print("\n>>> MY SENDER MAC ADDRESS IS: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW Transmitter Ready, starting test in 3 s...");
  delay(3000);

  runPhase(MODE_STANDARD);
  delay(1500);
  runPhase(MODE_LR);

  Serial.println("\n========== SENDER SUMMARY ==========");
  Serial.printf("Standard: %u packets sent\n", (unsigned)packetsSent[MODE_STANDARD]);
  Serial.printf("LR:       %u packets sent\n", (unsigned)packetsSent[MODE_LR]);
  Serial.println("Compare against the receiver's received counts.");
  Serial.println("Press reset on BOTH boards to run again.");
}

void loop()
{
  delay(1000);
}
