#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>

#define MAX_PACKETS 200
#define TIMEOUT_MS 3000 // no packets for this long => close out the phase
#define WIFI_CHANNEL 11 // Changed to Channel 11 to avoid heavy neighborhood interference

typedef struct struct_message
{
  char deviceName[32];
  int sensorValue;
  float temperature;
  bool status;
  unsigned long timestamp;
  uint8_t mode; // 0 = standard, 1 = Long Range
  uint16_t seq;
  uint16_t total; // packets the sender transmits in this mode
} struct_message;

struct ModeStats
{
  uint32_t received;
  int32_t rssiSum;
  uint16_t total; // reported by sender
  unsigned long lastRx;
  bool seen[MAX_PACKETS]; // dedupe by sequence number
};

ModeStats stats[2];
bool reported[2] = {false, false};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const char *modeName(int m) { return m == 1 ? "LONG RANGE" : "STANDARD"; }

void resetAll()
{
  portENTER_CRITICAL(&mux);
  memset(stats, 0, sizeof(stats));
  reported[0] = reported[1] = false;
  portEXIT_CRITICAL(&mux);
  Serial.println("--- Stats Reset: Ready for a new test run ---");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  if (len != sizeof(struct_message))
    return;

  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  if (msg.mode > 1 || msg.seq >= MAX_PACKETS)
    return;

  int rssi = info->rx_ctrl->rssi;

  // New test run starting (sender was reset) after a finished run: clear stats
  if (msg.mode == 0 && msg.seq == 0 && reported[0] && reported[1])
  {
    resetAll();
  }

  bool isNew = false;
  uint32_t count = 0;

  portENTER_CRITICAL(&mux);
  ModeStats &s = stats[msg.mode];
  s.total = msg.total;
  s.lastRx = millis();
  if (!s.seen[msg.seq])
  {
    s.seen[msg.seq] = true;
    s.received++;
    s.rssiSum += rssi;
    isNew = true;
  }
  count = s.received;
  portEXIT_CRITICAL(&mux);

  if (isNew)
  {
    // Extract the MAC address bytes dynamically from the incoming info struct
    const uint8_t *mac = info->src_addr;

    Serial.printf("[%s %u/%u] seq=%u RSSI: %d dBm | %s | MAC: %02X:%02X:%02X:%02X:%02X:%02X | Temp: %.2f C\n",
                  modeName(msg.mode), (unsigned)count, msg.total, msg.seq,
                  rssi, msg.deviceName,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  msg.temperature);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin(); // Forces network interface initialization

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Receiver ready. Waiting for ESP-NOW data...");

  Serial.print("My (RECEIVER) STA MAC Address is: ");
  Serial.println(WiFi.macAddress());
}

void loop()
{
  unsigned long now = millis();

  for (int m = 0; m < 2; m++)
  {
    uint32_t rx = 0;
    uint16_t tot = 0;
    int32_t rSum = 0;
    unsigned long lr = 0;
    bool alreadyReported = false;

    portENTER_CRITICAL(&mux);
    rx = stats[m].received;
    tot = stats[m].total;
    rSum = stats[m].rssiSum;
    lr = stats[m].lastRx;
    alreadyReported = reported[m];
    portEXIT_CRITICAL(&mux);

    if (rx > 0 && !alreadyReported && (now - lr >= TIMEOUT_MS))
    {
      float lossPercent = 100.0 * (1.0 - ((float)rx / (float)tot));
      float avgRssi = (float)rSum / (float)rx;

      Serial.println("\n==============================================");
      Serial.printf(" PHASE FINISHED: %s\n", modeName(m));
      Serial.println("==============================================");
      Serial.printf(" Packets Expected : %u\n", tot);
      Serial.printf(" Packets Received : %u\n", rx);
      Serial.printf(" Packet Loss     : %.2f%%\n", lossPercent);
      Serial.printf(" Average RSSI     : %.2f dBm\n", avgRssi);
      Serial.println("==============================================\n");

      portENTER_CRITICAL(&mux);
      reported[m] = true;
      portEXIT_CRITICAL(&mux);
    }
  }

  delay(100);
}
