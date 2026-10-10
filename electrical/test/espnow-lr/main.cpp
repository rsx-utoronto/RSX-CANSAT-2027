#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0xF0, 0x16, 0x1D, 0x93, 0xBF, 0xB4};

const int myNumber = 2;
bool messageSent = false;

// Universal Receive Callback: Smart data parsing
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
#else
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int len)
#endif
{
  Serial.print("-> Reply Received: ");

  // If the other board sent a raw 1-byte number (like 0 or 1)
  if (len == 1)
  {
    uint8_t numericByte = data[0];
    Serial.println(numericByte); // Prints a clean "0" or "1"
  }
  // If the other board sent a raw 4-byte integer
  else if (len == 4)
  {
    int numericInt;
    memcpy(&numericInt, data, sizeof(numericInt));
    Serial.println(numericInt); // Prints a clean 4-byte integer
  }
  // Otherwise, treat it as an alphabet text message string
  else
  {
    String receivedMessage = ""; // Explicitly declared inside this block
    for (int i = 0; i < len; i++)
    {
      receivedMessage += (char)data[i];
    }
    Serial.println(receivedMessage);
  }
}

// Send Status Callback: Confirms if the number left your board successfully
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("<- Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    Serial.printf("Delivered successfully! Value sent: %d (Odd)\n", myNumber);
    Serial.println("Waiting for reply from the other board...");
  }
  else
  {
    Serial.println("Delivery Failed. (Is the target board powered on?)");
    messageSent = false; // Reset so it tries to resend in the loop if it failed
  }
}

void setup()
{
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register the Send Callback
  esp_now_register_send_cb(onDataSent);

// Register the Receive Callback
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceive));
#else
  esp_now_register_recv_cb(onDataReceive);
#endif

  // Register Peer (The target board)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("System Ready! Sending number 67...");
}

void loop()
{
  if (!messageSent)
  {
    messageSent = true;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myNumber, sizeof(myNumber));

    if (result != ESP_OK)
    {
      Serial.println("Error initiating send transaction");
      messageSent = false;
    }
  }

  delay(100);
}
