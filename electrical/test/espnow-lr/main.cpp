#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Compile-time check: If this fails to build, PlatformIO is still targeting Core 2!
static_assert(ESP_ARDUINO_VERSION_MAJOR >= 3, "Not building with Arduino core 3");

uint8_t broadcastAddress[] = {0xF0, 0x16, 0x1D, 0x93, 0xBF, 0xB4};

const int myNumber = 69;
bool messageSent = false;

// ---- Core 3.x Receive Callback ----
// The first argument must be 'const esp_now_recv_info_t *'
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  Serial.print("-> Reply Received: ");

  if (len == 1)
  {
    Serial.println(data[0]);
  }
  else if (len == 4)
  {
    int numericInt;
    memcpy(&numericInt, data, sizeof(numericInt));
    Serial.println(numericInt);
  }
  else
  {
    String receivedMessage = "";
    for (int i = 0; i < len; i++)
    {
      receivedMessage += (char)data[i];
    }
    Serial.println(receivedMessage);
  }
}

// ---- Core 3.x Send Callback ----
// The first argument must be 'const esp_now_send_info_t *'
void onDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
  Serial.print("<- Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    Serial.printf("Delivered successfully! Value sent: %d\n", myNumber);
    Serial.println("Waiting for reply from the other board...");
  }
  else
  {
    Serial.println("Delivery Failed. (Is the target board powered on?)");
    messageSent = false;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1500); // Give the serial monitor window time to connect after resetting

  Serial.println("\n--- RUNTIME FIRMWARE VERSION TEST ---");
// This macro test checks what core the compiled binary is actively executing on
#ifdef ESP_ARDUINO_VERSION_MAJOR
  Serial.printf("CORE STATUS: Running on Arduino Core v%d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH);

  if (ESP_ARDUINO_VERSION_MAJOR >= 3)
  {
    Serial.println("TEST PASSED: The microcontroller is forcing Core 3 framework execution!");
  }
  else
  {
    Serial.println("TEST FAILED: Microcontroller is still executing an older Core 2 framework!");
  }
#else
  Serial.println("TEST FAILED: Ancient framework core version (Pre-Core 2) detected.");
#endif
  Serial.println("-------------------------------------\n");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register callbacks natively under Core 3.x
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  // Peer configuration
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop()
{
  // Main function: Sends the payload if it hasn't been sent yet
  if (!messageSent)
  {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myNumber, sizeof(myNumber));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
      messageSent = true;
    }
    else
    {
      Serial.println("Error sending the data");
    }

    delay(5000); // Wait 5 seconds before a retry if transmission fails
  }
}
