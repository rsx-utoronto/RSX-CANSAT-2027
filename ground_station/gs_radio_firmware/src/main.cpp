#include <Arduino.h>
#include "config.hpp"
#include "espnow_link.hpp"

static char lineBuf[300];
static int  lineLen = 0;

// From the WiFi task — copy and flag only, never block here.
static volatile bool rxPending = false;
static uint8_t  rxBuf[250];
static volatile int rxLen = 0;
static volatile char rxTag = '?';

static void onRx(const uint8_t* mac, const uint8_t* data, int len) {
  if (rxPending) return;

  char tag;
  if      (memcmp(mac, CONTAINER_MAC,  6) == 0) tag = 'C';
  else if (memcmp(mac, POCKETQUBE_MAC, 6) == 0) tag = 'P';
  else return;                     // unknown sender — drop it

  rxTag = tag;
  memcpy(rxBuf, data, len);
  rxLen = len;
  rxPending = true;
}

void setup() {
  Serial.begin(115200);
  EspNowLink::onReceive(onRx);
  EspNowLink::begin(ESPNOW_CHANNEL);
  EspNowLink::addPeer(CONTAINER_MAC);
  EspNowLink::addPeer(POCKETQUBE_MAC);
}

void loop() {
  // radio -> Pi
  if (rxPending) {
    Serial.write(rxTag);
    Serial.write(':');
    Serial.write(rxBuf, rxLen);
    Serial.write('\n');
    rxPending = false;
  }

  // Pi -> radio
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 2 && lineBuf[1] == ':') {
        const uint8_t* dest = (lineBuf[0] == 'C') ? CONTAINER_MAC : POCKETQUBE_MAC;
        EspNowLink::send(dest, (uint8_t*)(lineBuf + 2), lineLen - 2);
      }
      lineLen = 0;
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}