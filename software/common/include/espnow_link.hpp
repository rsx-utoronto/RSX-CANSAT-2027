// common/include/espnow_link.hpp
#pragma once
#include <stdint.h>
#include <stddef.h>

using RxHandler = void (*)(const uint8_t* mac, const uint8_t* data, int len);

namespace EspNowLink {
  bool begin(uint8_t channel);                  // radio setup only
  bool addPeer(const uint8_t mac[6]);           // call once per peer
  bool send(const uint8_t mac[6],
            const uint8_t* data, size_t len);   // always explicit
  void onReceive(RxHandler handler);

  uint32_t sendFailures();
  bool     isUp();
}