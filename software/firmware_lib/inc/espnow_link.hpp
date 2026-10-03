// common/include/espnow_link.hpp
//
// ESP-NOW transport, shared by all three boards.
//
// Topology (X3): both flight nodes peer to the ground station; the
// ground station peers to both flight nodes. The flight nodes never
// talk to each other.
//
//   Container  ──telemetry──▶  Ground Station
//              ◀──commands───
//   PocketQube ──telemetry──▶  Ground Station
//              ◀──commands───
//
// Registration is required to SEND to a MAC. Receiving is not gated by
// registration, which is why isKnownPeer() exists — see X4/X5.

#pragma once
#include <stdint.h>
#include <stddef.h>

// Called from the WiFi task when a packet arrives.
// MUST NOT block: copy the bytes, set a flag, return.
using RxHandler = void (*)(const uint8_t* mac, const uint8_t* data, int len);

namespace EspNowLink {

  // Radio setup only — no peers. Call onReceive() before this so a
  // packet arriving during init has somewhere to go.
  // Returns false on failure; caller should degrade, never hang.
  bool begin(uint8_t channel);

  // Register a peer. Call once per MAC you need to send to.
  // Re-registering an existing peer is a no-op success.
  bool addPeer(const uint8_t mac[6]);

  // Send to a registered peer. Max payload is 250 bytes.
  bool send(const uint8_t mac[6], const uint8_t* data, size_t len);

  // Register the receive handler.
  void onReceive(RxHandler handler);

  // True if this MAC has been registered via addPeer().
  bool isKnownPeer(const uint8_t mac[6]);

  // Diagnostics — useful in telemetry and for the range test.
  uint32_t sendFailures();
  uint32_t sendOk();
  uint32_t rxRejected();
  bool     isUp();

}  // namespace EspNowLink

// ---- Usage ------------------------------------------------------------
//
// Container and PocketQube are identical:
//   EspNowLink::onReceive(onRx);
//   EspNowLink::begin(ESPNOW_CHANNEL);
//   EspNowLink::addPeer(GS_MAC);
//   EspNowLink::send(GS_MAC, buf, len);
//
// Ground station bridge:
//   EspNowLink::onReceive(onRx);
//   EspNowLink::begin(ESPNOW_CHANNEL);
//   EspNowLink::addPeer(CONTAINER_MAC);
//   EspNowLink::addPeer(POCKETQUBE_MAC);
//   EspNowLink::send(CONTAINER_MAC, cmd, n);