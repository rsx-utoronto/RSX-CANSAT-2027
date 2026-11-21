#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include "protocol.hpp"

// Binary PQ packet serializer (PQ wire format still undecided, see the
// CHECK in protocol.hpp). The container does NOT use this file: it sends
// ASCII CSV built by container_telemetry.hpp. The ContainerPacket functions
// are commented out (reason below).

namespace Telemetry{
     const char* modeToString(Mode mode);
     const char* opStateToString(OpState state);

     // COMMENTED OUT, NOT DELETED (2026-11-21): the team chose ASCII CSV as the
     // over-the-air format, so the container no longer sends a binary packet
     // (wire-format rules at the top of firmware_lib/inc/protocol.hpp). Binary
     // packets can contain '\r'/'\n' bytes, which break the line framing used
     // by the radio bridge and the GUI. The container builds its CSV line in
     // software/firmware/container/include/container_telemetry.hpp instead.
     // Kept here, commented, in case the decision is revisited. Do not re-enable
     // without changing the bridge/GUI framing first.
     // bool serializeContainerPacket(
     //      const ContainerPacket& packet, 
     //      uint8_t* buffer, 
     //      size_t bufferSize,
     //      size_t& bytesWritten
     // );
     bool serializePQPacket(
          const PQPacket& packet, 
          uint8_t* buffer, 
          size_t bufferSize,
          size_t& bytesWritten
     );
     // Commented out with the binary ContainerPacket (ASCII CSV decision, see above).
     // bool deserializeContainerPacket(
     //      const uint8_t* buffer, 
     //      size_t bufferSize, 
     //      ContainerPacket& packet
     // );
     bool deserializePQPacket(
          const uint8_t* buffer,
          size_t bufferSize, 
          PQPacket& packet
     );
     // std::string containerPacketToString(const ContainerPacket& packet);  // commented out, see above
     std::string pqPacketToString(const PQPacket& packet);
     const char* pqPacketToStringHeader();
     // const char* containerPacketToStringHeader();  // commented out, see above
}