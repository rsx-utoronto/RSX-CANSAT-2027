#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include "protocol.hpp"

namespace Telemetry{
     const char* modeToString(Mode mode);
     const char* opStateToString(OpState state);

     bool serializeContainerPacket(
          const ContainerPacket& packet, 
          uint8_t* buffer, 
          size_t bufferSize
     );
     bool serializePQPacket(
          const PQPacket& packet, 
          uint8_t* buffer, 
          size_t bufferSize
     );
     bool deserializeContainerPacket(
          const uint8_t* buffer, 
          size_t bufferSize, 
          ContainerPacket& packet
     );
     bool deserializePQPacket(
          const uint8_t* buffer,
          size_t bufferSize, 
          PQPacket& packet
     );
     std::string containerPacketToString(const ContainerPacket& packet);
     std::string pqPacketToString(const PQPacket& packet);
     const char* pqPacketToStringHeader();
     const char* containerPacketToStringHeader();
}