#include "telemetry.hpp"
#include <cstring>
#include <sstream>
#include <iomanip>
#include <type_traits>

namespace Telemetry{
     namespace{
          class Writer{
               public:
                    Writer(uint8_t* buffer, size_t bufferSize)
                         : buffer_(buffer), bufferSize_(bufferSize), position_(0), ok_(buffer != nullptr) {}
                    template<typename T>
                    void put(T value){
                         static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value, "Type must be arithmetic or enum");
                         if (!ok_ || position_ + sizeof(T) > bufferSize_) return;
                         using Value=typename ScalarType<T>::type;
                         using Bits=typename BitsType<Value>::type;
                         Value scalar=static_cast<Value>(value);
                         Bits bits{};
                         std::memcpy(&bits, &scalar, sizeof(Value));
                         putUnsigned(static_cast<uint64_t>(bits), sizeof(Value));
                    }
                    void putBytes(const char* bytes, size_t count){
                         if (!ok_ || position_ + count > bufferSize_) return;
                         if (bytes==nullptr || count>bufferSize_-position_) {ok_=false; return;}
                         std::memcpy(buffer_ + position_, bytes, count);
                         position_ += count;
                    }
                    bool ok() const { return ok_; }
                    size_t size() const { return position_; }
               private:
                    template<typename T,bool IsEnum=std::is_enum<T>::value>
                    struct ScalarTypeImpl{using type=T;};

                    template<typename T>
                    struct ScalarTypeImpl<T,true>{using type=typename std::underlying_type<T>::type;};

                    template<typename T>
                    struct ScalarType{using type=typename ScalarTypeImpl<T>::type;};

                    template<typename T,bool IsFloat=std::is_floating_point<T>::value>
                    struct BitsTypeImpl{using type=typename std::make_unsigned<T>::type;};

                    template<typename T>
                    struct BitsTypeImpl<T,true>{using type=typename std::conditional<sizeof(T)==4, uint32_t, uint64_t>::type;};

                    template<typename T>
                    struct BitsType{using type=typename BitsTypeImpl<T>::type;};

                    void putUnsigned(uint64_t output, size_t count){
                         if (count>bufferSize_-position_) {ok_=false; return;}
                         for (size_t i=0;i<count;i++){
                              buffer_[position_++]=static_cast<uint8_t>((output>>(8*i))&0xFFu);
                         }
                    }
                    uint8_t* buffer_;
                    size_t bufferSize_;
                    size_t position_;
                    bool ok_;
          };
          class Reader{
               public:
                    Reader(const uint8_t* buffer, size_t bufferSize)
                         : buffer_(buffer), bufferSize_(bufferSize), position_(0), ok_(buffer != nullptr) {}
                    template<typename T>
                    bool get(T& output){
                         static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value, "Type must be arithmetic or enum");
                         using Value=typename ScalarType<T>::type;
                         using Bits=typename BitsType<Value>::type;
                         uint64_t raw=0;
                         if (!getUnsigned(raw, sizeof(Value))) return false;
                         Bits bits=static_cast<Bits>(raw);
                         Value scalar{};
                         std::memcpy(&scalar, &bits, sizeof(Value));
                         output=static_cast<T>(scalar);
                         return true;
                    }
                    bool getBytes(char* output, size_t count){
                         if (!ok_ || output==nullptr || count>bufferSize_-position_) {ok_=false; return false;}
                         std::memcpy(output, buffer_ + position_, count);
                         position_ += count;
                         return true;
                    }
                    bool finished() const { return ok_ && position_==bufferSize_; }
               private:
                    template<typename T,bool IsEnum=std::is_enum<T>::value>
                    struct ScalarTypeImpl{using type=T;};

                    template<typename T>
                    struct ScalarTypeImpl<T,true>{using type=typename std::underlying_type<T>::type;};

                    template<typename T>
                    struct ScalarType{using type=typename ScalarTypeImpl<T>::type;};

                    template<typename T,bool IsFloat=std::is_floating_point<T>::value>
                    struct BitsTypeImpl{using type=typename std::make_unsigned<T>::type;};

                    template<typename T>
                    struct BitsTypeImpl<T,true>{using type=typename std::conditional<sizeof(T)==4, uint32_t, uint64_t>::type;};

                    template<typename T>
                    struct BitsType{using type=typename BitsTypeImpl<T>::type;};

                    bool getUnsigned(uint64_t& output, size_t count){
                         if (!ok_ || count>bufferSize_-position_) {ok_=false; return false;}
                         uint64_t output=0;
                         for (size_t i=0;i<count;i++){
                              output|=(static_cast<uint64_t>(buffer_[position_++])<<(8*i));
                         }
                         return true;
                    }
                    const uint8_t* buffer_;
                    size_t bufferSize_;
                    size_t position_;
                    bool ok_;
          };
          std::string fixedText(const char* text, size_t capacity){
               size_t length=0;
               while (length<capacity && text[length]!='\0') {length++;}
               return std::string(text, length);
          }
          //read & write container
          void writeContainer(Writer& writer, const ContainerPacket& packet){
               writer.put(packet.team_id);
               writer.put(packet.mission_time_ms);
               writer.put(packet.packet_count);
               writer.put(packet.command_count);
               writer.put(packet.mode);
               writer.put(packet.altitude_m);
               writer.put(packet.pressure_pa);
               writer.put(packet.temperature_c);
               writer.put(packet.battery_v);
               writer.put(packet.battery_ma);
               writer.put(packet.mech_state);
               writer.put(packet.state);
               writer.putBytes(packet.cmd_echo, sizeof(packet.cmd_echo));
          }
          bool readContainer(Reader& reader, ContainerPacket& packet){
               return reader.get(packet.team_id) &&
                      reader.get(packet.mission_time_ms) &&
                      reader.get(packet.packet_count) &&
                      reader.get(packet.command_count) &&
                      reader.get(packet.mode) &&
                      reader.get(packet.altitude_m) &&
                      reader.get(packet.pressure_pa) &&
                      reader.get(packet.temperature_c) &&
                      reader.get(packet.battery_v) &&
                      reader.get(packet.battery_ma) &&
                      reader.get(packet.mech_state) &&
                      reader.get(packet.state) &&
                      reader.getBytes(packet.cmd_echo, sizeof(packet.cmd_echo));
          }
          void writePQ(Writer& writer, const PQPacket& packet){
               writer.put(packet.team_id);
               writer.put(packet.mission_time_ms);
               writer.put(packet.packet_count);
               writer.put(packet.command_count);
               writer.putBytes(packet.cmd_echo, sizeof(packet.cmd_echo));
               writer.put(packet.mode);
               writer.put(packet.altitude_m);
               writer.put(packet.pressure_pa);
               writer.put(packet.temperature_c);
               writer.put(packet.battery_v);
               writer.put(packet.battery_ma);
               writer.put(packet.rot_rate_x_dps);
               writer.put(packet.rot_rate_y_dps);
               writer.put(packet.rot_rate_z_dps);
               writer.put(packet.accel_x_mps2);
               writer.put(packet.accel_y_mps2);
               writer.put(packet.accel_z_mps2);
               writer.put(packet.mag_x_mG);
               writer.put(packet.mag_y_mG);
               writer.put(packet.mag_z_mG);
               writer.put(packet.gnss_latitude_deg);
               writer.put(packet.gnss_longitude_deg);
               writer.put(packet.gnss_altitude_m);
               writer.put(packet.gnss_time_s);
               writer.put(packet.gnss_sats);
               writer.put(packet.solar_1_v);
               writer.put(packet.solar_2_v);
               writer.put(packet.mech_state);
               writer.putBytes(packet.image_stabilization, sizeof(packet.image_stabilization));
               writer.putBytes(packet.science_exp, sizeof(packet.science_exp));
          }
          bool readPQ(Reader& reader, PQPacket& packet){
               return reader.get(packet.team_id) &&
                      reader.get(packet.mission_time_ms) &&
                      reader.get(packet.packet_count) &&
                      reader.get(packet.command_count) &&
                      reader.getBytes(packet.cmd_echo, sizeof(packet.cmd_echo)) &&
                      reader.get(packet.mode) &&
                      reader.get(packet.altitude_m) &&
                      reader.get(packet.pressure_pa) &&
                      reader.get(packet.temperature_c) &&
                      reader.get(packet.battery_v) &&
                      reader.get(packet.battery_ma) &&
                      reader.get(packet.rot_rate_x_dps) &&
                      reader.get(packet.rot_rate_y_dps) &&
                      reader.get(packet.rot_rate_z_dps) &&
                      reader.get(packet.accel_x_mps2) &&
                      reader.get(packet.accel_y_mps2) &&
                      reader.get(packet.accel_z_mps2) &&
                      reader.get(packet.mag_x_mG) &&
                      reader.get(packet.mag_y_mG) &&
                      reader.get(packet.mag_z_mG) &&
                      reader.get(packet.gnss_latitude_deg) &&
                      reader.get(packet.gnss_longitude_deg) &&
                      reader.get(packet.gnss_altitude_m) &&
                      reader.get(packet.gnss_time_s) &&
                      reader.get(packet.gnss_sats) &&
                      reader.get(packet.solar_1_v) &&
                      reader.get(packet.solar_2_v) &&
                      reader.get(packet.mech_state) &&
                      reader.getBytes(packet.image_stabilization, sizeof(packet.image_stabilization)) &&
                      reader.getBytes(packet.science_exp, sizeof(packet.science_exp));
          }
          bool serializeContainerPacket(const ContainerPacket& packet, uint8_t* buffer, size_t bufferSize, size_t& bytesWritten){
               bytesWritten=0;
               if (buffer==nullptr) {return false;}
               Writer writer(buffer, bufferSize);
               writeContainer(writer, packet);
               if (!writer.ok()) {return false;}
               bytesWritten=writer.size();
               return true;
          }
          bool serializePQPacket(const PQPacket& packet, uint8_t* buffer, size_t bufferSize, size_t& bytesWritten){
               bytesWritten=0;
               if (buffer==nullptr) {return false;}
               Writer writer(buffer, bufferSize);
               writePQ(writer, packet);
               if (!writer.ok()) {return false;}
               bytesWritten=writer.size();
               return true;
          }
          bool deserializeContainerPacket(const uint8_t* buffer, size_t bufferSize, ContainerPacket& packet){
               Reader reader(buffer, bufferSize);
               ContainerPacket tempPacket;
               if (!readContainer(reader, tempPacket) || !reader.finished()) {return false;}
               packet = tempPacket;
               return true;
          }
          bool deserializePQPacket(const uint8_t* buffer, size_t bufferSize, PQPacket& packet){
               Reader reader(buffer, bufferSize);
               PQPacket tempPacket;
               if (!readPQ(reader, tempPacket) || !reader.finished()) {return false;}
               packet = tempPacket;
               return true;
          }
          const char* modeToString(Mode mode){
               switch (mode){
                    case Mode::FLIGHT: return "F";
                    case Mode::SIMULATION: return "S";
                    default: return "UNKNOWN";
               }
          }
          const char* opStateToString(OpState state){
               switch (state){
                    case OpState::LAUNCH_PAD: return "LAUNCH_PAD";
                    case OpState::ASCENT: return "ASCENT";
                    case OpState::APOGEE: return "APOGEE";
                    case OpState::PQ_RELEASE: return "PQ_RELEASE";
                    default: return "UNKNOWN";
               }
          }
          const char* containerPacketToStringHeader(){
               return "ID,MISSION_TIME,PACKET_COUNT,COMMAND_COUNT,MODE,ALTITUDE,PRESSURE,TEMPERATURE,BATTERY_V,BATTERY_MA,MECH_STATE,STATE,CMD_ECHO";
          }
          const char* pqPacketToStringHeader(){
               return "ID,MISSION_TIME,PACKET_COUNT,COMMAND_COUNT,CMD_ECHO,MODE,ALTITUDE,PRESSURE,TEMPERATURE,BATTERY_V,BATTERY_MA,ROT_RATE_X,ROT_RATE_Y,ROT_RATE_Z,ACCEL_X,ACCEL_Y,ACCEL_Z,MAG_X,MAG_Y,MAG_Z,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_TIME,GNSS_SATS,SOLAR_1,SOLAR_2,MECH_STATE,IMAGE_STABILIZATION,SCIENCE_EXP";
          }
          std::string containerPacketToString(const ContainerPacket& packet){
               std::ostringstream out;
               out << packet.team_id << "," 
               << packet.mission_time_ms << ','
               << packet.packet_count << ','
               << packet.command_count << ','
               << modeToString(packet.mode) << ','
               << packet.altitude_m << ','
               << packet.pressure_pa << ','
               << packet.temperature_c << ','
               << packet.battery_v << ','
               << packet.battery_ma << ','
               << static_cast<unsigned int>(packet.mech_state) << ','
               << opStateToString(packet.state) << ','
               << fixedText(packet.cmd_echo, sizeof(packet.cmd_echo));
               return out.str();
          }
          std::string pqPacketToString(const PQPacket& packet){
               std::ostringstream out;
               out << std::fixed;
               out << packet.team_id << ","
               << packet.mission_time_ms << ','
               << packet.packet_count << ','
               << packet.command_count << ','
               << fixedText(packet.cmd_echo, sizeof(packet.cmd_echo)) << ','
               << modeToString(packet.mode) << ','
               << packet.altitude_m << ','
               << packet.pressure_pa << ','
               << packet.temperature_c << ','
               << packet.battery_v << ','
               << packet.battery_ma << ','
               << packet.rot_rate_x_dps << ','
               << packet.rot_rate_y_dps << ','
               << packet.rot_rate_z_dps << ','
               << packet.accel_x_mps2 << ','
               << packet.accel_y_mps2 << ','
               << packet.accel_z_mps2 << ','
               << packet.mag_x_mG << ','
               << packet.mag_y_mG << ','
               << packet.mag_z_mG << ','
               << packet.gnss_latitude_deg << ','
               << packet.gnss_longitude_deg << ','
               << packet.gnss_altitude_m << ','
               << packet.gnss_time_s << ','
               << packet.gnss_sats << ','
               << packet.solar_1_v << ','
               << packet.solar_2_v << ','
               << static_cast<unsigned int>(packet.mech_state) << ',';
               //<< "IMAGE_STABILIZATION" << ','
               //<< "SCIENCE_EXP";
               return out.str();
          }
     }
}