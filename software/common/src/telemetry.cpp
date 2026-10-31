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
                         if (bytes==nullptr || count>bufferSize-position_) {ok_=false; return;}
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
     }
}