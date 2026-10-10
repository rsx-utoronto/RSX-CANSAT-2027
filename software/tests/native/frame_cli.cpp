#include <cansat/serial_frame.hpp>
#include <iostream>
#include <iterator>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main(int argc, char** argv) {
#ifdef _WIN32
    // Test exactly the same wire bytes on Windows; avoid CRLF translation.
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    if (argc == 2 && std::string{argv[1]} == "--decode") {
        const std::string input{std::istreambuf_iterator<char>{std::cin}, {}};
        const auto result = cansat::decodeSerialFrame(input);
        if (!result) return 2;
        std::cout << static_cast<char>(result->node) << '\n' << result->payload << '\n';
        return 0;
    }
    if (argc != 3 || std::string{argv[1]}.size() != 1) return 2;
    const auto result = cansat::encodeSerialFrame(static_cast<cansat::Node>(argv[1][0]), argv[2]);
    if (!result) return 2;
    std::cout << *result;
    return 0;
}
