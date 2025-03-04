#ifndef _PACKET_HPP_
#define _PACKET_HPP_

#include <Arduino.h>
#include <cstddef>

namespace {
constexpr size_t packet_len = 256;
}  // namespace

class P2OSPacket {
private:
    HardwareSerial* pioneer_serial;

public:
    P2OSPacket(HardwareSerial& pioneer_serial);
    ~P2OSPacket();

    void set_pioneer_serial(HardwareSerial& pioneer_serial);

    unsigned char packet[packet_len];
    unsigned char size;

    int CalcChkSum();

    void Print();
    void PrintHex();
    int  Build(unsigned char* data, unsigned char datasize);
    int  Send();
    int  Receive();
    bool Check();

    bool operator!=(P2OSPacket p) {
        if (size != p.size) {
            return true;
        }

        if (memcmp(packet, p.packet, size) != 0) {
            return true;
        }

        return false;
    }
};

#endif  // _PACKET_HPP_
