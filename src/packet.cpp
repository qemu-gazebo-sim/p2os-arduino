#include "p2os_config.hpp"
#include <Arduino.h>
#include <packet.hpp>

P2OSPacket::P2OSPacket(HardwareSerial& debug_serial, HardwareSerial& pioneer_serial) {
    this->debug_serial = &debug_serial;
    this->pioneer_serial = &pioneer_serial;
}

P2OSPacket::~P2OSPacket() { /** Destructor **/ }

void P2OSPacket::set_pioneer_serial(HardwareSerial& pioneer_serial) {
    this->pioneer_serial = &pioneer_serial;
}

void P2OSPacket::Print() {
#ifdef P2OS_DEBUG_PRINT
    if (this->packet) {
        for (int i = 0; i < this->size; i++) {
            this->debug_serial->printf("%u ", packet[i]);
        }
        this->debug_serial->printf("\n");
    }
#endif
}

void P2OSPacket::PrintHex() {
#ifdef P2OS_DEBUG_PRINT
    if (this->packet) {
        for (int i = 0; i < this->size; i++) {
            this->debug_serial->printf("0x%.2x ", packet[i]);
        }
        this->debug_serial->printf("\n");
    }
#endif
}

bool P2OSPacket::Check() {
    const int16_t chksum = CalcChkSum();
    return (chksum == (this->packet[this->size - 2] << 8)) | this->packet[this->size - 1];
}

int P2OSPacket::CalcChkSum() {
    unsigned char* buffer = &(this->packet[3]);
    int            c = 0;
    int            n;

    for (n = this->size - 5; n > 1;) {
        c += (*(buffer) << 8) | *(buffer + 1);
        c = c & 0xffff;
        n -= 2;
        buffer += 2;
    }

    if (n > 0) {
        c ^= static_cast<int>(*(buffer++));
    }

    return c;
}

int P2OSPacket::Receive() {
    unsigned char prefix[3];
    int           cnt;

    ::memset(this->packet, 0, sizeof(this->packet));

    do {
        ::memset(prefix, 0, sizeof(prefix));

        int retries_0 = 100;
        while (1) {
            cnt = 0;
            int retries_1 = 50;
            int read_result;
            while (cnt != 1 && retries_1 > 0) {
                if (this->pioneer_serial->available()) {
                    read_result = this->pioneer_serial->read(&prefix[2], 1);
                    cnt += read_result;
                    if (cnt < 0) {
#ifdef P2OS_ERROR_PRINT
                        this->debug_serial->printf(
                            "Error: error reading packet.header from robot connection: P2OSPacket():Receive():read():\n"
                        );
#endif
                        return 1;
                    }
                } else {
                    // retries_1--;
                    cnt = 0;
                    if (retries_1 < 1) {
#ifdef P2OS_ERROR_PRINT
                        this->debug_serial->printf("Error: timout reading packet.header from robot connection: "
                                                   "P2OSPacket():Receive():read():\n");
#endif
                        return 1;
                    }
                }
                // this->debug_serial->printf("%i \n", read_result);
            }

            // this->debug_serial->printf("%i", prefix[2]);
            // this->debug_serial->println("Receive 3");
            if (prefix[0] == 0xFA && prefix[1] == 0xFB) {
                break;
            }
            prefix[0] = prefix[1];
            prefix[1] = prefix[2];
            // skipped++;

            retries_0--;
            if (retries_0 < 0) {
#ifdef P2OS_ERROR_PRINT
                this->debug_serial->printf("Error: timout retried: P2OSPacket():Receive():read():\n");
#endif
                return 1;
            }
        }
        // // if (skipped>3) ROS_INFO("Skipped %d bytes\n", skipped);

        this->size = prefix[2] + 3;
        memcpy(this->packet, prefix, 3);

        cnt = 0;
        while (cnt != prefix[2]) {
            if (this->pioneer_serial->available()) {
                int read_result = this->pioneer_serial->read(&packet[3 + cnt], prefix[2] - cnt);
                cnt += read_result;

                if (cnt < 0) {
#ifdef P2OS_ERROR_PRINT
                    this->debug_serial->printf(
                        "Error reading packet body from robot connection: P2OSPacket():Receive():read():\n"
                    );
#endif
                    return 1;
                }
            } else {
#ifdef P2OS_ERROR_PRINT
                this->debug_serial->printf(
                    "Error: reading packet.body from robot connection: P2OSPacket():Receive():read():\n"
                );
#endif
            }
        }
#ifdef P2OS_DEBUG_PRINT
        this->debug_serial->println("Received:");
        this->PrintHex();
#endif
    } while (!Check());

    return 0;
}

int P2OSPacket::Build(unsigned char* data, unsigned char datasize) {
    int16_t chksum;

    this->size = datasize + 5;

    /* header */
    this->packet[0] = 0xFA;
    this->packet[1] = 0xFB;

    if (this->size > 198) {
#ifdef P2OS_ERROR_PRINT
        this->debug_serial->printf("Error: Packet to P2OS can't be larger than 200 bytes\n");
#endif
        return 1;
    }
    this->packet[2] = datasize + 2;

    memcpy(&(this->packet[3]), data, datasize);

    chksum = CalcChkSum();
    this->packet[3 + datasize] = chksum >> 8;
    this->packet[3 + datasize + 1] = chksum & 0xFF;

    if (!Check()) {
#ifdef P2OS_ERROR_PRINT
        this->debug_serial->printf("Error: DAMN\n");
#endif
        return 1;
    }
    return 0;
}

int P2OSPacket::Send() {
    int cnt = 0;

    while (cnt != this->size) {
        if (this->pioneer_serial->availableForWrite()) {
            int read_result = this->pioneer_serial->write(this->packet, this->size);
            cnt += read_result;
            if (cnt < 0) {
#ifdef P2OS_ERROR_PRINT
                this->debug_serial->println("Error: Send");
#endif
                return 1;
            }
        } else {
#ifdef P2OS_DEBUG_PRINT
            this->debug_serial->println("Not available to write");
#endif
        }
    }
    this->pioneer_serial->flush(true);

#ifdef P2OS_DEBUG_PRINT
    this->debug_serial->println("Sent:");
    this->Print();
#endif
    return 0;
}
