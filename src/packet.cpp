#include <Arduino.h>
#include <packet.hpp>
#include <ArduinoLog.h>

P2OSPacket::P2OSPacket(HardwareSerial& pioneer_serial) {
    this->pioneer_serial = &pioneer_serial;
}

P2OSPacket::~P2OSPacket() { /** Destructor **/ }

void P2OSPacket::set_pioneer_serial(HardwareSerial& pioneer_serial) {
    this->pioneer_serial = &pioneer_serial;
}

void P2OSPacket::Print() {
    if (Log.getLevel() == LOG_LEVEL_VERBOSE) {
        Log.setShowLevel(false);
        if (this->packet) {
            for (int i = 0; i < this->size; i++) {
                Log.verbose("%u ", packet[i]);
            }
            Log.verbose("\n");
        }
        Log.setShowLevel(true);
    }
}

void P2OSPacket::PrintHex() {
    if (Log.getLevel() == LOG_LEVEL_VERBOSE) {
        Log.setShowLevel(false);
        if (this->packet) {
            for (int i = 0; i < this->size; i++) {
                Log.verbose("%X ", packet[i]);
            }
            Log.verbose("\n");
        }
        Log.setShowLevel(true);
    }
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
    unsigned char new_prefix[3];
    int           cnt;

    ::memset(this->packet, 0, sizeof(this->packet));

    do {
        ::memset(new_prefix, 0, sizeof(new_prefix));

        int retries_0 = 100;
        while (1) {
            cnt = 0;
            int retries_1 = 100;
            while (cnt != 1 && retries_1 > 0) {
                if (this->pioneer_serial->available()) {
                    cnt += this->pioneer_serial->read(&new_prefix[2], 1);;
                    if (cnt < 0) {
                        Log.errorln(
                            "error reading packet.header from robot connection: P2OSPacket():Receive():read():"
                        );
                        return 1;
                    }
                } else {
                    retries_1--;
                    if (retries_1 < 0) {
                        Log.errorln(
                            "P2OSPacket():Receive():read(): timout reading packet.header from robot connection: "
                        );
                        return 1;
                    }
                }
            }

            if (new_prefix[0] == 0xFA && new_prefix[1] == 0xFB) {
                break;
            }

            new_prefix[0] = new_prefix[1];
            new_prefix[1] = new_prefix[2];
            // skipped++;

            retries_0--;
            if (retries_0 < 0) {
                Log.errorln("timout retried: P2OSPacket():Receive():read():");
                return 1;
            }
        }
        // // if (skipped>3) ROS_INFO("Skipped %d bytes\n", skipped);

        this->size = new_prefix[2] + 3;
        memcpy(this->packet, new_prefix, 3);

        cnt = 0;
        while (cnt != new_prefix[2]) {
            if (this->pioneer_serial->available()) {
                cnt += this->pioneer_serial->read(&packet[3 + cnt], new_prefix[2] - cnt);

                if (cnt < 0) {
                    Log.errorln("Error reading packet body from robot connection: P2OSPacket():Receive():read():");
                    return 1;
                }
            } else {
                Log.verboseln("reading packet.body from robot connection: P2OSPacket():Receive():read():");
            }
        }
        // Log.verboseln("Received:");
        // this->PrintHex();
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
        Log.errorln("Packet to P2OS can't be larger than 200 bytes");
        return 1;
    }
    this->packet[2] = datasize + 2;

    memcpy(&(this->packet[3]), data, datasize);

    chksum = CalcChkSum();
    this->packet[3 + datasize] = chksum >> 8;
    this->packet[3 + datasize + 1] = chksum & 0xFF;

    if (!Check()) {
        Log.errorln("DAMN! packet built problem\n");
        return 1;
    }
    return 0;
}

int P2OSPacket::Send() {
    int cnt = 0;

    while (cnt != this->size) {
        if (this->pioneer_serial->availableForWrite()) {
            cnt += this->pioneer_serial->write(this->packet, this->size);
            if (cnt < 0) {
                Log.errorln("Send");
                return 1;
            }
        } else {
            Log.verboseln("Not available to write");
        }
    }
    this->pioneer_serial->flush(true);

    // Log.verboseln("Sent:");
    // this->Print();
    return 0;
}
