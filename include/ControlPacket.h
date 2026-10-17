#pragma once

#include <Arduino.h>
#include <stddef.h>

struct __attribute__((packed)) ControlPacket {
    uint8_t start1 = 0xAA;
    uint8_t start2 = 0x55;

    uint16_t sequence = 0;  // 送信番号

    int16_t leftPower = 0;   // -255〜255
    int16_t rightPower = 0;  // -255〜255

    // front, rear, leftfront, rightfront, leftrear, rightrear
    uint16_t servoAngle[6] = {90, 90, 90, 90, 90, 90};

    uint16_t crc = 0;
};

static_assert(sizeof(ControlPacket) == 22,
              "ControlPacket size mismatch");

// CRC-16/CCITT-FALSE
inline uint16_t calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

inline void sendPacket(HardwareSerial& link,
                       ControlPacket& packet) {
    packet.crc = calculateCRC(
        reinterpret_cast<const uint8_t*>(&packet),
        offsetof(ControlPacket, crc)
    );

    link.write(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet)
    );
}