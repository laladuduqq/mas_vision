/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 17:13:01
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 18:43:49
 * @FilePath: /mas_vision/serial/include/serial_types.hpp
 * @Description: 
 */
#ifndef _SERIAL_TYPES_H_
#define _SERIAL_TYPES_H_

#include <cstdint>

namespace mas_serial {
#pragma pack(push, 1)
    struct ReceivePacket {
        uint8_t header=0xAA;
        uint8_t mode;
        float yaw;
        float pitch;
        float roll;
        uint8_t tail=0X5A;
    };

    struct SendPacket {
        uint8_t header=0XBB;
        float yaw;
        float pitch;
        float distance;
        uint8_t fire_advice;
        uint8_t check;
        uint8_t tail=0X5B;
    };
#pragma pack(pop)
}

#endif // _SERIAL_TYPES_H_