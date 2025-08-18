/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 17:13:01
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 15:39:28
 * @FilePath: /mas_vision/serial/include/serial_types.hpp
 * @Description: 
 */
#ifndef _SERIAL_TYPES_H_
#define _SERIAL_TYPES_H_

#include <cstdint>
#include <string>

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

enum VisionMode {
  AUTO_AIM_RED = 0,
  AUTO_AIM_BLUE = 1,
  SMALL_RUNE_RED = 2,
  SMALL_RUNE_BLUE = 3,
  BIG_RUNE_RED = 4,
  BIG_RUNE_BLUE = 5,
};

}

#endif // _SERIAL_TYPES_H_