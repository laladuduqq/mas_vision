#ifndef BASE_SERIAL_H
#define BASE_SERIAL_H

#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>
#include <unistd.h>
#include <cstdint>
#include <vector>
#include <memory>
#include <queue>

#include "serial_types.hpp"


namespace mas_serial {

    struct ReceivedDataMsg {
        double yaw;
        double pitch;
        double roll;
        uint8_t mode;
        std::chrono::steady_clock::time_point timestamp;
    };

    class Serial {
        public:
            Serial();
            ~Serial();

            bool init(const std::string& config_path);
            void close();

            bool readData(ReceivedDataMsg& data);
            bool sendData(const SendPacket& data);

            bool isVirtual() const { return virtual_serial_; }
            bool isOpen() const { return fd_ != -1; }

        private:
            std::string port_;
            int baudrate_;
            std::string flow_control_;
            std::string parity_;
            std::string stop_bits_;
            bool virtual_serial_;

            // 虚拟串口使用的默认值
            ReceivedDataMsg virtual_data_;

            // 实际串口文件描述符
            int fd_;
            
            // 输入缓冲区
            std::queue<uint8_t> input_buffer_;

            bool openPort();
            void closePort();
            bool readFromPort(std::vector<uint8_t>& buffer, size_t size);
            bool writeToPort(const std::vector<uint8_t>& data);
            
            // 按字节读取
            bool readByte(uint8_t& byte);
            // 从缓冲区读取
            bool readFromBuffer(uint8_t* data, size_t size);
            // 填充缓冲区
            bool fillBuffer();

            // 断线重连机制
            void reopenPort();
    };

}



#endif //BASE_SERIAL_H