#include "serial.hpp"
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>

mas_serial::Serial::Serial() : virtual_serial_(false), fd_(-1) {
}

mas_serial::Serial::~Serial() {
    close();
}

bool mas_serial::Serial::init(const std::string& config_path) {
    try {
        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return false;
        }

        fs["port"] >> port_;
        fs["baudrate"] >> baudrate_;
        fs["flow_control"] >> flow_control_;
        fs["parity"] >> parity_;
        fs["stop_bits"] >> stop_bits_;
        fs["virtual_serial"] >> virtual_serial_;

        if (virtual_serial_) {
            // 虚拟串口模式 - 读取虚拟数据配置
            cv::FileNode virtual_data_node = fs["virtual_data"];
            virtual_data_.yaw = virtual_data_node["yaw"];
            virtual_data_.pitch = virtual_data_node["pitch"];
            virtual_data_.roll = virtual_data_node["roll"];
            virtual_data_.mode = (int)virtual_data_node["mode"];
            virtual_data_.timestamp = std::chrono::steady_clock::now();
            fs.release();
            std::cout << "Virtual serial mode enabled" << std::endl;
            return true;
        }

        fs.release();

        // 实际串口模式 - 打开串口设备
        std::cout << "Opening serial port: " << port_ << std::endl;
        // 初始化时只尝试一次，失败则返回false
        return openPort();
    } catch (const std::exception& ex) {
        std::cerr << "Exception during serial initialization: " << ex.what() << std::endl;
        return false;
    }
}

void mas_serial::Serial::close() {
    try {
        if (!virtual_serial_ && fd_ != -1) {
            closePort();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Exception during serial close: " << ex.what() << std::endl;
    }
}

bool mas_serial::Serial::readByte(uint8_t& byte) {
    if (fd_ == -1) return false;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    
    struct timeval timeout;
    timeout.tv_sec = 1;   // 1秒超时
    timeout.tv_usec = 0;

    int result = select(fd_ + 1, &readfds, NULL, NULL, &timeout);
    if (result > 0 && FD_ISSET(fd_, &readfds)) {
        ssize_t bytes_read = ::read(fd_, &byte, 1);
        return (bytes_read == 1);
    }
    
    return false;
}

bool mas_serial::Serial::fillBuffer() {
    uint8_t byte;
    if (readByte(byte)) {
        input_buffer_.push(byte);
        return true;
    }
    return false;
}

bool mas_serial::Serial::readFromBuffer(uint8_t* data, size_t size) {
    // 确保缓冲区中有足够的数据
    while (input_buffer_.size() < size) {
        if (!fillBuffer()) {
            return false;
        }
    }
    
    // 从缓冲区读取数据
    for (size_t i = 0; i < size; i++) {
        data[i] = input_buffer_.front();
        input_buffer_.pop();
    }
    
    return true;
}

bool mas_serial::Serial::readData(ReceivedDataMsg& data) {
    try {
        if (virtual_serial_) {
            // 虚拟串口模式 - 返回预设数据
            virtual_data_.timestamp = std::chrono::steady_clock::now();
            data = virtual_data_;
            usleep(10000); // 10ms delay to simulate real serial communication
            return true;
        }

        // 实际串口模式 - 从串口读取数据
        std::vector<uint8_t> buffer(sizeof(ReceivePacket));

        // 使用缓冲区读取优化的方式
        if (readFromBuffer(buffer.data(), sizeof(ReceivePacket))) {
            // 检查帧头和帧尾
            const uint8_t expected_header = ReceivePacket().header;
            const uint8_t expected_tail = ReceivePacket().tail;

            if (buffer[0] != expected_header || buffer[sizeof(ReceivePacket) - 1] != expected_tail) {
                std::cerr << "Invalid header or tail: " << std::hex << static_cast<int>(buffer[0])
                        << " " << static_cast<int>(buffer[sizeof(ReceivePacket) - 1]) << std::dec << std::endl;
                return false;
            }

            // 解析数据包
            ReceivePacket packet;
            std::memcpy(&packet, buffer.data(), sizeof(ReceivePacket));

            // 解析数据
            data.yaw = static_cast<double>(packet.yaw);
            data.pitch = static_cast<double>(packet.pitch);
            data.roll = static_cast<double>(packet.roll);
            data.mode = static_cast<int>(packet.mode);
            data.timestamp = std::chrono::steady_clock::now();

            return true;
        }
        return false;
    } catch (const std::exception& ex) {
        std::cerr << "Exception during readData: " << ex.what() << std::endl;
        // 尝试重新打开端口
        if (!virtual_serial_) {
            reopenPort();
        }
        return false;
    }
}

bool mas_serial::Serial::sendData(const SendPacket& data) {
    try {
        if (virtual_serial_) {
            // 虚拟串口模式 - 不执行任何操作
            return true;
        }

        // 实际串口模式 - 发送数据到串口
        std::vector<uint8_t> data_vector(reinterpret_cast<const uint8_t*>(&data),
                                         reinterpret_cast<const uint8_t*>(&data) + sizeof(SendPacket));

        return writeToPort(data_vector);
    } catch (const std::exception& ex) {
        std::cerr << "Exception during sendData: " << ex.what() << std::endl;
        // 尝试重新打开端口
        if (!virtual_serial_) {
            reopenPort();
        }
        return false;
    }
}

// 参考libserial的串口配置方式
bool mas_serial::Serial::openPort() {
    try {
        // 使用阻塞模式打开串口
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            std::cerr << "Failed to open serial port: " << port_ << ", error: " << strerror(errno) << std::endl;
            return false;
        }

        // 保存当前的tty配置
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Error getting tty attributes: " << strerror(errno) << std::endl;
            closePort();
            return false;
        }

        // 初始化tty配置为0
        memset(&tty, 0, sizeof(tty));

        // 设置波特率
        speed_t baudrate_speed;
        switch (baudrate_) {
            case 9600:   baudrate_speed = B9600;   break;
            case 19200:  baudrate_speed = B19200;  break;
            case 38400:  baudrate_speed = B38400;  break;
            case 57600:  baudrate_speed = B57600;  break;
            case 115200: baudrate_speed = B115200; break;
            default:
                std::cerr << "Unsupported baudrate: " << baudrate_ << std::endl;
                closePort();
                return false;
        }
        
        // 设置输入和输出波特率
        cfsetispeed(&tty, baudrate_speed);
        cfsetospeed(&tty, baudrate_speed);

        // 设置为原始模式 (类似libserial的实现)
        // 清除所有标志位
        tty.c_cflag &= ~PARENB;  // 无奇偶校验
        tty.c_cflag &= ~CSTOPB;  // 1个停止位
        tty.c_cflag &= ~CSIZE;   // 清除数据位设置
        tty.c_cflag |= CS8;      // 8个数据位
        tty.c_cflag &= ~CRTSCTS; // 无流控制
        tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略控制行

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 原始输入

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
        tty.c_oflag &= ~OPOST; // 原始输出

        // 设置超时和最小字符数
        tty.c_cc[VMIN]  = 0; // 最小字符数
        tty.c_cc[VTIME] = 5; // 超时时间 (0.5秒)

        // 清空输入输出缓冲区
        tcflush(fd_, TCIOFLUSH);

        // 应用配置
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting tty attributes: " << strerror(errno) << std::endl;
            closePort();
            return false;
        }

        // 清空输入缓冲区
        while (!input_buffer_.empty()) input_buffer_.pop();

        return true;
    } catch (const std::exception& ex) {
        std::cerr << "Exception during openPort: " << ex.what() << std::endl;
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
        return false;
    }
}

void mas_serial::Serial::closePort() {
    try {
        if (fd_ != -1) {
            // 清空缓冲区
            tcflush(fd_, TCIOFLUSH);
            
            // 关闭文件描述符
            ::close(fd_);
            fd_ = -1;
            
            // 清空输入缓冲区
            while (!input_buffer_.empty()) input_buffer_.pop();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Exception during closePort: " << ex.what() << std::endl;
        fd_ = -1; // 强制设置为-1
    }
}

bool mas_serial::Serial::readFromPort(std::vector<uint8_t>& buffer, size_t size) {
    try {
        if (fd_ == -1) return false;

        buffer.resize(size);
        size_t bytes_read = 0;
        while (bytes_read < size) {
            ssize_t result = ::read(fd_, buffer.data() + bytes_read, size - bytes_read);
            if (result < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // 非阻塞模式下的超时，继续尝试
                    continue;
                }
                std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
                return false;
            } else if (result == 0) {
                // EOF 或连接断开
                std::cerr << "Serial port connection lost" << std::endl;
                return false;
            }
            bytes_read += result;
        }

        return true;
    } catch (const std::exception& ex) {
        std::cerr << "Exception during readFromPort: " << ex.what() << std::endl;
        return false;
    }
}

bool mas_serial::Serial::writeToPort(const std::vector<uint8_t>& data) {
    try {
        if (fd_ == -1) return false;

        size_t bytes_written = 0;
        size_t total_bytes = data.size();
        
        // 使用select确保写入不会阻塞
        while (bytes_written < total_bytes) {
            fd_set writefds;
            FD_ZERO(&writefds);
            FD_SET(fd_, &writefds);
            
            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            int result = select(fd_ + 1, NULL, &writefds, NULL, &timeout);
            if (result > 0 && FD_ISSET(fd_, &writefds)) {
                ssize_t written = ::write(fd_, data.data() + bytes_written, total_bytes - bytes_written);
                if (written < 0) {
                    std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
                    return false;
                }
                bytes_written += written;
            } else {
                std::cerr << "Write timeout or error" << std::endl;
                return false;
            }
        }

        // 确保数据被发送
        tcdrain(fd_);
        return true;
    } catch (const std::exception& ex) {
        std::cerr << "Exception during writeToPort: " << ex.what() << std::endl;
        return false;
    }
}

void mas_serial::Serial::reopenPort() {
    try {
        std::cerr << "Attempting to reopen port: " << port_ << std::endl;
        // 先关闭端口
        if (fd_ != -1) {
            closePort();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 指数退避
        if (openPort()) {return;}
    
        std::cerr << "Failed to reopen port, attempt "  << ", retrying..." << std::endl;
        
    } catch (const std::exception& ex) {
        std::cerr << "Exception during reopenPort: " << ex.what() << std::endl;
        
    }
}