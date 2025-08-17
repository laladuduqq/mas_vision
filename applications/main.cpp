/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 16:17:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 19:46:24
 * @FilePath: /mas_vision/applications/main.cpp
 * @Description: 
 */
#include "HikCamera.h"
#include "serial.hpp"  
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <signal.h>

std::atomic<bool> running(true);
std::atomic<bool> camReady(false);
std::queue<cv::Mat> frameQueue;
std::mutex queueMutex;
std::condition_variable queueCond;

// 添加串口相关全局变量
std::queue<mas_serial::ReceivedDataMsg> receivedDataQueue;
std::mutex dataQueueMutex;
std::condition_variable dataQueueCond;

// 声明串口线程函数
void serialThreadFunc(mas_serial::Serial& serial);

void cameraThreadFunc(hikcamera::HikCamera& cam);

// 信号处理函数，用于退出
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
    running = false;
}

int main(int argc, char* argv[])
{
    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 初始化相机
    hikcamera::HikCamera cam;
    std::thread camThread(cameraThreadFunc, std::ref(cam));

    // 初始化串口
    mas_serial::Serial serial;
    if (!serial.init("config/serial_config.json")) {
        std::cerr << "Failed to initialize serial port" << std::endl;
        return -1;
    }
    std::thread serialThread(serialThreadFunc, std::ref(serial));

    // 主循环
    while (running.load()) {
        // 主循环可以处理其他任务
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待所有线程结束
    camThread.join();
    serialThread.join();

    std::cout << "exit." << std::endl;
    return 0;
}