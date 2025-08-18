/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 16:17:20
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 09:44:19
 * @FilePath: /mas_vision/applications/main.cpp
 * @Description: 
 */
#include "HikCamera.h"
#include "performance_monitor.hpp"
#include "serial.hpp"  
#include "pubsub.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>

std::atomic<bool> running(true);
std::atomic<bool> camReady(false);
std::atomic<bool> serialReady(false);

// 声明线程启动函数
void startCameraThread(hikcamera::HikCamera& cam);
void startSerialThread(mas_serial::Serial& serial);
void startPubSubThread();
void stopPubSubThread();
void startarmordetectorThread();

// 性能监控器实例
mas_utils::PerformanceMonitor perfMonitor;

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
    
    // 初始化串口
    mas_serial::Serial serial;
    if (!serial.init("config/serial_config.json")) {
        std::cerr << "Failed to initialize serial port" << std::endl;
        return -1;
    }

    // 配置性能监控器
    perfMonitor.addThread("Main Thread", perfMonitor.getThreadsId());
    
    // 启动性能监控
    perfMonitor.startMonitoring();
    
    // 启动PubSub消息中心线程
    startPubSubThread();

    // 启动相机线程
    startCameraThread(cam);

    // 启动串口线程
    startSerialThread(serial);

    // 启动装甲板检测线程
    startarmordetectorThread();

    // 主循环
    while (running.load()) {
        // 显示性能监控窗口
        perfMonitor.showPerformanceWindow();
        
        // 主循环可以处理其他任务
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // 停止PubSub消息中心
    stopPubSubThread();

    // 停止性能监控
    perfMonitor.stopMonitoring();

    std::cout << "exit." << std::endl;
    return 0;
}