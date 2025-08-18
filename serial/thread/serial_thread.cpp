/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 17:10:54
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 09:51:36
 * @FilePath: /mas_vision/serial/thread/serial_thread.cpp
 * @Description: 
 */
#include "serial.hpp"
#include <atomic>
#include <thread>
#include "pubsub.hpp"
#include "performance_monitor.hpp"

// 全局变量，用于在主线程和串口线程之间共享数据
extern std::atomic<bool> running;
extern mas_utils::PerformanceMonitor perfMonitor;
extern std::atomic<bool> serialReady;

void serialThreadFunc(mas_serial::Serial& serial) {
    // 注册性能监控
    perfMonitor.addThread("SerialThread", perfMonitor.getThreadsId());
    // 创建发布者
    Publisher<mas_serial::ReceivedDataMsg> serialPublisher("serial/data");
    while (running.load()) {
        mas_serial::ReceivedDataMsg data;
        // 串口状态
        if (serial.isOpen() || serial.isVirtual()){
            serialReady = true;
            if (serial.readData(data)) {
                // 发布消息到PubSub系统
                serialPublisher.publish(data);
            }
        }else{
            serialReady = false;
        }
    }
}

// 启动串口线程
void startSerialThread(mas_serial::Serial& serial) {
    static std::thread serial_thread(serialThreadFunc, std::ref(serial));
    // 分离线程，让它独立运行
    serial_thread.detach();
}