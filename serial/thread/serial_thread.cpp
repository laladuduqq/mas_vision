/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 17:10:54
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 19:37:23
 * @FilePath: /mas_vision/serial/thread/serial_thread.cpp
 * @Description: 
 */
#include "serial.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

// 全局变量，用于在主线程和串口线程之间共享数据
extern std::atomic<bool> running;
extern std::queue<mas_serial::ReceivedDataMsg> receivedDataQueue;
extern std::mutex dataQueueMutex;
extern std::condition_variable dataQueueCond;

void serialThreadFunc(mas_serial::Serial& serial) {
    while (running.load()) {
        mas_serial::ReceivedDataMsg data;
        if (serial.readData(data)) {
            // 加锁并更新队列，保持队列长度为1
            {
                std::lock_guard<std::mutex> lock(dataQueueMutex);
                if (!receivedDataQueue.empty()) {
                    receivedDataQueue.pop(); // 移除旧数据
                }
                receivedDataQueue.push(data); // 添加新数据
            }
            dataQueueCond.notify_one();
        }
    }
}