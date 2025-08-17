#include "hikcamera/include/HikCamera.h"
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


void cameraThreadFunc(HikCamera& cam);

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
    HikCamera cam;
    std::thread camThread(cameraThreadFunc, std::ref(cam));


    // 主循环
    while (running.load()) {
        // 主循环可以处理其他任务
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待所有线程结束
    camThread.join();


    std::cout << "exit." << std::endl;
    return 0;
}
