/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 00:14:24
 * @FilePath: /mas_vision/hikcamera/thread/cam_thread.cpp
 * @Description:
 */
#include "HikCamera.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <iostream>
#include <thread>
#include "pubsub.hpp"
#include "performance_monitor.hpp"


extern std::atomic<bool> running;
extern std::atomic<bool> camReady;
extern mas_utils::PerformanceMonitor perfMonitor;

static bool displayEnabled = false;

// 相机线程函数
void cameraThreadFunc(hikcamera::HikCamera& cam) {
    // 注册性能监控
    perfMonitor.addThread("CameraThread", perfMonitor.getThreadsId());
    
    cv::Mat frame;
    cv::Mat undistortedFrame;   

    try {
        cv::FileStorage fs("config/camera_set.json", cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["display"] >> displayEnabled;
            fs.release();
        }
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to load: " << e.what() << std::endl;
    };
    
    // 初始化相机
    if (!cam.openCamera()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        running = false;
        return;
    }
    
    camReady = true;
    std::cout << "Camera initialized successfully" << std::endl;


    // 创建图像发布者
    Publisher<cv::Mat> imagePublisher("camera/image");
    
    while (running.load()) {
        // 获取帧
        if (cam.grabImage(frame)) {
            // 发布图像消息到PubSub系统
            imagePublisher.publish(frame);
            
            if (displayEnabled)
            {
                // 显示图像
                cv::Mat resizedDrawingFrame;
                cv::resize(frame, resizedDrawingFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
                cv::imshow("Camera", resizedDrawingFrame);
                cv::waitKey(1);
            }
        } 
    }


    cam.closeCamera();
    if (displayEnabled)
    {
        // 关闭所有OpenCV窗口
        cv::destroyAllWindows();
    }
    
    std::cout << "Camera thread exit" << std::endl;
}

// 启动相机线程
void startCameraThread(hikcamera::HikCamera& cam) {
    static std::thread camera_thread(cameraThreadFunc, std::ref(cam));
    // 分离线程，让它独立运行
    camera_thread.detach();
}