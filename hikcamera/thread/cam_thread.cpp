/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 20:00:35
 * @FilePath: /mas_vision/hikcamera/thread/cam_thread.cpp
 * @Description:
 */
#include "HikCamera.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>

extern std::atomic<bool> running;
extern std::queue<cv::Mat> frameQueue;
extern std::mutex queueMutex;
extern std::condition_variable queueCond;
extern std::atomic<bool> camReady;

// 全局标定参数
static cv::Mat cameraMatrix;
static cv::Mat distCoeffs;
static cv::Mat map1, map2;  // 用于remap的映射表
static bool useCalibration = false;
static cv::Size imageSize;
static bool displayEnabled = false;


// 初始化标定参数 
bool initCalibration() {
    try {
        cv::FileStorage fs("config/camera_calibration.json", cv::FileStorage::READ);
        if (fs.isOpened()) {
            cv::Mat camMatrix, distCoeff;
            int width, height;

            fs["image_width"] >> width;
            fs["image_height"] >> height;
            fs["camera_matrix"] >> camMatrix;
            fs["distortion_coefficients"] >> distCoeff;

            if (!camMatrix.empty() && !distCoeff.empty()) {
                cameraMatrix = camMatrix.clone();
                distCoeffs = distCoeff.clone();
                imageSize = cv::Size(width, height);

                // 预计算remap映射表
                cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                          cameraMatrix, imageSize, CV_16SC2, map1, map2);

                fs.release();

                std::cout << "Calibration parameters loaded successfully" << std::endl;
                return true;
            }
            fs.release();
        }
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to load calibration parameters: " << e.what() << std::endl;
    }

    std::cout << "No calibration parameters found, using raw images" << std::endl;
    return false;
}

void cameraThreadFunc(hikcamera::HikCamera& cam) {
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
    
    // 初始化标定参数
    useCalibration = initCalibration();
    
    while (running.load()) {
        // 获取帧
        if (cam.grabImage(frame)) {
            // 如果启用了标定，则进行畸变校正
            if (useCalibration && !frame.empty()) {
                cv::remap(frame, undistortedFrame, map1, map2, cv::INTER_LINEAR);
                frame = undistortedFrame;
            }
            if (displayEnabled)
            {
                // 显示图像
                cv::Mat resizedDrawingFrame;
                cv::resize(frame, resizedDrawingFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
                cv::imshow("Camera", resizedDrawingFrame);
                cv::waitKey(1);
            }
            // 将帧放入队列供其他线程使用
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                frameQueue.push(frame);
            }
            queueCond.notify_one();
        } 
    }

    cam.closeCamera();
    if (displayEnabled)
    {
        // 关闭所有OpenCV窗口
        cv::destroyAllWindows();
    }
    
    std::cout << "Camera thread finished" << std::endl;
}