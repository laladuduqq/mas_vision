/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-07-28 18:10:53
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-02 16:56:54
 * @FilePath: /learn1/thread/cam_thread.cpp
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
                std::cout << "Camera matrix: " << std::endl << cameraMatrix << std::endl;
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

void cameraThreadFunc(HikCamera& cam) {
    cv::Mat frame;
    cv::Mat undistortedFrame;

    // 初始化相机
    if (!cam.openCamera()) {
        printf("打开摄像头失败！\n");
        running = false;
        return;
    }

    // 初始化标定参数
    useCalibration = initCalibration();

    camReady = true;
    while (running) {
        if (cam.grabImage(frame)) {
            // 如果有标定参数，则进行去畸变处理
            if (useCalibration && !map1.empty() && !map2.empty()) {
                // 使用预计算的映射表进行快速去畸变
                cv::remap(frame, undistortedFrame, map1, map2, cv::INTER_LINEAR);
            } else {
                undistortedFrame = frame;
            }

            std::lock_guard<std::mutex> lock(queueMutex);
            // 只保留最新一帧
            while (!frameQueue.empty()) frameQueue.pop();
            frameQueue.push(undistortedFrame); // 使用处理后的图像
            queueCond.notify_one();
        }
    }
    cam.closeCamera();
}
