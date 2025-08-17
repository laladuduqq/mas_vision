/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 17:03:17
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 17:05:39
 * @FilePath: /mas_vision/applications/test_calibration.cpp
 * @Description: 
 */
#include "HikCamera.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "=== Camera Calibration Test Program ===" << std::endl;

    // 初始化相机
    HikCamera camera;
    if (!camera.openCamera()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    // 尝试读取标定参数
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size imageSize;
    double reprojectionError = 0.0;

    try {
        cv::FileStorage fs("config/camera_calibration.json", cv::FileStorage::READ);
        if (fs.isOpened()) {
            int width, height;
            fs["image_width"] >> width;
            fs["image_height"] >> height;
            imageSize = cv::Size(width, height);

            fs["camera_matrix"] >> cameraMatrix;
            fs["distortion_coefficients"] >> distCoeffs;

            // 尝试读取重投影误差（如果存在）
            cv::FileNode n = fs["avg_reprojection_error"];
            if (!n.empty()) {
                reprojectionError = (double)n;
            }

            fs.release();

            std::cout << "Successfully loaded calibration parameters" << std::endl;
            std::cout << "Image size: " << imageSize.width << "x" << imageSize.height << std::endl;
            if (reprojectionError > 0) {
                std::cout << "Average reprojection error: " << reprojectionError << " pixels" << std::endl;
            }
        } else {
            std::cout << "Calibration file not found, distortion correction will not be applied" << std::endl;
        }
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to read calibration parameters: " << e.what() << std::endl;
    }

    cv::Mat frame, undistortedFrame;
    bool useUndistort = !cameraMatrix.empty() && !distCoeffs.empty();

    std::cout << std::endl << "Controls:" << std::endl;
    std::cout << "  'u'     - Toggle distortion correction" << std::endl;
    std::cout << "  'q'/'ESC'- Quit" << std::endl;
    std::cout << std::endl;

    while (true) {
        if (!camera.grabImage(frame)) {
            std::cerr << "Failed to grab image!" << std::endl;
            continue;
        }

        if (useUndistort && !cameraMatrix.empty() && !distCoeffs.empty()) {
            cv::undistort(frame, undistortedFrame, cameraMatrix, distCoeffs);
        } else {
            undistortedFrame = frame;
        }

        // 显示信息
        std::string info = useUndistort ? "Undistorted" : "Original";
        cv::putText(undistortedFrame, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(undistortedFrame, "Press 'u' to toggle correction, 'q' or ESC to exit",
                   cv::Point(10, undistortedFrame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Camera with Calibration", undistortedFrame);

        int key = cv::waitKey(30) & 0xFF;
        if (key == 27 || key == 'q') { // ESC or 'q' to quit
            break;
        } else if (key == 'u') { // 'u' to toggle undistortion
            useUndistort = !useUndistort;
            std::cout << "Distortion correction " << (useUndistort ? "enabled" : "disabled") << std::endl;
        }
    }

    camera.closeCamera();
    cv::destroyAllWindows();

    return 0;
}
