#include "HikCamera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <time.h>
#include <thread>
#include <future>
#include <algorithm>

// 计算重投影误差
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    std::vector<cv::Point2f> imagePoints2;
    int totalPoints = 0;
    double totalErr = 0;

    // 使用多线程并行计算每幅图像的重投影误差
    std::vector<std::future<std::pair<double, int>>> futures;

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        futures.push_back(std::async(std::launch::async, [&, i]() {
            std::vector<cv::Point2f> imgPoints;
            cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgPoints);
            double err = cv::norm(imagePoints[i], imgPoints, cv::NORM_L2);
            return std::make_pair(err * err, (int)objectPoints[i].size());
        }));
    }

    // 收集所有线程的结果
    for (auto& future : futures) {
        auto result = future.get();
        totalErr += result.first;
        totalPoints += result.second;
    }

    return std::sqrt(totalErr/totalPoints);
}

// 运行相机标定
bool runCalibration(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                   const std::vector<std::vector<cv::Point2f>>& imagePoints,
                   const cv::Size& imageSize,
                   cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::cout << "Starting camera calibration with " << objectPoints.size() << " images..." << std::endl;

    // 获取系统线程数并设置OpenCV使用多线程
    int numThreads = std::thread::hardware_concurrency();
    if (numThreads > 0) {
        cv::setNumThreads(numThreads);
        std::cout << "Using " << numThreads << " threads for computation" << std::endl;
    }

    // 运行标定
    auto start_time = std::chrono::high_resolution_clock::now();
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Calibration completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

    // 计算重投影误差
    std::cout << "Computing reprojection errors..." << std::endl;
    start_time = std::chrono::high_resolution_clock::now();
    double reprojErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs);
    end_time = std::chrono::high_resolution_clock::now();

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Reprojection error computation completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Overall RMS re-projection error: " << reprojErr << std::endl;

    return rms < 2.0; // 如果重投影误差小于2像素，认为标定成功
}

// 检查角点分布质量
double evaluateCornersQuality(const std::vector<cv::Point2f>& corners, const cv::Size& imageSize) {
    if (corners.empty()) return 0.0;

    // 计算角点的边界框
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (const auto& corner : corners) {
        minX = std::min(minX, corner.x);
        maxX = std::max(maxX, corner.x);
        minY = std::min(minY, corner.y);
        maxY = std::max(maxY, corner.y);
    }

    // 计算覆盖面积比例（相对于整个图像）
    float coverageX = (maxX - minX) / imageSize.width;
    float coverageY = (maxY - minY) / imageSize.height;
    float coverageArea = coverageX * coverageY;

    // 检查角点是否太靠近边缘（避免边缘畸变影响）
    int margin = 30;
    int goodPoints = 0;
    for (const auto& corner : corners) {
        if (corner.x > margin && corner.x < imageSize.width - margin &&
            corner.y > margin && corner.y < imageSize.height - margin) {
            goodPoints++;
        }
    }

    float edgeRatio = (float)goodPoints / corners.size();

    // 综合质量评分 (0-1之间)
    // 覆盖面积权重70%，边缘点比例权重30%
    return (coverageArea * 0.7 + edgeRatio * 0.3);
}

// 检查新图像是否提供了新的视角信息
bool isViewpointDiverse(const std::vector<std::vector<cv::Point2f>>& existingCorners,
                       const std::vector<cv::Point2f>& newCorners) {
    if (existingCorners.empty()) return true;

    // 计算新角点的中心点
    cv::Point2f newCenter(0, 0);
    for (const auto& pt : newCorners) {
        newCenter += pt;
    }
    newCenter *= (1.0 / newCorners.size());

    // 检查与已捕获图像的中心点距离
    for (const auto& corners : existingCorners) {
        cv::Point2f center(0, 0);
        for (const auto& pt : corners) {
            center += pt;
        }
        center *= (1.0 / corners.size());

        // 计算中心点距离（以像素为单位）
        double distance = cv::norm(newCenter - center);

        // 如果与任何已捕获图像的中心点足够远，则认为视角是新颖的
        // 距离阈值可以根据图像分辨率调整
        if (distance < 100) {  // 阈值可以调整
            return false;
        }
    }

    return true;
}

int main() {
    std::cout << "=== Camera Calibration Program ===" << std::endl;
    std::cout << "This program will calibrate your camera using a chessboard pattern." << std::endl;
    std::cout << "Please show the chessboard pattern to the camera from various angles and distances." << std::endl;
    std::cout << std::endl;

    // 初始化相机
    HikCamera camera;
    if (!camera.openCamera()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    // 读取标定配置
    int boardWidth = 9;
    int boardHeight = 6;
    float squareSize = 1.0f;  // 棋盘格方格的实际大小（以您选择的单位为准，如厘米）
    std::string outputFile = "config/camera_calibration.json";
    int minImages = 10;   // 最少需要的图像数量
    int maxImages = 25;   // 最多图像数量

    try {
        cv::FileStorage fs("config/camera_set.json", cv::FileStorage::READ);
        if (fs.isOpened()) {
            cv::FileNode calibNode = fs["calibration"];
            if (!calibNode.empty()) {
                boardWidth = (int)calibNode["board_width"];
                boardHeight = (int)calibNode["board_height"];
                squareSize = (float)calibNode["square_size"];
                outputFile = (std::string)calibNode["output_file"];
                minImages = (int)calibNode["min_images"];
                maxImages = (int)calibNode["max_images"];
            }
            fs.release();
        }
    } catch (const cv::Exception& e) {
        std::cerr << "Warning: Could not read calibration config, using defaults. Error: " << e.what() << std::endl;
    }

    std::cout << "Chessboard configuration:" << std::endl;
    std::cout << "  Corners: " << boardWidth << " x " << boardHeight << std::endl;
    std::cout << "  Square size: " << squareSize << " cm (assumed)" << std::endl;
    std::cout << std::endl;
    std::cout << "Capture settings:" << std::endl;
    std::cout << "  Minimum images: " << minImages << std::endl;
    std::cout << "  Maximum images: " << maxImages << std::endl;
    std::cout << std::endl;

    // 准备标定板的3D点（在世界坐标系中）
    std::vector<cv::Point3f> objectCorners;
    for (int i = 0; i < boardHeight; i++) {
        for (int j = 0; j < boardWidth; j++) {
            objectCorners.push_back(cv::Point3f(float(j * squareSize),
                                              float(i * squareSize),
                                              0)); // 所有点的Z坐标为0（标定板在一个平面上）
        }
    }

    // 存储检测到的角点
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    cv::Mat frame;
    cv::Size boardSize(boardWidth, boardHeight);
    bool autoCapture = false;
    time_t lastCaptureTime = 0;
    const int captureInterval = 1500; // 捕获间隔（毫秒）
    int detectionFailures = 0; // 检测失败计数器
    const int maxDetectionFailures = 5; // 最大连续检测失败次数

    std::cout << "Controls:" << std::endl;
    std::cout << "  SPACE - Capture image manually" << std::endl;
    std::cout << "  'a'     - Toggle auto-capture mode" << std::endl;
    std::cout << "  'c'     - Run calibration (when enough images are captured)" << std::endl;
    std::cout << "  'q'/'ESC'- Quit" << std::endl;
    std::cout << std::endl;

    while (true) {
        // 设置超时以避免卡死
        if (!camera.grabImage(frame)) {
            std::cerr << "Failed to grab image!" << std::endl;
            // 等待一小段时间避免忙等待
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        // 查找棋盘角点 - 使用多种检测方法提高鲁棒性
        std::vector<cv::Point2f> corners;
        bool found = false;

        // 尝试不同的检测标志组合
        std::vector<int> flags = {
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS,
            0  // 不使用任何特殊标志
        };

        // 并行尝试不同的检测标志
        std::vector<std::future<std::pair<bool, std::vector<cv::Point2f>>>> detection_futures;
        for (int flag : flags) {
            detection_futures.push_back(std::async(std::launch::async, [&frame, &boardSize, flag]() {
                std::vector<cv::Point2f> detected_corners;
                bool result = cv::findChessboardCorners(frame, boardSize, detected_corners, flag);
                return std::make_pair(result, detected_corners);
            }));
        }

        // 检查结果
        for (auto& future : detection_futures) {
            auto result = future.get();
            if (result.first) {
                found = true;
                corners = result.second;
                break;
            }
        }

        // 如果仍未找到，尝试图像预处理
        if (!found) {
            cv::Mat gray, processed;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // 尝试不同的预处理方法
            std::vector<cv::Mat> processed_images = {
                gray,
                gray.clone()
            };

            // 对第二张图像进行直方图均衡化
            cv::equalizeHist(gray, processed_images[1]);

            // 并行尝试在预处理后的图像上检测角点
            std::vector<std::future<std::pair<bool, std::vector<cv::Point2f>>>> preprocess_futures;
            for (const auto& img : processed_images) {
                for (int flag : flags) {
                    preprocess_futures.push_back(std::async(std::launch::async, [&img, &boardSize, flag]() {
                        std::vector<cv::Point2f> detected_corners;
                        bool result = cv::findChessboardCorners(img, boardSize, detected_corners, flag);
                        return std::make_pair(result, detected_corners);
                    }));
                }
            }

            // 检查预处理结果
            for (auto& future : preprocess_futures) {
                auto result = future.get();
                if (result.first) {
                    found = true;
                    corners = result.second;
                    break;
                }
            }
        }

        // 绘制角点
        cv::Mat displayFrame = frame.clone();

        if (found) {
            // 重置检测失败计数器
            detectionFailures = 0;

            // 细化角点位置
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            cv::drawChessboardCorners(displayFrame, boardSize, cv::Mat(corners), found);

            // 评估角点质量
            double quality = evaluateCornersQuality(corners, frame.size());

            // 自动捕获高质量图像
            if (autoCapture && quality > 0.3 && // 质量阈值
                (time(nullptr) - lastCaptureTime) * 1000 > captureInterval &&
                imagePoints.size() < (size_t)maxImages) {

                // 检查视角是否新颖
                if (isViewpointDiverse(imagePoints, corners)) {
                    imagePoints.push_back(corners);
                    objectPoints.push_back(objectCorners);
                    lastCaptureTime = time(nullptr);
                    std::cout << "Auto-captured image " << imagePoints.size() << "/" << maxImages
                             << " (quality: " << quality << ", viewpoint: novel)" << std::endl;
                } else {
                    // 视角重复，不捕获
                    std::cout << "Skipped capture (viewpoint not novel, quality: " << quality << ")" << std::endl;
                }
            }

            // 显示质量信息
            std::string qualityStr = "Quality: " + std::to_string(quality).substr(0, 4);
            cv::putText(displayFrame, qualityStr, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);

        } else {
            // 增加检测失败计数器
            detectionFailures++;

            // 如果连续检测失败次数过多，重置计数器避免累积
            if (detectionFailures > maxDetectionFailures) {
                detectionFailures = 0;
            }
        }

        // 显示信息
        std::string info = "Captured: " + std::to_string(imagePoints.size()) +
                          " (min: " + std::to_string(minImages) + ")";
        if (autoCapture) {
            info += " [AUTO]";
        }
        cv::putText(displayFrame, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        std::string status = found ? "Chessboard detected" : "Chessboard not detected";
        cv::putText(displayFrame, status, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                   found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        // 添加提示信息
        if (!found) {
            cv::putText(displayFrame, "Tips: Try different angles and distances",
                       cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
        }

        cv::putText(displayFrame, "SPACE:Capture  'a':Auto  'c':Calibrate  'q':Quit",
                   cv::Point(10, displayFrame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Camera Calibration", displayFrame);

        // 使用超时等待按键，避免卡死
        int key = cv::waitKey(50) & 0xFF; // 50ms超时
        if (key == 27 || key == 'q') { // ESC or 'q' to quit
            break;
        } else if (key == ' ') { // Space to manually capture
            if (found) {
                if (imagePoints.size() < (size_t)maxImages) {
                    // 检查视角是否新颖
                    if (isViewpointDiverse(imagePoints, corners)) {
                        imagePoints.push_back(corners);
                        objectPoints.push_back(objectCorners);
                        std::cout << "Manually captured image " << imagePoints.size() << "/" << maxImages << std::endl;
                    } else {
                        std::cout << "Skipped capture (viewpoint not novel)" << std::endl;
                    }
                } else {
                    std::cout << "Maximum number of images reached!" << std::endl;
                }
            } else {
                std::cout << "Chessboard not detected, cannot capture image" << std::endl;
            }
        } else if (key == 'a') { // 'a' to toggle auto capture
            autoCapture = !autoCapture;
            std::cout << "Auto-capture " << (autoCapture ? "enabled" : "disabled") << std::endl;
            if (autoCapture) {
                std::cout << "Auto-capture will collect images with good quality automatically" << std::endl;
            }
        } else if (key == 'c') { // 'c' to start calibration
            if (imagePoints.size() >= (size_t)minImages) {
                std::cout << std::endl << "Starting calibration with " << imagePoints.size() << " images..." << std::endl;
                break;
            } else {
                std::cout << "Not enough images captured. Minimum required: " << minImages << std::endl;
                std::cout << "Current images: " << imagePoints.size() << std::endl;
            }
        }
    }

    // 执行标定
    if (imagePoints.size() >= (size_t)minImages) {
        std::cout << "Performing camera calibration..." << std::endl;
        cv::Mat cameraMatrix, distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        cv::Size imageSize(frame.cols, frame.rows);

        bool calibrationSuccess = runCalibration(objectPoints, imagePoints, imageSize,
                                               cameraMatrix, distCoeffs, rvecs, tvecs);

        if (calibrationSuccess) {
            std::cout << std::endl << "Calibration successful!" << std::endl;
            std::cout << "Camera matrix:" << std::endl << cameraMatrix << std::endl;
            std::cout << "Distortion coefficients:" << std::endl << distCoeffs.t() << std::endl;

            // 保存标定结果
            try {
                cv::FileStorage fs(outputFile, cv::FileStorage::WRITE);
                if (fs.isOpened()) {
                    fs << "image_width" << imageSize.width;
                    fs << "image_height" << imageSize.height;
                    fs << "camera_matrix" << cameraMatrix;
                    fs << "distortion_coefficients" << distCoeffs;
                    fs << "avg_reprojection_error" << computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs);
                    fs << "num_calibration_images" << (int)imagePoints.size();
                    fs.release();
                    std::cout << "Calibration results saved to " << outputFile << std::endl;
                } else {
                    std::cerr << "Failed to open " << outputFile << " for writing" << std::endl;
                }
            } catch (const cv::Exception& e) {
                std::cerr << "Failed to save calibration results: " << e.what() << std::endl;
            }
        } else {
            std::cerr << "Calibration failed! Reprojection error is too high." << std::endl;
        }
    } else {
        std::cout << "Not enough images captured for calibration." << std::endl;
        std::cout << "Captured " << imagePoints.size() << " images, need at least " << minImages << std::endl;
    }

    camera.closeCamera();
    cv::destroyAllWindows();

    return 0;
}
