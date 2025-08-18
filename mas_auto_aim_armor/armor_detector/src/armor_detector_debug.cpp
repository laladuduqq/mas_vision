#include "armor_detector_debug.hpp"
#include <algorithm>

mas_auto_aim_armor::ArmorDetectorDebug::ArmorDetectorDebug() : debug_enabled_(true), display_size_(640, 480) {}

void mas_auto_aim_armor::ArmorDetectorDebug::showDebugImages(const cv::Mat& originalImage,
                                        const cv::Mat& binaryImage,
                                        const std::vector<LightBar>& lights,
                                        const std::vector<Armor>& armors,
                                        const std::string& windowName) {
    if (!debug_enabled_) return;

    cv::Mat debugImage = originalImage.clone();

    // 绘制灯条
    for (size_t i = 0; i < lights.size(); i++) {
        const LightBar& light = lights[i];

        // 绘制灯条的旋转矩形
        cv::Point2f vertices[4];
        light.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(debugImage, vertices[j], vertices[(j + 1) % 4],
                     light.color == EnemyColor::RED ? cv::Scalar(0, 0, 255) :
                     light.color == EnemyColor::BLUE ? cv::Scalar(255, 0, 0) :
                     cv::Scalar(255, 255, 255), 2);
        }

        // 绘制中心点
        cv::circle(debugImage, light.center, 3, cv::Scalar(0, 255, 0), -1);

        // 绘制编号
        std::string label = std::to_string(i);
        cv::putText(debugImage, label, light.center, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(0, 255, 255), 1);
    }

    // 绘制装甲板
    for (const auto& armor : armors) {
        // 获取装甲板的角点
        auto landmarks = armor.landmarks();

        // 绘制装甲板边界
        for (size_t i = 0; i < landmarks.size(); i++) {
            cv::line(debugImage, landmarks[i], landmarks[(i + 1) % landmarks.size()],
                     cv::Scalar(0, 255, 0), 2);
        }

        // 绘制装甲板中心点
        cv::circle(debugImage, armor.center, 5, cv::Scalar(0, 255, 255), -1);

        // 显示装甲板类型
        std::string armor_type = (armor.type == ArmorType::SMALL) ? "SMALL" : "LARGE";
        cv::putText(debugImage, armor_type, cv::Point(armor.center.x - 20, armor.center.y - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

        // 显示识别的数字和置信度
        if (!armor.number.empty()) {
            cv::putText(debugImage, armor.classification_result,
                       cv::Point(armor.center.x - 20, armor.center.y + 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
    }

    cv::Mat resizedImage;
    cv::resize(debugImage, resizedImage, display_size_, 0, 0, cv::INTER_LINEAR);
    cv::imshow(windowName, resizedImage);
}

void mas_auto_aim_armor::ArmorDetectorDebug::showBinaryImage(const cv::Mat& binaryImage,
                                        const std::string& windowName) {
    if (!debug_enabled_) return;

    cv::Mat coloredBinaryImage;
    cv::cvtColor(binaryImage, coloredBinaryImage, cv::COLOR_GRAY2BGR);

    // 添加标签
    cv::putText(coloredBinaryImage,
               "Binary Image",
               cv::Point(10, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               1.0,
               cv::Scalar(0, 255, 0),
               2);

    cv::Mat resizedImage;
    cv::resize(coloredBinaryImage, resizedImage, display_size_, 0, 0, cv::INTER_LINEAR);
    cv::imshow(windowName, resizedImage);
}

void mas_auto_aim_armor::ArmorDetectorDebug::showCornerOptimization(const cv::Mat& originalImage,
                                               const std::vector<Armor>& armors,
                                               const std::string& windowName) {
    if (!debug_enabled_) return;

    cv::Mat cornerImage = originalImage.clone();

    for (const auto& armor : armors) {
        // 绘制原始旋转矩形顶点（蓝色）
        cv::Point2f left_vertices[4], right_vertices[4];
        armor.left_light.points(left_vertices);
        armor.right_light.points(right_vertices);

        for (int i = 0; i < 4; i++) {
            cv::circle(cornerImage, left_vertices[i], 4, cv::Scalar(255, 0, 0), -1);
            cv::circle(cornerImage, right_vertices[i], 4, cv::Scalar(255, 0, 0), -1);
        }

        // 绘制优化后的角点（绿色）
        cv::circle(cornerImage, armor.left_light.top, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(cornerImage, armor.left_light.bottom, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(cornerImage, armor.right_light.top, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(cornerImage, armor.right_light.bottom, 4, cv::Scalar(0, 255, 0), -1);

        // 绘制对称轴
        auto drawSymmetryAxis = [](cv::Mat& img, const LightBar& light, const cv::Scalar& color) {
            constexpr float AXIS_LENGTH = 30.0f;
            cv::Point2f start = light.center - light.axis * AXIS_LENGTH;
            cv::Point2f end = light.center + light.axis * AXIS_LENGTH;
            cv::line(img, start, end, color, 2, cv::LINE_AA);
        };

        drawSymmetryAxis(cornerImage, armor.left_light, cv::Scalar(0, 255, 255));
        drawSymmetryAxis(cornerImage, armor.right_light, cv::Scalar(0, 255, 255));
    }

    // 添加标签
    cv::putText(cornerImage,
               "Corner Optimization: Blue=Original, Green=Optimized, Cyan=Symmetry Axis",
               cv::Point(10, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               0.6,
               cv::Scalar(0, 255, 255),
               2);

    cv::Mat resizedImage;
    cv::resize(cornerImage, resizedImage, display_size_, 0, 0, cv::INTER_LINEAR);
    cv::imshow(windowName, resizedImage);
}

void mas_auto_aim_armor::ArmorDetectorDebug::showNumberRecognition(const std::vector<Armor>& armors,
                                              const std::string& windowName) {
    if (!debug_enabled_) return;

    cv::Mat numberImage;

    if (!armors.empty()) {
        std::vector<cv::Mat> numberImages;
        for (const auto& armor : armors) {
            if (!armor.number_img.empty()) {
                numberImages.push_back(armor.number_img);
            }
        }

        if (!numberImages.empty()) {
            // 垂直拼接所有数字图像
            cv::vconcat(numberImages, numberImage);

            // 添加标签
            cv::putText(numberImage,
                       "Recognized Numbers",
                       cv::Point(10, 20),
                       cv::FONT_HERSHEY_SIMPLEX,
                       0.5,
                       cv::Scalar(0, 255, 0),
                       1);
        } else {
            // 如果没有数字图像，创建一个空白图像
            numberImage = cv::Mat::zeros(100, 100, CV_8UC1);
            cv::putText(numberImage,
                       "No Numbers",
                       cv::Point(10, 50),
                       cv::FONT_HERSHEY_SIMPLEX,
                       0.5,
                       cv::Scalar(255, 255, 255),
                       1);
        }
    } else {
        // 如果没有装甲板，创建一个空白图像
        numberImage = cv::Mat::zeros(100, 100, CV_8UC1);
        cv::putText(numberImage,
                   "No Armors",
                   cv::Point(10, 50),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.5,
                   cv::Scalar(255, 255, 255),
                   1);
    }

    // 转换为彩色图像
    cv::Mat coloredNumberImage;
    cv::cvtColor(numberImage, coloredNumberImage, cv::COLOR_GRAY2BGR);

    cv::Mat resizedImage;
    cv::resize(coloredNumberImage, resizedImage, display_size_, 0, 0, cv::INTER_LINEAR);
    cv::imshow(windowName, resizedImage);
}
