#ifndef ARMOR_DETECTOR_TYPES_H
#define ARMOR_DETECTOR_TYPES_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>  
#include <Eigen/Dense> 

// 定义敌方颜色枚举
enum class EnemyColor {
    RED,
    BLUE,
    WHITE
};

// 装甲板类型枚举
enum class ArmorType {
    SMALL,
    LARGE,
    INVALID
};

// 灯条结构体
struct LightBar : public cv::RotatedRect {
    LightBar() = default;
    explicit LightBar(const std::vector<cv::Point> &contour)
            : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::WHITE) {
        if (contour.empty()) return;

        center = std::accumulate(
                contour.begin(),
                contour.end(),
                cv::Point2f(0, 0),
                [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b) {
                    return a + cv::Point2f(b.x, b.y) / n;
                });

        cv::Point2f p[4];
        this->points(p);
        std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        axis = top - bottom;
        axis = axis / cv::norm(axis);

        // Calculate the tilt angle
        // The angle is the angle between the light bar and the horizontal line
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    EnemyColor color;
    cv::Point2f top, bottom, center;
    cv::Point2f axis;
    double length;
    double width;
    float tilt_angle;
};

// 装甲板结构体
struct Armor {
    static constexpr const int N_LANDMARKS = 6;
    static constexpr const int N_LANDMARKS_2 = N_LANDMARKS * 2;

    Armor() = default;
    Armor(const LightBar &l1, const LightBar &l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }

        center = (left_light.center + right_light.center) / 2;
    }

    // Landmarks start from bottom left in clockwise order
    std::vector<cv::Point2f> landmarks() const {
        return {left_light.bottom,
                left_light.center,
                left_light.top,
                right_light.top,
                right_light.center,
                right_light.bottom};
    }

    // Light pairs part
    LightBar left_light, right_light;
    cv::Point2f center;
    ArmorType type;

    // Number part
    cv::Mat number_img;
    std::string number;
    float confidence;
    std::string classification_result;

    // Pose part (新增)
    Eigen::Vector3d position;      // 位置 (x, y, z)
    Eigen::Quaterniond orientation; // 四元数方向
    bool pose_valid;               // 姿态是否有效
    float distance_to_image_center;
};

// 灯条参数配置结构体
struct LightParams {
    double min_ratio;
    double max_ratio;
    double max_angle;
    int color_diff_thresh;
};

// 装甲板参数配置结构体
struct ArmorParams {
    double min_light_ratio;
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    double max_angle;
};

#endif // ARMOR_DETECTOR_TYPES_H
