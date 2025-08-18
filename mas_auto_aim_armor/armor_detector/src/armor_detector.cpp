/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 21:13:32
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 10:55:04
 * @FilePath: /mas_vision/mas_auto_aim_armor/armor_detector/src/armor_detector.cpp
 * @Description: 
 */
#include "armor_detector.hpp"
#include "number_classifier.hpp"
#include "light_corner_correct.hpp"
#include "armor_detector_debug.hpp"
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>

mas_auto_aim_armor::ArmorDetector::ArmorDetector() 
{
    // 默认参数
    binary_threshold_= 90;
    detect_color_    = EnemyColor::RED;
    debug_enabled_   = false;
    // 默认灯条参数
    light_params_.min_ratio = 0.0001;
    light_params_.max_ratio = 1.0;
    light_params_.max_angle = 40.0;
    light_params_.color_diff_thresh = 20;
    // 默认装甲板参数
    armor_params_.min_light_ratio = 0.6;
    armor_params_.min_small_center_distance = 0.8;
    armor_params_.max_small_center_distance = 3.2;
    armor_params_.min_large_center_distance = 1.8;
    armor_params_.max_large_center_distance = 6.4;
    armor_params_.max_angle = 35.0;

    // 初始化角点优化器
    corner_corrector_ = std::make_unique<LightCornerCorrector>();

    // 初始化调试可视化模块
    debug_visualizer_ = std::make_unique<ArmorDetectorDebug>();

    try {
        cv::FileStorage fs("config/armor_detector.json", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "无法打开 JSON 配置文件: " << "config/armor_detector.json" << "，使用默认值\n";
        } else {
            cv::FileNode armor_detector = fs["armor_detector"];
            if (!armor_detector.empty()) {
                // 读取调试模式
                debug_enabled_ = static_cast<int>(armor_detector["debug"]) != 0;
                debug_visualizer_->setDebugEnabled(debug_enabled_);
                std::cout << "调试模式: " << (debug_enabled_ ? "启用" : "禁用") << std::endl;

                // 读取二值化阈值
                binary_threshold_ = static_cast<int>(armor_detector["binary_thres"]);
                std::cout << "读取到 binary_thres: " << binary_threshold_ << std::endl;

                // 读取检测颜色
                std::string detect_color_str = armor_detector["detect_color"];
                if (!detect_color_str.empty()) {
                    if (detect_color_str == "RED" || detect_color_str == "red") {
                        detect_color_ = EnemyColor::RED;
                    } else if (detect_color_str == "BLUE" || detect_color_str == "blue") {
                        detect_color_ = EnemyColor::BLUE;
                    } else {
                        std::cerr << "未知的检测颜色: " << detect_color_str << "，使用默认值 RED\n";
                        detect_color_ = EnemyColor::RED;
                    }
                    std::cout << "读取到 detect_color: " << detect_color_str << std::endl;
                }

                // 读取灯条参数
                cv::FileNode light_params = armor_detector["light_params"];
                if (!light_params.empty()) {
                    light_params_.min_ratio = static_cast<double>(light_params["min_ratio"]);
                    light_params_.max_ratio = static_cast<double>(light_params["max_ratio"]);
                    light_params_.max_angle = static_cast<double>(light_params["max_angle"]);
                    light_params_.color_diff_thresh = static_cast<int>(light_params["color_diff_thresh"]);
                }
                std::cout << "读取到 light_params:\n"
                              << "  min_ratio: " << light_params_.min_ratio << "\n"
                              << "  max_ratio: " << light_params_.max_ratio << "\n"
                              << "  max_angle: " << light_params_.max_angle << "\n"
                              << "  color_diff_thresh: " << light_params_.color_diff_thresh << std::endl;

                // 读取装甲板参数
                cv::FileNode armor_params = armor_detector["armor_params"];
                if (!armor_params.empty()) {
                    armor_params_.min_light_ratio = static_cast<double>(armor_params["min_light_ratio"]);
                    armor_params_.min_small_center_distance = static_cast<double>(armor_params["min_small_center_distance"]);
                    armor_params_.max_small_center_distance = static_cast<double>(armor_params["max_small_center_distance"]);
                    armor_params_.min_large_center_distance = static_cast<double>(armor_params["min_large_center_distance"]);
                    armor_params_.max_large_center_distance = static_cast<double>(armor_params["max_large_center_distance"]);
                    armor_params_.max_angle = static_cast<double>(armor_params["max_angle"]);
                }
                std::cout << "读取到 armor_params:\n"
                              << "  min_light_ratio: " << armor_params_.min_light_ratio << "\n"
                              << "  min_small_center_distance: " << armor_params_.min_small_center_distance << "\n"
                              << "  max_small_center_distance: " << armor_params_.max_small_center_distance << "\n"
                              << "  min_large_center_distance: " << armor_params_.min_large_center_distance << "\n"
                              << "  max_large_center_distance: " << armor_params_.max_large_center_distance << "\n"
                              << "  max_angle: " << armor_params_.max_angle << std::endl;

                // 读取数字识别参数
                cv::FileNode number_params = armor_detector["number_params"];
                if (!number_params.empty()) {
                    std::string model_path = number_params["model_path"];
                    std::string label_path = number_params["label_path"];
                    double threshold = static_cast<double>(number_params["classifier_threshold"]);

                    std::vector<std::string> ignore_classes;
                    cv::FileNode ignore_classes_node = number_params["ignore_classes"];
                    if (!ignore_classes_node.empty()) {
                        for (const auto& class_node : ignore_classes_node) {
                            ignore_classes.push_back(static_cast<std::string>(class_node));
                        }
                    }

                    // 创建数字识别器
                    classifier_ = std::make_unique<NumberClassifier>(
                        model_path, label_path, threshold, ignore_classes);
                    std::cout << "数字识别器加载成功，阈值: " << threshold << std::endl;
                    std::cout << "模型路径: " << model_path << std::endl;
                    std::cout << "标签路径: " << label_path << std::endl;
                } else {
                    std::cout << "未找到数字识别配置，跳过数字识别功能" << std::endl;
                }
            } else {
                std::cerr << "配置文件中未找到 armor_detector 配置项\n";
            }
            fs.release();
        }

        // 加载相机参数已在构造函数中完成
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV 异常: " << e.what() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "其他异常: " << e.what() << "\n";
    }

}

mas_auto_aim_armor::ArmorDetector::~ArmorDetector() = default;


void mas_auto_aim_armor::ArmorDetector::preprocessImage(const cv::Mat& input) {
    // 转换为灰度图
    cv::Mat gray;
    if (input.channels() == 3) {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = input;
    }
    // 应用阈值进行二值化
    cv::threshold(gray, binary_image_, binary_threshold_, 255, cv::THRESH_BINARY);
}

// 辅助函数：判断是否为灯条
bool mas_auto_aim_armor::ArmorDetector::isLight(const LightBar& light) const {
    // 灯条的长宽比 (短边 / 长边)
    float ratio = light.width / light.length;
    bool ratio_ok = light_params_.min_ratio < ratio && ratio < light_params_.max_ratio;

    bool angle_ok = light.tilt_angle < light_params_.max_angle;

    return ratio_ok && angle_ok;
}

std::vector<LightBar> mas_auto_aim_armor::ArmorDetector::findLights(const cv::Mat& bin_img, const cv::Mat& src_img) const {
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<LightBar> lights;

    // 遍历所有轮廓
    for (const auto& contour : contours) {
        // 轮廓太小则跳过
        if (contour.size() < 6) continue;

        // 使用fitLine获得直线参数
        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

        // 计算线段端点
        float vx = line[0];  // 方向向量x分量
        float vy = line[1];  // 方向向量y分量
        float x0 = line[2];   // 直线上一点x坐标
        float y0 = line[3];   // 直线上一点y坐标

        // 找到轮廓中距离直线端点最远的点作为灯条端点
        float min_proj = FLT_MAX, max_proj = -FLT_MAX;
        cv::Point2f min_point, max_point;

        for (const auto& point : contour) {
            // 计算点在直线方向上的投影
            float proj = (point.x - x0) * vx + (point.y - y0) * vy;
            if (proj < min_proj) {
                min_proj = proj;
                min_point = point;
            }
            if (proj > max_proj) {
                max_proj = proj;
                max_point = point;
            }
        }

        // 构造一个新的轮廓用于计算最小外接矩形
        std::vector<cv::Point> line_contour = {min_point, max_point};

        // 创建灯条对象
        auto light = LightBar(line_contour);

        // 使用最小外接矩形进一步优化
        cv::RotatedRect min_rect = cv::minAreaRect(contour);

        // 结合fitLine和minAreaRect的结果优化灯条参数
        // 优先使用通过fitLine计算得到的更准确的角度和端点
        light.center = min_rect.center;

        // 根据最小外接矩形重新计算长宽
        cv::Point2f rect_points[4];
        min_rect.points(rect_points);

        // 对矩形点按y坐标排序以确定上下边缘
        std::sort(rect_points, rect_points + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y < b.y;
        });

        // 计算上下边缘中点作为灯条的端点
        cv::Point2f top_edge_center = (rect_points[0] + rect_points[1]) * 0.5;
        cv::Point2f bottom_edge_center = (rect_points[2] + rect_points[3]) * 0.5;

        light.top = top_edge_center;
        light.bottom = bottom_edge_center;
        light.length = cv::norm(light.top - light.bottom);
        light.width = min_rect.size.width < min_rect.size.height ? min_rect.size.width : min_rect.size.height;

        // 重新计算倾斜角度
        light.tilt_angle = std::atan2(std::abs(light.top.x - light.bottom.x),
                                      std::abs(light.top.y - light.bottom.y));
        light.tilt_angle = light.tilt_angle / CV_PI * 180;

                // 判断是否为灯条
        if (isLight(light)) {
            // 判断灯条颜色
            int sum_r = 0, sum_b = 0;
            int valid_points = 0; // 记录有效点的数量
            
            // 检查图像是否有效
            if (!src_img.empty() && src_img.channels() == 3) {
                for (const auto& point : contour) {
                    // 添加更全面的边界检查
                    if (point.x >= 0 && point.x < src_img.cols && point.y >= 0 && point.y < src_img.rows) {
                        // 进一步检查确保点坐标是有效的
                        if (!isnan(point.x) && !isnan(point.y) && 
                            !isinf(point.x) && !isinf(point.y)) {
                            try {
                                // 注意：OpenCV中是BGR顺序
                                sum_b += src_img.at<cv::Vec3b>(point.y, point.x)[0]; // 蓝色通道
                                sum_r += src_img.at<cv::Vec3b>(point.y, point.x)[2]; // 红色通道
                                valid_points++;
                            } catch (...) {
                                // 忽略任何访问异常
                                continue;
                            }
                        }
                    }
                }
            }

            // 只有当存在有效点时才计算颜色
            if (valid_points > 0) {
                int avg_diff = std::abs(sum_r - sum_b) / valid_points;
                if (avg_diff > light_params_.color_diff_thresh) {
                    // 红色灯条
                    if (sum_r > sum_b) {
                        light.color = EnemyColor::RED;
                    } else { // 蓝色灯条
                        light.color = EnemyColor::BLUE;
                    }
                } else {
                    // 无法确定颜色，设为白色
                    light.color = EnemyColor::WHITE;
                }
            } else {
                // 没有有效点，设为白色
                light.color = EnemyColor::WHITE;
            }

            lights.emplace_back(light);
        }
    }

    // 按照中心点x坐标排序
    std::sort(lights.begin(), lights.end(), [](const LightBar& l1, const LightBar& l2) {
        return l1.center.x < l2.center.x;
    });

    return lights;
}

std::vector<Armor> mas_auto_aim_armor::ArmorDetector::matchLights(const cv::Mat& src_img, const std::vector<LightBar>& lights) const {
    std::vector<Armor> armors;

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        if (light_1->color != detect_color_) continue;
        double max_iter_width = light_1->length * armor_params_.max_large_center_distance;

        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_2->color != detect_color_) continue;
            if (containLight(light_1 - lights.begin(), light_2 - lights.begin(), lights)) {
                continue;
            }
            if (light_2->center.x - light_1->center.x > max_iter_width) break;

            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }

    // 如果有装甲板且角点优化器已初始化，则进行角点优化
    if (!armors.empty() && corner_corrector_ != nullptr) {
        // 转换为灰度图用于角点优化
        cv::Mat gray_img;
        if (src_img.channels() == 3) {
            cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
        } else {
            gray_img = src_img;
        }

        try {
            for (auto& armor : armors) {
                corner_corrector_->correctCorners(armor, gray_img);
            }
        } catch (const std::exception& e) {
            std::cerr << "角点优化过程中发生异常: " << e.what() << std::endl;
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool mas_auto_aim_armor::ArmorDetector::containLight(size_t i, size_t j, const std::vector<LightBar>& lights) const {
    const LightBar &light_1 = lights.at(i), light_2 = lights.at(j);
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);
    double avg_length = (light_1.length + light_2.length) / 2.0;
    double avg_width = (light_1.width + light_2.width) / 2.0;

    // Only check lights in between
    for (size_t k = i + 1; k < j; k++) {
        const LightBar &test_light = lights.at(k);

        // 防止数字干扰
        if (test_light.width > 2 * avg_width) {
            continue;
        }
        // 防止红点准星或弹丸干扰
        if (test_light.length < 0.5 * avg_length) {
            continue;
        }

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

ArmorType mas_auto_aim_armor::ArmorDetector::isArmor(const LightBar& light_1, const LightBar& light_2) const {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                               : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (armor_params_.min_small_center_distance <= center_distance &&
                               center_distance < armor_params_.max_small_center_distance) ||
                              (armor_params_.min_large_center_distance <= center_distance &&
                               center_distance < armor_params_.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < armor_params_.max_angle;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
        type = center_distance > armor_params_.min_large_center_distance ? ArmorType::LARGE
                                                                        : ArmorType::SMALL;
    } else {
        type = ArmorType::INVALID;
    }

    return type;
}

void mas_auto_aim_armor::ArmorDetector::recognizeArmorNumbers(const cv::Mat& src_img, std::vector<Armor>& armors) {
    if (!armors.empty() && classifier_ != nullptr) {
        try {
            // Process each armor
            for (auto& armor : armors) {
                // 1. Extract the number image
                armor.number_img = classifier_->extractNumber(src_img, armor);
                // 2. Do classification
                classifier_->classify(src_img, armor);
            }

            // 3. Erase the armors with ignore classes
            classifier_->eraseIgnoreClasses(armors);
        } catch (const std::exception& e) {
            std::cerr << "数字识别过程中发生异常: " << e.what() << std::endl;
        }
    } else {
        if (classifier_ == nullptr) {
            // 数字识别器未初始化，这是正常的（如果配置文件中没有相关配置）
            //std::cout << "数字识别器未初始化，跳过数字识别" << std::endl;
        }
    }
}



void mas_auto_aim_armor::ArmorDetector::drawArmors(cv::Mat& image, const std::vector<Armor>& armors) {
    for (const auto& armor : armors) {
        // 获取装甲板的角点
        auto landmarks = armor.landmarks();

        // 绘制装甲板边界
        for (size_t i = 0; i < landmarks.size(); i++) {
            cv::line(image, landmarks[i], landmarks[(i + 1) % landmarks.size()],
                     cv::Scalar(0, 255, 0), 2);
        }

        // 绘制装甲板中心点
        cv::circle(image, armor.center, 5, cv::Scalar(0, 255, 255), -1);

        // 显示装甲板类型
        std::string armor_type = (armor.type == ArmorType::SMALL) ? "SMALL" : "LARGE";
        cv::putText(image, armor_type, cv::Point(armor.center.x - 20, armor.center.y - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

        // 显示识别的数字和置信度
        if (!armor.number.empty()) {
            cv::putText(image, armor.classification_result,
                       cv::Point(armor.center.x - 20, armor.center.y + 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
        }

    }
}



void mas_auto_aim_armor::ArmorDetector::showDebugInfo(const cv::Mat& originalImage) const {
    if (!debug_enabled_) return;

    // 显示主调试图像（包含灯条和装甲板）
    debug_visualizer_->showDebugImages(originalImage, binary_image_, lights, armors);

    // 显示二值化图像
    debug_visualizer_->showBinaryImage(binary_image_);

    // 显示角点优化对比
    debug_visualizer_->showCornerOptimization(originalImage, armors);

    // 显示数字识别结果
    debug_visualizer_->showNumberRecognition(armors);
}

std::vector<Armor> mas_auto_aim_armor::ArmorDetector::getArmors(cv::Mat &image) {
    // 执行图像预处理（二值化）
    preprocessImage(image);
    // 寻找灯条
    lights = findLights( binary_image_, image);
    // 匹配灯条形成装甲板（包含角点优化）
    armors = matchLights(image,lights);
    // 识别装甲板上的数字
    recognizeArmorNumbers(image, armors);

    return armors;
}


float mas_auto_aim_armor::ArmorDetector::calculateDistance(const cv::Point2f& point1, const cv::Point2f& point2) {
    return std::sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                     (point1.y - point2.y) * (point1.y - point2.y));
}

cv::Point2f mas_auto_aim_armor::ArmorDetector::calculateCenter(const cv::Point2f& point1, const cv::Point2f& point2) {
    return cv::Point2f((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
}