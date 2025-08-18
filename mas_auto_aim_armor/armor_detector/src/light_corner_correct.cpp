#include "light_corner_correct.hpp"
#include <numeric>

mas_auto_aim_armor::LightCornerCorrector::LightCornerCorrector() {
    
}

void mas_auto_aim_armor::LightCornerCorrector::correctCorners(Armor &armor, const cv::Mat &gray_img) {
    // 检查输入参数的有效性
    if (gray_img.empty()) {
        return;
    }

    // If the width of the light is too small, the correction is not performed
    constexpr int PASS_OPTIMIZE_WIDTH = 3;

    if (armor.left_light.width > PASS_OPTIMIZE_WIDTH) {
        // Find the symmetry axis of the light
        SymmetryAxis left_axis = findSymmetryAxis(gray_img, armor.left_light);
        armor.left_light.center = left_axis.centroid;
        armor.left_light.axis = left_axis.direction;
        // Find the corner of the light
        cv::Point2f t = findCorner(gray_img, armor.left_light, left_axis, "top");
        if (t.x > 0) {
            armor.left_light.top = t;
        }
        cv::Point2f b = findCorner(gray_img, armor.left_light, left_axis, "bottom");
        if (b.x > 0) {
            armor.left_light.bottom = b;
        }
    }

    if (armor.right_light.width > PASS_OPTIMIZE_WIDTH) {
        // Find the symmetry axis of the light
        SymmetryAxis right_axis = findSymmetryAxis(gray_img, armor.right_light);
        armor.right_light.center = right_axis.centroid;
        armor.right_light.axis = right_axis.direction;
        // Find the corner of the light
        cv::Point2f t = findCorner(gray_img, armor.right_light, right_axis, "top");
        if (t.x > 0) {
            armor.right_light.top = t;
        }
        cv::Point2f b = findCorner(gray_img, armor.right_light, right_axis, "bottom");
        if (b.x > 0) {
            armor.right_light.bottom = b;
        }
    }
}

mas_auto_aim_armor::SymmetryAxis mas_auto_aim_armor::LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, const LightBar &light) {
    constexpr float MAX_BRIGHTNESS = 25;
    constexpr float SCALE = 0.07;

    // Scale the bounding box
    cv::Rect light_box = light.boundingRect();
    light_box.x -= cvRound(light_box.width * SCALE);
    light_box.y -= cvRound(light_box.height * SCALE);
    light_box.width += cvRound(light_box.width * SCALE * 2);
    light_box.height += cvRound(light_box.height * SCALE * 2);

    // Check boundary
    light_box.x = std::max(light_box.x, 0);
    light_box.x = std::min(light_box.x, gray_img.cols - 1);
    light_box.y = std::max(light_box.y, 0);
    light_box.y = std::min(light_box.y, gray_img.rows - 1);
    light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
    light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

    // 检查边界有效性
    if (light_box.width <= 0 || light_box.height <= 0 || 
        light_box.x >= gray_img.cols || light_box.y >= gray_img.rows) {
        cv::Point2f axis(0, -1); // 默认向上
        return SymmetryAxis{light.center, axis, 0};
    }

    // Get normalized light image
    cv::Mat roi = gray_img(light_box);
    float mean_val = (float)cv::mean(roi)[0];
    roi.convertTo(roi, CV_32F);
    cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

    // Calculate the centroid
    cv::Moments moments = cv::moments(roi, false);
    // 检查moments是否有效
    if (moments.m00 == 0) {
        cv::Point2f axis(0, -1); // 默认向上
        return SymmetryAxis{light.center, axis, mean_val};
    }
    
    cv::Point2f centroid = cv::Point2f((float)(moments.m10 / moments.m00), (float)(moments.m01 / moments.m00)) +
                           cv::Point2f((float)light_box.x, (float)light_box.y);

    // Initialize the PointCloud
    std::vector<cv::Point2f> points;
    for (int i = 0; i < roi.rows; i++) {
        for (int j = 0; j < roi.cols; j++) {
            // 添加边界检查
            if (i < roi.rows && j < roi.cols) {
                try {
                    float value = roi.at<float>(i, j);
                    // 使用std::round并确保非负
                    int count = std::max(0, cvRound(value));
                    for (int k = 0; k < count; k++) {
                        points.emplace_back(cv::Point2f((float)j, (float)i));
                    }
                } catch (...) {
                    // 忽略访问异常
                }
            }
        }
    }
    
    // 检查点云是否为空
    if (points.empty()) {
        cv::Point2f axis(0, -1); // 默认向上
        return SymmetryAxis{centroid, axis, mean_val};
    }
    
    cv::Mat points_mat = cv::Mat(points).reshape(1);

    // PCA (Principal Component Analysis)
    cv::PCA pca(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // Get the symmetry axis
    cv::Point2f axis = cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

    // Normalize the axis
    float norm = cv::norm(axis);
    if (norm > 0) {
        axis = axis / norm;
    }

    if (axis.y > 0) {
        axis = -axis;
    }

    return SymmetryAxis{centroid, axis, mean_val};
}

cv::Point2f mas_auto_aim_armor::LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const LightBar &light,
                                             const SymmetryAxis &axis,
                                             const std::string &order) {
    constexpr float START = 0.8f / 2;
    constexpr float END = 1.2f / 2;

    // 检查输入参数的有效性
    if (gray_img.empty()) {
        return cv::Point2f(-1, -1);
    }

    auto inImage = [&gray_img](const cv::Point &point) -> bool {
        return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
    };

    auto distance = [](float x0, float y0, float x1, float y1) -> float {
        return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    };

    int oper = order == "top" ? 1 : -1;
    float L = (float)light.length;
    float dx = axis.direction.x * oper;
    float dy = axis.direction.y * oper;

    std::vector<cv::Point2f> candidates;

    // Select multiple corner candidates and take the average as the final corner
    int n = (int)light.width - 2;
    if (n <= 0) {
        return cv::Point2f(-1, -1);
    }
    
    int half_n = cvRound(n / 2.0f);
    for (int i = -half_n; i <= half_n; i++) {
        float x0 = axis.centroid.x + L * START * dx + i;
        float y0 = axis.centroid.y + L * START * dy;

        cv::Point2f prev = cv::Point2f(x0, y0);
        cv::Point2f corner = cv::Point2f(x0, y0);
        float max_brightness_diff = 0;
        bool has_corner = false;
        
        // Search along the symmetry axis to find the corner that has the maximum brightness difference
        for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
             x += dx, y += dy) {
            cv::Point2f cur = cv::Point2f(x, y);
            if (!inImage(cv::Point(cur))) {
                break;
            }

            // 添加边界检查
            if (inImage(cv::Point(prev)) && inImage(cv::Point(cur))) {
                int prev_x = cvRound(prev.x);
                int prev_y = cvRound(prev.y);
                int cur_x = cvRound(cur.x);
                int cur_y = cvRound(cur.y);
                
                // 双重检查确保索引有效
                if (prev_y >= 0 && prev_y < gray_img.rows && prev_x >= 0 && prev_x < gray_img.cols &&
                    cur_y >= 0 && cur_y < gray_img.rows && cur_x >= 0 && cur_x < gray_img.cols) {
                    
                    try {
                        float brightness_diff = (float)(gray_img.at<uchar>(prev_y, prev_x) - 
                                                      gray_img.at<uchar>(cur_y, cur_x));
                        if (brightness_diff > max_brightness_diff && 
                            gray_img.at<uchar>(prev_y, prev_x) > axis.mean_val) {
                            max_brightness_diff = brightness_diff;
                            corner = prev;
                            has_corner = true;
                        }
                    } catch (...) {
                        // 忽略访问异常
                    }
                }
            }

            prev = cur;
        }

        if (has_corner) {
            candidates.emplace_back(corner);
        }
    }
    
    if (!candidates.empty()) {
        cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
        return result / (float)candidates.size();
    }

    return cv::Point2f(-1, -1);
}