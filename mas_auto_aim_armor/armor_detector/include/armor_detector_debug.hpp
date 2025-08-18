/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-18 09:00:08
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 09:11:25
 * @FilePath: /mas_vision/mas_auto_aim_armor/armor_detector/include/armor_detector_debug.hpp
 * @Description: 
 */
#ifndef ARMOR_DETECTOR_DEBUG_H
#define ARMOR_DETECTOR_DEBUG_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "armor_detector_types.h"

namespace mas_auto_aim_armor {
    class ArmorDetectorDebug {
        public:
            ArmorDetectorDebug();

            // 显示调试信息的函数
            void showDebugImages(const cv::Mat& originalImage,
                                const cv::Mat& binaryImage,
                                const std::vector<LightBar>& lights,
                                const std::vector<Armor>& armors,
                                const std::string& windowName = "Armor Detector Debug");

            // 显示二值化图像
            void showBinaryImage(const cv::Mat& binaryImage,
                                const std::string& windowName = "Binary Image");

            // 显示角点优化对比
            void showCornerOptimization(const cv::Mat& originalImage,
                                    const std::vector<Armor>& armors,
                                    const std::string& windowName = "Corner Optimization");

            // 显示数字识别结果
            void showNumberRecognition(const std::vector<Armor>& armors,
                                    const std::string& windowName = "Number Recognition");

            // 设置是否显示调试信息
            void setDebugEnabled(bool enabled) { debug_enabled_ = enabled; }

            // 获取调试状态
            bool isDebugEnabled() const { return debug_enabled_; }

        private:
            bool debug_enabled_;
            cv::Size display_size_;
    };
}

#endif // ARMOR_DETECTOR_DEBUG_H
