/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 21:13:48
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 15:37:53
 * @FilePath: /mas_vision/mas_auto_aim_armor/armor_detector/include/armor_detector.hpp
 * @Description: 
 */
#ifndef _ARMOR_DETECTOR_H_
#define _ARMOR_DETECTOR_H_


#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>


#include "armor_detector_debug.hpp"
#include "armor_detector_types.h"
#include "light_corner_correct.hpp"
#include "number_classifier.hpp"

namespace mas_auto_aim_armor {
    class ArmorDetector {
    public:
        ArmorDetector();
        ~ArmorDetector();
        
        // 显示调试信息
        void showDebugInfo(const cv::Mat& originalImage) const;
        // 获取识别到的装甲板
        std::vector<Armor> getArmors(cv::Mat& image);
        // 在图像上绘制装甲板
        static void drawArmors(cv::Mat& image, const std::vector<Armor>& armors);
        // 设置要检测的装甲板颜色
        void setDetectColor(EnemyColor color);

    private:
        int binary_threshold_;  // 二值化阈值
        EnemyColor detect_color_; // 要检测的装甲板颜色
        // 灯条参数
        LightParams light_params_{};
        // 装甲板参数
        ArmorParams armor_params_{};
        // 存储识别到的灯条
        std::vector<LightBar> lights;
        // 存储识别到的装甲板
        std::vector<Armor> armors;
        // 存储二值化后的图像
        cv::Mat binary_image_;
        // 是否启用调试模式
        bool debug_enabled_;  
        // 数字识别器
        std::unique_ptr<NumberClassifier> classifier_;
        // 角点优化器
        std::unique_ptr<LightCornerCorrector> corner_corrector_;
        // 调试显示模块
        std::unique_ptr<ArmorDetectorDebug> debug_visualizer_;

        // 图像预处理方法 - 实现二值化
        void preprocessImage(const cv::Mat& input);
        // 查找灯条
        std::vector<LightBar> findLights(const cv::Mat& bin_img, const cv::Mat& src_img) const;
        // 匹配灯条形成装甲板（包含角点优化）
        std::vector<Armor> matchLights(const cv::Mat& src_img, const std::vector<LightBar>& lights) const;
        // 识别装甲板上的数字
        void recognizeArmorNumbers(const cv::Mat& src_img, std::vector<Armor>& armors);


        // 辅助函数：判断是否为灯条
        bool isLight(const LightBar& light) const;
        // 辅助函数：判断是否为装甲板
        ArmorType isArmor(const LightBar& light_1, const LightBar& light_2) const;
        // 辅助函数：检查两个灯条之间是否包含其他灯条
        bool containLight(size_t i, size_t j, const std::vector<LightBar>& lights) const;
        // 辅助函数：计算两点间距离
        static float calculateDistance(const cv::Point2f& point1, const cv::Point2f& point2);
        // 辅助函数：计算两点间中心点
        static cv::Point2f calculateCenter(const cv::Point2f& point1, const cv::Point2f& point2);
    };
}

#endif // _ARMOR_DETECTOR_H_