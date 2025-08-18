/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-18 09:00:08
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-18 09:10:39
 * @FilePath: /mas_vision/mas_auto_aim_armor/armor_detector/include/number_classifier.hpp
 * @Description: 
 */
#ifndef ARMOR_DETECTOR_NUMBER_CLASSIFIER_H
#define ARMOR_DETECTOR_NUMBER_CLASSIFIER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <mutex>

#include "armor_detector_types.h"

namespace mas_auto_aim_armor {
    class NumberClassifier {
        public:
            NumberClassifier(const std::string &model_path,
                            const std::string &label_path,
                            const double threshold,
                            const std::vector<std::string> &ignore_classes = {});

            // Extract the roi image of number from the src
            cv::Mat extractNumber(const cv::Mat &src, const Armor &armor) const;

            // Classify the number of the armor
            void classify(const cv::Mat &src, Armor &armor);

            // Erase the ignore classes
            void eraseIgnoreClasses(std::vector<Armor> &armors);

            double threshold;

        private:
            std::mutex mutex_;
            cv::dnn::Net net_;
            std::vector<std::string> class_names_;
            std::vector<std::string> ignore_classes_;
    };
}

#endif // ARMOR_DETECTOR_NUMBER_CLASSIFIER_H
