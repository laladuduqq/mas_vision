/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-17 22:13:38
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-17 23:40:43
 * @FilePath: /mas_vision/rm_utils/PubSub/test/test.cpp
 * @Description: PubSub系统测试程序
 */

#include "pubsub.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <opencv2/opencv.hpp>

// 图像数据结构
struct ImageData {
    std::vector<uint8_t> data;
    int width;
    int height;
    int channels;
    
    ImageData() : width(0), height(0), channels(0) {}
    
    ImageData(const cv::Mat& image) 
        : width(image.cols), height(image.rows), channels(image.channels()) {
        if (!image.empty()) {
            data.resize(image.total() * image.channels());
            std::memcpy(data.data(), image.data, data.size());
        }
    }
    
    cv::Mat toMat() const {
        if (data.empty()) return cv::Mat();
        return cv::Mat(height, width, CV_8UC(channels), const_cast<uint8_t*>(data.data())).clone();
    }
};

// 测试数据结构
struct SensorData {
    int value;
    double timestamp;
    
    SensorData() : value(0), timestamp(0.0) {}
    
    SensorData(int v) : value(v), timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count()) {}
};

// 性能测试结果
struct PerformanceResult {
    int messages_sent;
    int messages_received;
    double duration_ms;
    double messages_per_second;
};

// 功能测试
void functionTest() {
    std::cout << "=== 开始功能测试 ===" << std::endl;
    
    // 启动消息中心
    MessageCenter& center = MessageCenter::getInstance();
    center.start();
    
    // 启动处理线程
    std::thread processing_thread([&center]() {
        while (center.isRunning()) {
            center.processMessages();
        }
    });
    
    std::atomic<int> received_count(0);
    std::atomic<int> received_data(0);
    
    // 创建订阅者
    Subscriber subscriber1, subscriber2;
    
    // 订阅图像主题 - 使用指针模式
    subscriber1.subscribe<ImageData>("camera/image", [&received_count](const ImageData& img_data) {
        received_count++;
        if (received_count % 100 == 0) {
            std::cout << "收到图像消息: " << received_count << std::endl;
        }
    }, DeliveryMode::POINTER);
    
    // 订阅数据主题 - 使用复制模式
    subscriber2.subscribe<SensorData>("sensor/data", [&received_data](const SensorData& sensor_data) {
        received_data += sensor_data.value;
        std::cout << "收到数据消息: " << sensor_data.value 
                  << ", 累计: " << received_data << std::endl;
    }, DeliveryMode::COPY);
    
    // 确保订阅者有时间注册
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 创建发布者
    Publisher<ImageData> image_publisher("camera/image");
    Publisher<SensorData> data_publisher("sensor/data");
    
    // 发布一些测试消息
    for (int i = 0; i < 5; ++i) {
        // 发布图像消息
        cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(i*50, i*50, i*50));
        ImageData img_data(test_image);
        image_publisher.publish(img_data);
        
        // 发布数据消息
        SensorData sensor_data(i + 1);
        data_publisher.publish(sensor_data);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 给一些时间处理消息
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "功能测试结果:" << std::endl;
    std::cout << "  图像消息接收数量: " << received_count << std::endl;
    std::cout << "  数据累计值: " << received_data << std::endl;
    
    // 停止消息中心
    center.stop();
    
    // 等待处理线程结束
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
    
    std::cout << "=== 功能测试完成 ===" << std::endl << std::endl;
}

// 性能测试
PerformanceResult performanceTest(int message_count, int thread_count) {
    std::cout << "=== 开始性能测试 ===" << std::endl;
    std::cout << "消息数量: " << message_count << ", 线程数: " << thread_count << std::endl;
    
    MessageCenter& center = MessageCenter::getInstance();
    center.start();
    
    // 启动处理线程
    std::thread processing_thread([&center]() {
        while (center.isRunning()) {
            center.processMessages();
        }
    });
    
    std::atomic<int> total_received(0);
    std::vector<std::unique_ptr<Subscriber>> subscribers(thread_count);
    
    // 创建多个订阅者线程
    std::vector<std::thread> sub_threads;
    for (int i = 0; i < thread_count; ++i) {
        sub_threads.emplace_back([i, &total_received, &subscribers]() {
            subscribers[i] = std::make_unique<Subscriber>();
            subscribers[i]->subscribe<SensorData>("test/topic", [&total_received](const SensorData& data) {
                total_received++;
            }, DeliveryMode::POINTER);
        });
    }
    
    // 等待订阅者线程完成初始化
    for (auto& t : sub_threads) {
        t.join();
    }
    
    // 确保订阅者有时间注册
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 创建发布者
    Publisher<SensorData> publisher("test/topic");
    
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 发布大量消息
    for (int i = 0; i < message_count; ++i) {
        SensorData data(i);
        publisher.publish(data);
    }
    
    // 等待所有消息处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 计算性能结果
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double duration_ms = duration.count() / 1000.0;
    double messages_per_second = (message_count / duration_ms) * 1000;
    
    PerformanceResult result;
    result.messages_sent = message_count;
    result.messages_received = total_received;
    result.duration_ms = duration_ms;
    result.messages_per_second = messages_per_second;
    
    // 停止消息中心
    center.stop();
    
    // 等待处理线程结束
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
    
    std::cout << "性能测试结果:" << std::endl;
    std::cout << "  发送消息数: " << result.messages_sent << std::endl;
    std::cout << "  接收消息数: " << result.messages_received << std::endl;
    std::cout << "  耗时: " << result.duration_ms << " ms" << std::endl;
    std::cout << "  吞吐量: " << result.messages_per_second << " 消息/秒" << std::endl;
    
    std::cout << "=== 性能测试完成 ===" << std::endl << std::endl;
    
    return result;
}

// 高频率图像传输测试 - 指针模式
PerformanceResult highFrequencyImageTestPointer() {
    std::cout << "=== 开始高频率图像传输测试 (指针模式) ===" << std::endl;
    
    const int image_width = 640;
    const int image_height = 480;
    const int message_count = 500; // 500Hz持续1秒
    
    MessageCenter& center = MessageCenter::getInstance();
    center.setBatchSize(20);
    center.setQueueSizeLimit(1000);
    center.start();
    
    // 启动处理线程
    std::thread processing_thread([&center]() {
        while (center.isRunning()) {
            center.processMessages();
        }
    });
    
    std::atomic<int> received_count(0);
    
    // 创建订阅者
    Subscriber subscriber;
    subscriber.subscribe<ImageData>("camera/image", [&received_count](const ImageData& img_data) {
        received_count++;
    }, DeliveryMode::POINTER);
    
    // 确保订阅者有时间注册
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 创建发布者
    Publisher<ImageData> publisher("camera/image");
    
    // 创建测试图像数据
    cv::Mat test_image(image_height, image_width, CV_8UC3, cv::Scalar(100, 150, 200));
    ImageData img_data(test_image);
    
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 高频率发布图像消息
    for (int i = 0; i < message_count; ++i) {
        publisher.publish(img_data);
    }
    
    // 等待处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 计算性能结果
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double duration_ms = duration.count() / 1000.0;
    double messages_per_second = (message_count / duration_ms) * 1000;
    
    PerformanceResult result;
    result.messages_sent = message_count;
    result.messages_received = received_count;
    result.duration_ms = duration_ms;
    result.messages_per_second = messages_per_second;
    
    // 停止消息中心
    center.stop();
    
    // 等待处理线程结束
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
    
    std::cout << "高频率图像传输测试结果 (指针模式):" << std::endl;
    std::cout << "  图像尺寸: " << image_width << "x" << image_height << std::endl;
    std::cout << "  发送图像数: " << result.messages_sent << std::endl;
    std::cout << "  接收图像数: " << result.messages_received << std::endl;
    std::cout << "  耗时: " << result.duration_ms << " ms" << std::endl;
    std::cout << "  吞吐量: " << result.messages_per_second << " 图像/秒" << std::endl;
    
    std::cout << "=== 高频率图像传输测试完成 (指针模式) ===" << std::endl << std::endl;
    
    return result;
}

// 高频率图像传输测试 - 复制模式
PerformanceResult highFrequencyImageTestCopy() {
    std::cout << "=== 开始高频率图像传输测试 (复制模式) ===" << std::endl;
    
    const int image_width = 1280;
    const int image_height = 1024;
    const int message_count = 3000; // 500Hz持续1秒
    
    MessageCenter& center = MessageCenter::getInstance();
    center.setBatchSize(20);
    center.setQueueSizeLimit(1000);
    center.start();
    
    // 启动处理线程
    std::thread processing_thread([&center]() {
        while (center.isRunning()) {
            center.processMessages();
        }
    });
    
    std::atomic<int> received_count(0);
    
    // 创建订阅者
    Subscriber subscriber;
    subscriber.subscribe<ImageData>("camera/image", [&received_count](const ImageData& img_data) {
        received_count++;
    }, DeliveryMode::COPY);
    
    // 确保订阅者有时间注册
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 创建发布者
    Publisher<ImageData> publisher("camera/image");
    
    // 创建测试图像数据
    cv::Mat test_image(image_height, image_width, CV_8UC3, cv::Scalar(100, 150, 200));
    ImageData img_data(test_image);
    
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 高频率发布图像消息
    for (int i = 0; i < message_count; ++i) {
        publisher.publish(img_data);
    }
    
    // 等待处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 计算性能结果
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double duration_ms = duration.count() / 1000.0;
    double messages_per_second = (message_count / duration_ms) * 1000;
    
    PerformanceResult result;
    result.messages_sent = message_count;
    result.messages_received = received_count;
    result.duration_ms = duration_ms;
    result.messages_per_second = messages_per_second;
    
    // 停止消息中心
    center.stop();
    
    // 等待处理线程结束
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
    
    std::cout << "高频率图像传输测试结果 (复制模式):" << std::endl;
    std::cout << "  图像尺寸: " << image_width << "x" << image_height << std::endl;
    std::cout << "  发送图像数: " << result.messages_sent << std::endl;
    std::cout << "  接收图像数: " << result.messages_received << std::endl;
    std::cout << "  耗时: " << result.duration_ms << " ms" << std::endl;
    std::cout << "  吞吐量: " << result.messages_per_second << " 图像/秒" << std::endl;
    
    std::cout << "=== 高频率图像传输测试完成 (复制模式) ===" << std::endl << std::endl;
    
    return result;
}

int main() {
    std::cout << "PubSub系统测试程序" << std::endl;
    std::cout << "========================" << std::endl;
    
    try {
        // 运行功能测试
        functionTest();
        
        // 运行性能测试
        auto perf_result1 = performanceTest(1000, 1);
        auto perf_result2 = performanceTest(1000, 5);
        
        // 运行高频率图像传输测试 - 指针模式
        auto image_result_pointer = highFrequencyImageTestPointer();
        
        // 运行高频率图像传输测试 - 复制模式
        auto image_result_copy = highFrequencyImageTestCopy();
        
        // 输出汇总结果
        std::cout << "测试汇总:" << std::endl;
        std::cout << "========================" << std::endl;
        std::cout << "基础性能测试 (1线程): " << perf_result1.messages_per_second << " 消息/秒" << std::endl;
        std::cout << "多线程性能测试 (5线程): " << perf_result2.messages_per_second << " 消息/秒" << std::endl;
        std::cout << "高频率图像传输测试 (指针模式): " << image_result_pointer.messages_per_second << " 图像/秒" << std::endl;
        std::cout << "高频率图像传输测试 (复制模式): " << image_result_copy.messages_per_second << " 图像/秒" << std::endl;
        
        if (image_result_copy.messages_per_second >= 500) {
            std::cout << "✓ 满足500Hz图像传输性能要求 (复制模式)" << std::endl;
        } else {
            std::cout << "✗ 未满足500Hz图像传输性能要求 (复制模式)" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "测试过程中发生错误: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "测试完成!" << std::endl;
    return 0;
}