#include "armor_detector.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <sys/syscall.h>

#include "performance_monitor.hpp"
#include "pubsub.hpp"
#include "serial.hpp"

// 声明外部变量
extern std::atomic<bool> running;
extern std::atomic<bool> camReady;
extern std::atomic<bool> serialReady;
extern mas_utils::PerformanceMonitor perfMonitor;

//订阅者
Subscriber imageSubscriber;
Subscriber serialSubscriber;
std::atomic<bool> imagesubReady(false);
std::atomic<bool> serialsubReady(false);

// 定义内部变量
static cv::Mat SubImage;
static mas_serial::ReceivedDataMsg SubSerial;

// 在图像上绘制信息的函数
void drawInfoOnImage(cv::Mat& image,
                     double fps,
                     double processTime,
                     const mas_serial::ReceivedDataMsg& latestData,
                     double dataDelay) {
    // 在原始图像上添加信息
    cv::putText(image,
               "FPS: " + std::to_string(static_cast<int>(fps)),
               cv::Point(800, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               1.5,
               cv::Scalar(0, 255, 0),
               2);

    cv::putText(image,
               "Process Time: " + std::to_string(processTime) + " ms",
               cv::Point(10, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               1.5,
               cv::Scalar(0, 255, 0),
               2);

    //串口数据显示
    std::string serialInfo = "Serial - Y:" + std::to_string(latestData.yaw).substr(0, 6) +
                           " P:" + std::to_string(latestData.pitch).substr(0, 4) +
                           " R:" + std::to_string(latestData.roll).substr(0, 4) +
                           " D:" + std::to_string(dataDelay).substr(0, 4) + "ms";
    cv::putText(image,
               serialInfo,
               cv::Point(10, 75),
               cv::FONT_HERSHEY_SIMPLEX,
               1.5,
               cv::Scalar(0, 255, 255),
               2);
}

void processImage(const cv::Mat& image) {
    if (!image.empty())
    {
        SubImage = image.clone();
        imagesubReady = true;
    }

}

void processSerialData(const mas_serial::ReceivedDataMsg& serialData) {
    SubSerial = serialData;
    serialsubReady = true;
}

void armorDetectorThreadFunc() {
    // 等待相机和串口准备就绪
    while (running && (!camReady || !serialReady)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!running) return;

    // 订阅图像数据
    imageSubscriber.subscribe<cv::Mat>("camera/image", 
    [](const cv::Mat& image) {processImage(image);}, DeliveryMode::COPY);
    // 订阅串口数据
    serialSubscriber.subscribe<mas_serial::ReceivedDataMsg>("serial/data", 
    [](const mas_serial::ReceivedDataMsg& serialData) {processSerialData(serialData);}, DeliveryMode::COPY);
    // 初始化装甲板检测器
    mas_auto_aim_armor::ArmorDetector detector;

    // FPS计算相关变量
    auto lastTime = std::chrono::steady_clock::now();
    int frameCount = 0;
    double fps = 0.0;

    // 处理时间计算
    auto processStartTime = std::chrono::steady_clock::now();
    double processTime = 0.0;

    // 注册性能监控
    perfMonitor.addThread("ArmorDetector", perfMonitor.getThreadsId());
    
    // 保存上一次的模式，用于检测模式变化
    uint8_t lastMode = SubSerial.mode;
    // 标记是否处于符文模式
    bool inRuneMode = false;
    
    while (running) {
        while (!camReady || !serialReady) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
        
        // 检查模式是否发生变化
        if (SubSerial.mode != lastMode) {
            // 根据模式设置装甲板检测器的颜色或挂起/恢复检测
            switch (SubSerial.mode) {
                case 0: // AUTO_AIM_RED
                    detector.setDetectColor(EnemyColor::RED);
                    inRuneMode = false;
                    std::cout << "Switching to RED detection mode" << std::endl;
                    break;
                case 1: // AUTO_AIM_BLUE
                    detector.setDetectColor(EnemyColor::BLUE);
                    inRuneMode = false;
                    std::cout << "Switching to BLUE detection mode" << std::endl;
                    break;
                case 2: // SMALL_RUNE_RED
                case 3: // SMALL_RUNE_BLUE
                case 4: // BIG_RUNE_RED
                case 5: // BIG_RUNE_BLUE
                    inRuneMode = true;
                    std::cout << "Entering rune mode, pausing armor detection" << std::endl;
                    break;
                default:
                    std::cout << "Unknown mode: " << static_cast<int>(SubSerial.mode) << std::endl;
                    break;
            }
            lastMode = SubSerial.mode;
        }
        
        // 如果处于符文模式，则跳过装甲板检测
        if (inRuneMode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (!SubImage.empty() && imagesubReady && serialsubReady) {
            imagesubReady = false;
            serialsubReady = false;
            processStartTime = std::chrono::steady_clock::now();
            
            // 创建用于处理的图像副本
            cv::Mat processImage = SubImage.clone();
            
            // 执行装甲板检测
            std::vector<Armor> armors = detector.getArmors(processImage);
            // 计算处理时间
            auto processEndTime = std::chrono::steady_clock::now();
            processTime = std::chrono::duration<double, std::milli>(processEndTime - processStartTime).count();
            // 计算FPS
            frameCount++;
            auto currentTime = std::chrono::steady_clock::now();
            double elapsedTime = std::chrono::duration<double>(currentTime - lastTime).count();
            if (elapsedTime >= 1.0) {
                fps = frameCount / elapsedTime;
                frameCount = 0;
                lastTime = currentTime;
            }
            // 计算数据延迟（当前时间与串口数据时间戳的差值）
            auto now = std::chrono::steady_clock::now();
            double dataDelay = std::chrono::duration<double, std::milli>(now - SubSerial.timestamp).count();

            // 创建一个副本来显示调试信息（根据配置文件决定是否显示）
            cv::Mat debugFrame = processImage.clone();
            detector.showDebugInfo(debugFrame);
            // 在处理图像副本上绘制检测到的装甲板和数字（始终显示）
            detector.drawArmors(processImage, armors);

            // 绘制相机中心点
            cv::Point2f camera_center = cv::Point2f(processImage.cols / 2.0f, processImage.rows / 2.0f);
            cv::circle(processImage, camera_center, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(processImage, camera_center, 10, cv::Scalar(0, 255, 0), 2);

            // 在图像上绘制各种信息
            drawInfoOnImage(processImage, fps, processTime, SubSerial, dataDelay);
            // 显示主结果图像
            cv::Mat resizedDrawingFrame;
            cv::resize(processImage, resizedDrawingFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
            cv::imshow("Armor Detection Result", resizedDrawingFrame);
            // 等待按键
            cv::waitKey(1);
        }
    }
    // 关闭所有OpenCV窗口
    if (running) {
        cv::destroyAllWindows();
    }
}

// 启动线程
void startarmordetectorThread() {
    static std::thread armordetector_thread(armorDetectorThreadFunc);
    // 分离线程，让它独立运行
    armordetector_thread.detach();
}
