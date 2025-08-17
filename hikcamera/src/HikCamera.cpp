#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include "HikCamera.h"
#include "MvCameraControl.h"

hikcamera::HikCamera::HikCamera() : handle(NULL), isConnected(false) {
    memset(serialNumber, 0, sizeof(serialNumber));
}

hikcamera::HikCamera::~HikCamera() {
    closeCamera();
}

bool hikcamera::HikCamera::openCamera() {
    int nRet = MV_OK;

    // Initialize SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return false;
    }


    // ch:枚举设备 | en:Enum device
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return false;
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo){break;} 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return false;
    }

    // ch:选择设备并创建句柄,默认选择第一个设备 | en:Select device and create handle, default to select the first device
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet)
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // ch:打开设备 | en:Open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("Open Device fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // 默认参数
    float exposure_time = 5000.0f;
    float gain = 10.0f;

    try {
        cv::FileStorage fs("config/camera_set.json", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "无法打开 JSON 文件，使用默认值\n";
        } else {
            cv::FileNode cam = fs["camera"];
            if (!cam.empty()) {
                exposure_time = (float)cam["exposuretime"];  // 支持 int/float 自动转换
                gain          = (float)cam["gain"];
            }
            fs.release();
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV 异常: " << e.what() << "\n";
    }

    // Exposure time
    // 获取并设置曝光
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    if (MV_OK == nRet) {
        printf("exposure time current value:%f\n", stExposureTime.fCurValue);
        if (exposure_time >= stExposureTime.fMin && exposure_time <= stExposureTime.fMax) {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time);
            if (MV_OK == nRet) printf("set exposure time:%f OK!\n",exposure_time);
            else printf("set exposure time failed! nRet [%x]\n", nRet);
        } else {
            printf("exposure value invalid!\n");
        }
    } else {
        printf("Get ExposureTime failed! nRet [0x%x]\n", nRet);
        return false;
    }

    // Gain
    // 获取并设置gain
    MVCC_FLOATVALUE stGain = {0};
    nRet = MV_CC_GetFloatValue(handle, "Gain", &stGain);
    if (MV_OK == nRet) {
        printf("gain current value:%f\n", stGain.fCurValue);
        if (gain >= stGain.fMin && gain <= stGain.fMax) {
            nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
            if (MV_OK == nRet) printf("set gain:%f OK!\n", gain);
            else printf("set gain failed! nRet [%x]\n", nRet);
        } else {
            printf("gain value invalid!\n");
        }
    } else {
        printf("Get Gain failed! nRet [0x%x]\n", nRet);
        return false;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValueEx(handle,"GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK)
            {
                printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
            }
        }
        else  
        {
            printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
        }
    }


    // ch:设置触发模式为off | en:Set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return false;
    }
    g_nPayloadSize = stParam.nCurValue;

    // ch:开始取流 | en:Start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return false;
    }
    
    isConnected = true;
    
    return true;
}

void hikcamera::HikCamera::closeCamera() {
    if (isConnected) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = NULL;

        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
        isConnected = false;
        MV_CC_Finalize();
    }
}

bool hikcamera::HikCamera::grabImage(cv::Mat& outImg) {
    if (!isConnected) {
        return false;
    }

    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

    int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
    if (nRet == MV_OK && stOutFrame.pBufAddr) {
        // 优先用海康SDK转换为RGB8
        if (stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8 ||
            stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8) {

            // 设置插值方法为均衡
            MV_CC_SetBayerCvtQuality(handle, 1);

            int width = stOutFrame.stFrameInfo.nWidth;
            int height = stOutFrame.stFrameInfo.nHeight;
            int outBufSize = width * height * 3 + 2048;
            std::vector<unsigned char> rgbBuf(outBufSize);

            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            stConvertParam.nWidth = width;
            stConvertParam.nHeight = height;
            stConvertParam.pSrcData = (unsigned char*)stOutFrame.pBufAddr;
            stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLenEx;
            stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = rgbBuf.data();
            stConvertParam.nDstBufferSize = outBufSize;

            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK == nRet) {
                outImg = cv::Mat(height, width, CV_8UC3, rgbBuf.data()).clone();
                cv::cvtColor(outImg, outImg, cv::COLOR_RGB2BGR); // 关键：RGB转BGR
            } else {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                MV_CC_FreeImageBuffer(handle, &stOutFrame);
                return false;
            }
        } else if (stOutFrame.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
            outImg = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3, stOutFrame.pBufAddr).clone();
        } else {
            cv::Mat imgMono(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC1, stOutFrame.pBufAddr);
            cv::cvtColor(imgMono, outImg, cv::COLOR_GRAY2BGR);
        }

        MV_CC_FreeImageBuffer(handle, &stOutFrame);
        fail_conut_ =0;
        
        return true;
    } else {
        printf("Get image buffer failed! nRet [0x%x]\n", nRet);
        MV_CC_StopGrabbing(handle);
        MV_CC_StartGrabbing(handle);
        fail_conut_++;
        if (fail_conut_ > 5)
        {
            closeCamera();
        }
    }
    return false;
}

bool hikcamera::HikCamera::calibrateCamera(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                               const std::vector<std::vector<cv::Point2f>>& imagePoints,
                               const cv::Size& imageSize,
                               cv::Mat& cameraMatrix,
                               cv::Mat& distCoeffs) {
    if (objectPoints.empty() || imagePoints.empty()) {
        printf("Calibration points are empty!\n");
        return false;
    }

    if (objectPoints.size() != imagePoints.size()) {
        printf("Object points and image points size mismatch!\n");
        return false;
    }

    // 初始化相机内参矩阵和畸变系数
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    // 执行相机校准
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    printf("Re-projection error reported by calibrateCamera: %g\n", rms);

    return true;
}

bool hikcamera::HikCamera::saveCalibration(const std::string& filename,
                               const cv::Mat& cameraMatrix,
                               const cv::Mat& distCoeffs,
                               const cv::Size& imageSize) {
    try {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            printf("Failed to open %s for writing\n", filename.c_str());
            return false;
        }

        fs << "image_width" << imageSize.width;
        fs << "image_height" << imageSize.height;

        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;

        fs.release();
        printf("Calibration data saved to %s\n", filename.c_str());
        return true;
    } catch (const cv::Exception& e) {
        printf("Failed to save calibration: %s\n", e.what());
        return false;
    }
}


bool hikcamera::HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}