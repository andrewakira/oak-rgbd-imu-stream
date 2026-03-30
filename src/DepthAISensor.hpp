#ifndef DepthAISensor_hpp
#define DepthAISensor_hpp

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/Sync.hpp"

class DepthAISensor {
public:
    struct ImuData {
        double timestamp;
        cv::Vec3f accel;
        cv::Vec3f gyro;
    };

    struct Frame {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        cv::Mat depth;
        cv::Mat rgb;
    };

    struct SensorData {
        std::vector<ImuData> imuSequence;
        Frame frame;
    };

    DepthAISensor();
    ~DepthAISensor();

    void start();
    bool getSensorData(SensorData& outData);
    void stop();

    void printCalibrationInfo(int width, int height);
    cv::Mat visualizeDepth(const cv::Mat& depth16U, int minDisplayMM = 800, int maxDisplayMM = 20000);
private:
    std::shared_ptr<dai::Device> device;

    std::atomic<bool> isRunning{false};
    std::thread imuThread;
    std::thread cameraThread;

    std::deque<ImuData> imuBuffer;
    std::deque<Frame> frameBuffer;

    std::mutex imuMutex;
    std::mutex frameMutex;
    std::condition_variable condVar;

    bool isBuildingUndistortedRGBMap = false;
    cv::Mat mapX, mapY;

    void initDevice();
    dai::Pipeline createPipeline();
    void imuLoop();
    void cameraLoop();
};

#endif
