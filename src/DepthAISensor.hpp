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
    DepthAISensor();
    ~DepthAISensor();

    void start();
    void stop();
private:
    std::shared_ptr<dai::Device> device;

    std::atomic<bool> isRunning{false};
    std::thread imuThread;
    std::thread cameraThread;
    
    void initDevice();
    dai::Pipeline createPipeline();
    void imuLoop();
    void cameraLoop();
};

#endif