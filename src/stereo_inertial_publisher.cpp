
#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <opencv2/opencv.hpp>

// Inludes common necessary includes for development using depthai library
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

using namespace std;

dai::Pipeline createPipeline() {
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    // imu
    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);

    auto xoutImu = pipeline.create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    imu->out.link(xoutImu->input);

    // monoLeft
    dai::node::MonoCamera::Properties::SensorResolution monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(30);

    // monoRight
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(30);

    // stereoDepth
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    stereo->initialConfig.setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_5x5);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_B);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    //rgb 
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P);
    camRgb->setIspScale(1, 6);  //320 x 180

    // sync with rgb, left, depth
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(chrono::milliseconds(16));
    stereo->rectifiedRight.link(sync->inputs["sync_right"]);
    stereo->rectifiedLeft.link(sync->inputs["sync_left"]);
    stereo->depth.link(sync->inputs["sync_depth"]);
    camRgb->isp.link(sync->inputs["sync_rgb"]);

    auto xoutSync = pipeline.create<dai::node::XLinkOut>();
    xoutSync->setStreamName("sync_out");
    sync->out.link(xoutSync->input);

    return pipeline;
}

int main(int argc, char** argv) {
    cout << "---" << endl;
    vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();
    for(auto deviceInfo : availableDevices) {
        cout << "[OAK] Device Mx ID : " << deviceInfo.getMxId() << endl;
    }
    if (availableDevices.size() == 0) {
        throw runtime_error("DepthAI Device not found.\n");
    } else if (availableDevices.size() > 1) {
        throw runtime_error("This code do 1 device.\n");
    }

    dai::Pipeline pipeline = createPipeline();
    shared_ptr<dai::Device> device = make_shared<dai::Device>(pipeline);

    vector<string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    cout << "[OAK] Device USB status : " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << endl;
    cout << "[OAK] Product name : " << device->getProductName() << endl;
    
    unordered_map<dai::CameraBoardSocket, string> sensors = device->getCameraSensorNames();
    for(const auto& [socket, name] : sensors) {
        cout << "[OAK] Camera socket: " << static_cast<int>(socket)
            << ", sensor: " << name << endl;
    }
    cout << "---" << endl;

    auto calibrationHandler = device->readCalibration();
    auto boardName = calibrationHandler.getEepromData().boardName;

    shared_ptr<dai::DataOutputQueue> syncQueue = device->getOutputQueue("sync_out", 30, false);
    shared_ptr<dai::DataOutputQueue> imuQueue = device->getOutputQueue("imu", 30, false);

    thread imuThread([&]() {
        while(true) {
            auto imuData = imuQueue->get<dai::IMUData>();
            if(imuData == nullptr) continue;

            for(const auto& imuPacket : imuData->packets) {
                auto acceleroValues = imuPacket.acceleroMeter;
                auto gyroValues = imuPacket.gyroscope;

                auto acceleroTs = acceleroValues.getTimestamp();
                auto gyroTs = gyroValues.getTimestamp();

                // Print accelerometer data
                //cout << "Accelerometer timestamp: " << acceleroTs.time_since_epoch().count() << endl;
                //cout << "Latency [ms]: " << timeDeltaToMilliS(chrono::steady_clock::now() - acceleroValues.getTimestamp()) << endl;
                //cout << "Accelerometer [m/s^2]: x: " << acceleroValues.x << " y: " << acceleroValues.y << " z: " << acceleroValues.z << endl;

                // Print gyroscope data
                //cout << "Gyroscope timestamp: " << gyroTs.time_since_epoch().count() << endl;
                //cout << "Gyroscope [rad/s]: x: " << gyroValues.x << " y: " << gyroValues.y << " z: " << gyroValues.z << endl;
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });

    thread cameraThread([&]() {
        while(true) {
            auto messageGroup = syncQueue->get<dai::MessageGroup>();
            if(messageGroup == nullptr) continue;

            auto frameRgb = messageGroup->get<dai::ImgFrame>("sync_rgb");
            auto frameDepth = messageGroup->get<dai::ImgFrame>("sync_depth");
            auto frameLeft = messageGroup->get<dai::ImgFrame>("sync_left");
            auto frameRight = messageGroup->get<dai::ImgFrame>("sync_right");

            //cout << frameRgb->getTimestamp().time_since_epoch().count()  << endl;

            cv::Mat cvFrame = frameRgb->getCvFrame();
            cv::imshow("frameRgb", cvFrame);
            cv::waitKey(1);

            this_thread::sleep_for(chrono::milliseconds(1));
        }

    });


    imuThread.join();
    cameraThread.join();


    
    return 0;
}
