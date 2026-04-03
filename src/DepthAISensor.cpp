#include "DepthAISensor.hpp"

#define DEBUGMODE 0
#if DEBUGMODE
#include "FPSCounter.hpp"
FPSCounter imuFps;
FPSCounter cameraFps;
#endif

using namespace std;
using namespace cv;

DepthAISensor::DepthAISensor() {
}

DepthAISensor::~DepthAISensor() {
    stop();
}

void DepthAISensor::start() {
    initDevice();

    isRunning = true;

    imuThread = thread([this]() {
        imuLoop();
    });

    cameraThread = thread([this]() {
        cameraLoop();
    });

}

void DepthAISensor::stop() {
    isRunning = false;

    if (imuThread.joinable())  {
        imuThread.join();
    }
    if (cameraThread.joinable()) {
        cameraThread.join();
    }

    if (!device->isClosed()) {
        device->close();
    }
}

void DepthAISensor::printCalibrationInfo(int width, int height) {
    cout << "---" << endl;
    auto socketToString = [](dai::CameraBoardSocket s) {
        switch(s) {
            case dai::CameraBoardSocket::CAM_A: return "CAM_A (center)";
            case dai::CameraBoardSocket::CAM_B: return "CAM_B (left  )" ;
            case dai::CameraBoardSocket::CAM_C: return "CAM_C (right )";
            default: return "UNKNOWN";
        }
    };
    unordered_map<dai::CameraBoardSocket, string> sensors = device->getCameraSensorNames();
    for(const auto& [socket, name] : sensors) {
        cout << "[OAK] Camera socket: " << socketToString(socket)
            << ", sensor: " << name << endl;
    }

    auto calibHandler = device->readCalibration();
    auto boardName = calibHandler.getEepromData().boardName;

    cout << "width x height : " << width << " x " << height << endl; 
    cout << "[Intrinsic]" << endl;
    for(const auto& [socket, name] : sensors) {
        cout << socketToString(socket) << endl;
        vector<vector<float>> intrinsic = calibHandler.getCameraIntrinsics(socket, width, height);
        for (const auto& row : intrinsic) {
            for (const auto& val : row) {
                cout << val << " ";
            }
            cout << endl;
        }    
    }

    cout << "[Distortion]" << endl;
    for(const auto& [socket, name] : sensors) {
        cout << socketToString(socket) << endl;
        vector<float> distortion = calibHandler.getDistortionCoefficients(socket);
        for (const auto& val : distortion) {
            cout << val << " ";
        }
        cout << endl;    
    }

    vector<vector<float>> extrinsics = calibHandler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);
    cout << "[Extrinsics CAM_B -> CAM_C]\n";
    for (const auto& row : extrinsics) {
        for (const auto& val : row) {
            cout << val << " ";
        }
        cout << endl;
    }
    cout << "---" << endl;
}

Mat DepthAISensor::visualizeDepth(const Mat& depth16U, int minDisplayMM, int maxDisplayMM) {
    CV_Assert(depth16U.type() == CV_16UC1);

    Mat validMask = (depth16U >= minDisplayMM) & (depth16U <= maxDisplayMM);

    Mat clipped;
    min(depth16U, maxDisplayMM, clipped);
    max(clipped, minDisplayMM, clipped);

    Mat normalized;
    clipped.convertTo(normalized, CV_32F);

    normalized = (normalized - minDisplayMM) / (maxDisplayMM - minDisplayMM);
    normalized = normalized * 255.0;

    normalized.convertTo(normalized, CV_8U);

    Mat depthColored;
    applyColorMap(normalized, depthColored, cv::COLORMAP_JET);
    depthColored.setTo(cv::Scalar(0,0,0), ~validMask);

    return depthColored;
}

void DepthAISensor::initDevice() {
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
    this->device = make_shared<dai::Device>(pipeline);

    vector<string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    cout << "[OAK] Device USB status : " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << endl;
    cout << "[OAK] Product name : " << device->getProductName() << endl;
    cout << "---" << endl;
}

dai::Pipeline DepthAISensor::createPipeline() {
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

void DepthAISensor::imuLoop() {
    shared_ptr<dai::DataOutputQueue> imuQueue = device->getOutputQueue("imu", 30, false);
    while(isRunning) {
        auto imuData = imuQueue->get<dai::IMUData>();
        if(imuData == nullptr)  {
            continue;
        }
        #if DEBUGMODE
        imuFps.update("imu");
        #endif

        for(const auto& imuPacket : imuData->packets) {
            auto accel = imuPacket.acceleroMeter;
            auto gyro = imuPacket.gyroscope;

            auto acceleroTs = accel.getTimestamp();
            auto gyroTs = gyro.getTimestamp();

            ImuData imu;
            imu.timestamp = chrono::duration<double>(acceleroTs.time_since_epoch()).count();
            imu.accel = cv::Vec3f(accel.x, accel.y, accel.z);
            imu.gyro  = cv::Vec3f(gyro.x, gyro.y, gyro.z);

            {
                lock_guard<mutex> lock(imuMutex);
                imuBuffer.push_back(imu);

                double MAX_IMU_TIME = 2.0;
                while(!imuBuffer.empty()) {
                    if (imu.timestamp - imuBuffer.front().timestamp > MAX_IMU_TIME) {
                        imuBuffer.pop_front();
                    } else {
                        break;
                    }
                }
            }

            if(imuCallback) {
                imuCallback(imu);
            }

            // Print accelerometer data
            //cout << "Accelerometer timestamp: " << acceleroTs.time_since_epoch().count() << endl;
            //cout << "Latency [ms]: " << timeDeltaToMilliS(chrono::steady_clock::now() - acceleroValues.getTimestamp()) << endl;
            //cout << "Accelerometer [m/s^2]: x: " << accel.x << " y: " << accel.y << " z: " << accel.z << endl;

            // Print gyroscope data
            //cout << "Gyroscope timestamp: " << gyroTs.time_since_epoch().count() << endl;
            //cout << "Gyroscope [rad/s]: x: " << gyro.x << " y: " << gyro.y << " z: " << gyro.z << endl;
        }
    }
}

void DepthAISensor::cameraLoop() {
    shared_ptr<dai::DataOutputQueue> syncQueue = device->getOutputQueue("sync_out", 30, false);
    while(isRunning) {
        auto messageGroup = syncQueue->get<dai::MessageGroup>();
        if(messageGroup == nullptr) {
            continue;
        }
        #if DEBUGMODE
        cameraFps.update("camera");
        #endif

        auto frameRgb = messageGroup->get<dai::ImgFrame>("sync_rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("sync_depth");
        auto frameLeft = messageGroup->get<dai::ImgFrame>("sync_left");
        auto frameRight = messageGroup->get<dai::ImgFrame>("sync_right");

        Mat distortRGB = frameRgb->getCvFrame();
        if (!isBuildingUndistortedRGBMap) {
            auto calibHandler = device->readCalibration();
            auto rgbIntrinsics = calibHandler.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, distortRGB.cols, distortRGB.rows);
            auto rgbDistortion = calibHandler.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);

            Mat K(3, 3, CV_32F);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    K.at<float>(i, j) = rgbIntrinsics[i][j];
                }
            }
                    
            Mat D(rgbDistortion.size(), 1, CV_32F);
            for (int i = 0; i < rgbDistortion.size(); i++) {
                D.at<float>(i, 0) = rgbDistortion[i];
            }
            initUndistortRectifyMap(K, D, Mat(), K, Size(distortRGB.cols, distortRGB.rows), CV_32FC1, mapX, mapY); //using CV_16S2 is not good here. 
            isBuildingUndistortedRGBMap = true;
        }
        Mat undistortRGB;
        remap(distortRGB, undistortRGB, mapX, mapY, INTER_NEAREST);


        Frame f;
        f.timestamp = chrono::duration<double>(frameLeft->getTimestamp().time_since_epoch()).count();
        f.left = frameLeft->getCvFrame().clone();
        f.right = frameRight->getCvFrame().clone();
        f.depth = frameDepth->getCvFrame().clone();
        f.rgb = undistortRGB.clone();

        {
            lock_guard<mutex> lock(frameMutex);
            frameBuffer.push_back(f);

            const int MAX_IMG_SIZE = 3;
            while(frameBuffer.size() > MAX_IMG_SIZE) {
                frameBuffer.pop_front();
            }
        }

        condVar.notify_one(); 

        if(frameCallback) {
            frameCallback(f);
        }
    }

}

bool DepthAISensor::getSensorData(DepthAISensor::SensorData & outData) {
    static double prvsTime = -1;
    unique_lock<mutex> lock(frameMutex);
    condVar.wait(lock, [this] { return !frameBuffer.empty() || !isRunning; });

    if (!isRunning) {
        return false;
    }

    outData.frame = frameBuffer.front();
    frameBuffer.pop_front();
    double currentTime = outData.frame.timestamp;
    lock.unlock();
    
    if (prvsTime < 0) {
        prvsTime = currentTime;
        return false;
    }

    outData.imuSequence.clear();
    {
        lock_guard<mutex> imuLock(imuMutex);
        auto it = imuBuffer.begin();
        while (it != imuBuffer.end()) {
            if (prvsTime <= it->timestamp && it->timestamp <= currentTime) {
                outData.imuSequence.push_back(*it);
            } 
            it++;
        }
    }
    prvsTime = currentTime;
    return true;
}
