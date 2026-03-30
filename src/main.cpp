#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    DepthAISensor depthAISensor;
    depthAISensor.start();

    DepthAISensor::SensorData data;
    while(true) {
        int key = waitKey(1);
        if (key == 'q') {
            depthAISensor.stop();
            break;  
        } else if ( key == 's') {
            depthAISensor.printCalibrationInfo(640, 480);
        }

        if(!depthAISensor.getSensorData(data)) {
            continue;
        }
        imshow("left",data.frame.left);
    }

    return 0;
}
