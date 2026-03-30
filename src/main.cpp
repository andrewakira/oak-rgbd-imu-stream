#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    Mat frame;
    bool debugAddress = false;
    int viewMode = 4; 


    DepthAISensor depthAISensor;
    depthAISensor.start();

    DepthAISensor::SensorData data;
    while(true) {
        if(!depthAISensor.getSensorData(data)) {
            waitKey(5);
            continue;
        }

        switch(viewMode) {
            case 1: frame = data.frame.left; break;
            case 2: frame = data.frame.right; break;
            case 3: frame = data.frame.depth; break;
            case 4: frame = data.frame.rgb; break;
            default: frame.release(); break;
        }

        if (!frame.empty()) {
            if (debugAddress) {
                cout << "type : " << frame.type() << endl;
                cout << "address : " << (void*)frame.data << endl;
            }
            if (viewMode == 3 ){
                frame = depthAISensor.visualizeDepth(frame);
            }
            imshow("frame", frame);
        }

        int key = waitKey(1);
        if (key == 'q') {
            depthAISensor.stop();
            break;  
        } else if (key == 'a') {
            depthAISensor.printCalibrationInfo(640, 480);
        } else if (key == 's') {
            if(!frame.empty()) {
                imwrite("frame.png", frame);
            }
        } else if (key == 'd') {
            debugAddress = !debugAddress;
        } else if ( key == '1') {
            viewMode = 1;    
        } else if ( key == '2') {
            viewMode = 2;     
        } else if (key == '3') {
            viewMode = 3;   
        } else if (key == '4') {
            viewMode = 4;           
        }
    }

    return 0;
}
