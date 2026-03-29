#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"

using namespace std;

int main(int argc, char** argv) {


    DepthAISensor depthAISensor;
    depthAISensor.start();

    cout << "Press ENTER to stop..." << endl;
    cin.get();   // 等使用者輸入

    depthAISensor.stop();  

    cout << "Stopped." << endl;

    return 0;
}
