#ifndef FPSCounter_hpp
#define FPSCounter_hpp

#include <iostream>

class FPSCounter {
public:
    void update(const std::string& name) {
        frameCount++;
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;

        if (elapsed.count() >= 1.0) {
            std::cout << "[" << name << "] FPS: " << frameCount << std::endl;
            frameCount = 0;
            lastTime = currentTime;
        }
    }
private:
    std::chrono::high_resolution_clock::time_point lastTime = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
};

#endif