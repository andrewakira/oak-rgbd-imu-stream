#include "DepthAISensor.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>


class DepthAISensorPublisher {
public:
    DepthAISensorPublisher(DepthAISensor& sensor) : sensor(sensor){
        imuPub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

        leftPub  = nh.advertise<sensor_msgs::Image>("/left/image_raw", 10);
        rightPub = nh.advertise<sensor_msgs::Image>("/right/image_raw", 10);
        rgbPub   = nh.advertise<sensor_msgs::Image>("/rgb/image_raw", 10);
        depthPub = nh.advertise<sensor_msgs::Image>("/depth/image_raw", 10);

        sensor.setFrameCallback(
            [this](const DepthAISensor::Frame& f) {
                publishImage(f);
            }
        );

        sensor.setImuCallback(
            [this](const DepthAISensor::ImuData& imu) {
                publishImu(imu);
            }
        );
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imuPub;

    ros::Publisher leftPub;
    ros::Publisher rightPub;
    ros::Publisher rgbPub;
    ros::Publisher depthPub;

    DepthAISensor& sensor;

    void publishImu(const DepthAISensor::ImuData& imu) {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time(imu.timestamp);
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = imu.accel[0];
        msg.linear_acceleration.y = imu.accel[1];
        msg.linear_acceleration.z = imu.accel[2];

        msg.angular_velocity.x = imu.gyro[0];
        msg.angular_velocity.y = imu.gyro[1];
        msg.angular_velocity.z = imu.gyro[2];

        imuPub.publish(msg);
    }

    void publishImage(const DepthAISensor::Frame& f) {
        std_msgs::Header header;
        header.stamp = ros::Time(f.timestamp);

        if (!f.left.empty()) {
            header.frame_id = "left";
            auto msg = cv_bridge::CvImage(header, "mono8", f.left).toImageMsg();
            leftPub.publish(msg);
        }

        if (!f.right.empty()) {
            header.frame_id = "right";
            auto msg = cv_bridge::CvImage(header, "mono8", f.right).toImageMsg();
            rightPub.publish(msg);
        }

        if (!f.rgb.empty()) {
            header.frame_id = "rgb";
            auto msg = cv_bridge::CvImage(header, "bgr8", f.rgb).toImageMsg();
            rgbPub.publish(msg);
        }

        if (!f.depth.empty()) {
            header.frame_id = "depth";

            auto msg = cv_bridge::CvImage(header, "16UC1", f.depth).toImageMsg();
            depthPub.publish(msg);
        }
    }
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "oak_publisher");

    DepthAISensor sensor;
    DepthAISensorPublisher publisher(sensor);

    sensor.start();

    ros::spin();  

    sensor.stop();

    return 0;
}