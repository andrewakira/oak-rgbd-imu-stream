#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>

class DepthAISensorReceiver {
public:
    DepthAISensorReceiver() 
    :subRgb(nh, "/rgb/image_raw", 10), subLeft(nh, "/left/image_raw", 10), subRight(nh, "/right/image_raw", 10),
    subDepth(nh, "/depth/image_raw", 10), sync(SyncPolicy(10), subRgb, subLeft, subRight, subDepth) {
        sync.registerCallback(
            boost::bind(&DepthAISensorReceiver::imageCallback, this, _1, _2, _3, _4)
        );

        subImu = nh.subscribe("/imu", 1000, &DepthAISensorReceiver::imuCallback, this);
    }

private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> subRgb;
    message_filters::Subscriber<sensor_msgs::Image> subLeft;
    message_filters::Subscriber<sensor_msgs::Image> subRight;
    message_filters::Subscriber<sensor_msgs::Image> subDepth;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::Image,
        sensor_msgs::Image,
        sensor_msgs::Image
    > SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync;


    ros::Subscriber subImu;

    void imageCallback(
        const sensor_msgs::ImageConstPtr& rgbMsg,
        const sensor_msgs::ImageConstPtr& leftMsg,
        const sensor_msgs::ImageConstPtr& rightMsg,
        const sensor_msgs::ImageConstPtr& depthMsg) {
        try {
            double t = rgbMsg->header.stamp.toSec();

            cv::Mat rgb   = cv_bridge::toCvCopy(rgbMsg, "bgr8")->image;
            cv::Mat left  = cv_bridge::toCvCopy(leftMsg, "mono8")->image;
            cv::Mat right = cv_bridge::toCvCopy(rightMsg, "mono8")->image;
            cv::Mat depth = cv_bridge::toCvCopy(depthMsg, "16UC1")->image;

            ROS_INFO_STREAM_THROTTLE(1.0, "[SYNC IMG] t=" << t);

            cv::imshow("RGB", rgb);
            cv::imshow("Left", left);
            cv::imshow("Right", right);

            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        ROS_INFO_STREAM_THROTTLE(1.0,
            "[IMU] t=" << msg->header.stamp.toSec()
            << " gyro=("
            << msg->angular_velocity.x << ", "
            << msg->angular_velocity.y << ", "
            << msg->angular_velocity.z << ")"
        );
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "oak_receiver_node");

    DepthAISensorReceiver receiver;

    ros::spin();
    return 0;
}