## Introduction
The following table describes the published ROS1 topics and their corresponding message types and encodings:
| Sensor     | Type                | Encoding | Notes       |
| ---------- | ------------------- | -------- | ----------- |
| RGB        | `sensor_msgs/Image` | `bgr8`   | Undistorted |
| Left/Right | `sensor_msgs/Image` | `mono8`  | Rectified   |
| Depth      | `sensor_msgs/Image` | `16UC1`  | Unit: mm    |
| IMU        | `sensor_msgs/Imu`   | —        | 6-DoF       |   

## ROS1 docker setup
1. Setup workspace
    ```shell=
    mkdir -p oak_ros1/src/
    cd oak_ros1/src
    git clone https://github.com/andrewakira/oak-rgbd-imu-stream.git
    ```
2. Building image
    ```shell=
    cd oak-rgbd-imu-stream
    sudo docker build --network=host -t oak_ros1 -f dockerFile_ros1 .
    ```
3. Run Container
    ```shell=
    #run in oak_ros1 directory
    cd ../../  
    docker run -it --rm \
      --privileged \
      --net=host \
      --ipc=host \
      -e DISPLAY=$DISPLAY \
      -e XAUTHORITY=$XAUTHORITY \
      -v $XAUTHORITY:$XAUTHORITY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v $(pwd):/workspace \
      -v /dev:/dev \
      oak_ros1
    ```
4. Compile
    ```shell=
    cd /workspace
    source /opt/ros/noetic/setup.bash
    catkin build
    ```
5. Run
    ```shell=
    source devel/setup.bash
    roslaunch oak_ros oak_publisher.launch
    ```