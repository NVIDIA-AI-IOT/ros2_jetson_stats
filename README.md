# ROS 2 jetson_stats
[NVIDIA Developer Blog](https://developer.nvidia.com/blog/implementing-robotics-applications-with-ros-2-and-ai-on-jetson-platform-2/)

## ROS2 wrapper for Jetson Stats (jtop)
This repository takes inspiration from [ROS-jtop](https://github.com/rbonghi/ros_jetson_stats) and creates package for ros2 (currently tested on eloquent)

## Pre-requisite
ROS2 Eloquent [Install Guide](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/)

## Set up
1. Install Jetson Stats: <br/>
```sudo -H pip install -U jetson-stats```
2. Clone repo in ros2 workspace
``` cd dev_ws/src``` <br/>
``` git clone https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats.git``` <br/>
3. Build
``` sudo rosdep install --from-paths ros2_jtop --ignore-src --rosdistro eloquent -y ``` <br/>
``` colcon build ``` <br/>
``` . install/setup.bash ```<br/>

### Usage

The basic usage consists of creating `diagnostics_msgs` and services for control fan mode/speed, setting up `jetson_clocks`, and power mode.

1. Run JTOP node: <br/>
``` ros2 run ros2_jetson_stats ros2_jtop ```<br/>
If you want to change frequency of JTOP messages, do following:<br/>
``` ros2 run ros2_jetson_stats ros2_jtop --ros-args -p interval:=0.2```<br/>
*Note: JTOP frequency is from 0 - 1 range*

2. You can see diagnostic messages using following command:
``` sudo . install/setup.bash ``` <br/>
``` sudo . install/local_setup.bash ``` <br/>
``` ros2 run rqt_topic rqt_topic ``` <br/>
*Note: for default interval you will see message frequency ~2Hz if you change interval to 0.2 the message frequency will change to 6Hz*

3. To run JTOP services:<br/>
    You will need to keep running ```ros2_jetson_stats``` ; as shown in step 1. <br/>
    A. Change Power Model:<br/>
    Open new terminal <br/>
    ``` sudo . install/local_setup.bash ``` <br/>
    ```ros2 service call /jtop/nvpmodel jtop_services/srv/NVPModel "{nvpmodel: 0}"``` <br/>

    This will change power mode to ``` 15 W 2 Core ``` on Jetson Xavier NX
    Following Message will appear on the terminal: <br/>
    ```

    requester: making request: jtop_services.srv.NVPModel_Request(nvpmodel=2)

    response:
    jtop_services.srv.NVPModel_Response(power_mode='MODE_15W_2CORE')

    ```

    B. Change Fan mode and Fan Speed:<br/>
    Open new terminal <br/>
    ``` sudo . install/local_setup.bash ``` <br/>
    ```ros2 service call /jtop/fan jtop_services/srv/Fan "{mode: system, speed: 100}"``` <br/>

    This will change fan speed to 100; you can see Jetson Fan being turned on.

    ```

    requester: making request: jtop_services.srv.Fan_Request(mode='system', speed=100)

    response:
    jtop_services.srv.Fan_Response(set_fan_mode='system', set_fan_speed=100)
    ```

    C. Start Jetson Clocks:
    Open new terminal <br/>
    ``` sudo . install/local_setup.bash ``` <br/>
    ``` ros2 service call /jtop/jetson_clocks jtop_services/srv/JetsonClocks "{status: True}"``` <br/>

## Other related ROS 2 projects
- [ros2_torch_trt](https://github.com/NVIDIA-AI-IOT/ros2_torch_trt) : ROS2 Real Time Classification and Detection <br/>
- [ros2_deepstream](https://github.com/NVIDIA-AI-IOT/ros2_deepstream) : ROS2 nodes for DeepStream applications <br/>
- [ros2_trt_pose](https://github.com/NVIDIA-AI-IOT/ros2_trt_pose) : ROS 2 package for "trt_pose": real-time human pose estimation on NVIDIA Jetson Platform <br/>