# ros2_jetson_stats

A ROS2 wrapper [jetson-stats](https://github.com/rbonghi/jetson_stats) to ROS where you can read the status of your board via diagnostic messages

## Reference
* [Implementing robotics applications with ROS2 and AI on Jetson platforms](https://developer.nvidia.com/blog/implementing-robotics-applications-with-ros-2-and-ai-on-jetson-platform-2/)
* This repository takes inspiration from [ros_jetson_stats](https://github.com/rbonghi/ros_jetson_stats) and creates package for ros2


# Install
Follow the ROS2 [Install Guide](https://index.ros.org/doc/ros2/Installation/foxy/Linux-Development-Setup/)

## Set up
1. Install jetson_stats:

```
sudo -H pip install -U jetson-stats
```

2. Clone repo in ros2 workspace
``` 
mkdir -p $HOME/dev_ws/src
cd $HOME/dev_ws/src
git clone https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats.git
cd $HOME/dev_ws
```

3. Build
```
sudo rosdep install --from-paths $HOME/dev_ws --ignore-src --rosdistro foxy -y
colcon build --symlink-install
. install/setup.bash
```
