# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# https://hub.docker.com/_/ros
FROM ros:foxy-ros-base

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# https://answers.ros.org/question/355702/colcon-test-failed-in-python-package/
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install pydocstyle==6.1.1 && \
    rm -rf /var/lib/apt/lists/*

# Download and build nanosaur_ws
ENV ROS_WS /opt/ros_ws

# Copy this package
COPY . ${ROS_WS}/src/ros2_jetson_stats

# Change workdir
WORKDIR $ROS_WS

# Build Isaac ROS
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# run ros package launch file
CMD ["ros2", "run", "ros2_jetson_stats", "jtop"]