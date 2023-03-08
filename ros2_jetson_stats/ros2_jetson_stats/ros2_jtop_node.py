# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import jtop

from jtop_services.srv import NVPModel, JetsonClocks, Fan

from ros2_jetson_stats.utils import (
    other_status,
    board_status,
    disk_status,
    cpu_status,
    fan_status,
    gpu_status,
    ram_status,
    swap_status,
    power_status,
    temp_status,
    emc_status)


class JTOPPublisher(Node):

    def __init__(self):
        super().__init__('jtop_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, 'diagnostics', 1)
        # Create Services
        self.fan_srv = self.create_service(Fan, '/jtop/fan', self.fan_service)
        self.nvpmodel_srv = self.create_service(NVPModel, '/jtop/nvpmodel', self.nvpmodel_service)
        self.jetson_clocks_srv = self.create_service(JetsonClocks, '/jtop/jetson_clocks', self.jetson_clocks_service)
        self.declare_parameter('interval', 0.5) # Default interval 0.5
        self.declare_parameter('level_error', 60) # Default interval 0.5
        self.declare_parameter('level_warning', 40) # Default interval 0.5
        self.declare_parameter('level_ok', 20) # Default interval 0.5

        self.level_options = {
                self.get_parameter('level_error')._value: DiagnosticStatus.ERROR,
                self.get_parameter('level_warning')._value: DiagnosticStatus.WARN,
                self.get_parameter('level_ok')._value: DiagnosticStatus.OK,
                }

        timer_period = self.get_parameter('interval')._value
        self.timer = self.create_timer(timer_period, self.jetson_callback)
        self.i = 0
        self.jetson = jtop.jtop(interval=0.5)
        self.arr = DiagnosticArray()
        self.get_logger().info("Jetson Stats has started with interval : {}\n You can run following:\n  1. $ros2 run rqt_topic rqt_topic \n  2. Services for controlling fan_speed, power_mode, jetson_clocks\n".format(timer_period))

    def start(self):
        self.jetson.start()
        # Extract board information
        board = self.jetson.board
        # Define hardware name
        self.hardware = board["platform"]["Machine"]
        self.board_status = board_status(self.hardware, board, 'board')
        # Set callback
        #self.jetson.attach(self.jetson_callback)
    

    def fan_service(self, req, resp):
        # Try to set new nvpmodel
        fan_mode = req.mode
        fan_speed = req.speed
        try:
            self.jetson.fan.mode = fan_mode
            self.jetson.fan.speed = fan_speed
        except jtop.JtopException as e:
            # Return same nvp model
            fan_mode = str(self.jetson.fan.mode)
            fan_speed = self.jetson.fan.speed

        while self.jetson.ok():
            if self.jetson.fan.speed == fan_speed:
                break

        resp.set_fan_mode = fan_mode
        resp.set_fan_speed = fan_speed
        
        self.get_logger().info("Incoming Request \n Fan Mode:{}\t Fan Speed:{};\n Current Status of Fan Mode{}\t Fan Speed:{}".format(req.mode, req.speed, resp.set_fan_mode, resp.set_fan_speed))

        return resp

    def jetson_clocks_service(self, req, resp):
        # Set new jetson_clocks
        self.jetson.jetson_clocks = req.status

        resp.done = req.status
        self.get_logger().info("Incoming Request \n Jetson Clocks:{};\n Current Status of Jetson Clocks:{}".format(req.status, resp.done))
        return resp

    def nvpmodel_service(self, req, resp):
        # Try to set new nvpmodel
        cur_nvpmodel = self.jetson.nvpmodel
        self.get_logger().info("Incoming Request \n NVPModel:{};\n Current:{}".format(req.nvpmodel, cur_nvpmodel))
        nvpmodel = req.nvpmodel

        try:
            self.jetson.nvpmodel = nvpmodel
        except jtop.JtopException as e:
            # rospy.logerr(e)
            # Return same nvp model
            nvpmodel = self.jetson.nvpmodel

        while self.jetson.ok():
            if self.jetson.nvpmodel.id == nvpmodel:
                break

        resp.power_mode = str(self.jetson.nvpmodel)

        return resp

    def jetson_callback(self):
        # Add timestamp
        self.arr.header.stamp = self.get_clock().now().to_msg()
        # Status board and board info
        self.arr.status = [other_status(self.hardware, self.jetson, jtop.__version__)]
        # Make diagnostic message for each cpu
        self.arr.status += [cpu_status(self.hardware, name, self.jetson.cpu[name]) for name in self.jetson.cpu]
        # Merge all other diagnostics
        self.arr.status += [gpu_status(self.hardware, self.jetson.gpu[1])]
        self.arr.status += [ram_status(self.hardware, self.jetson.ram, 'mem')]
        self.arr.status += [swap_status(self.hardware, self.jetson.swap, 'mem')]
        self.arr.status += [emc_status(self.hardware, self.jetson.emc, 'mem')]
        # Temperature
        self.arr.status += [temp_status(self.hardware, self.jetson.temperature, self.level_options)]
        # Read power
        total, power = self.jetson.power
        if power:
            self.arr.status += [power_status(self.hardware, total, power)]
        # Fan controller
        if self.jetson.fan is not None:
            self.arr.status += [fan_status(self.hardware, self.jetson.fan, 'board')]
        # Status board and board info
        self.arr.status += [self.board_status]
        # Add disk status
        self.arr.status += [disk_status(self.hardware, self.jetson.disk, 'board')]
        # Update status jtop
        # rospy.logdebug("jtop message %s" % rospy.get_time())
        self.publisher_.publish(self.arr)



def main(args=None):
    rclpy.init(args=args)

    jtop_publisher = JTOPPublisher()
    jtop_publisher.start()

    rclpy.spin(jtop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    jtop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

