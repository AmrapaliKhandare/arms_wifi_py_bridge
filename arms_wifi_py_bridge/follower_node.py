#!/usr/bin/env python3
# Copyright 2025 Trossen Robotics
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

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import trossen_arm


class FollowerArmNode(Node):
    def __init__(self):
        super().__init__('follower_arm_node')
        
        # Declare parameters
        self.declare_parameter('server_ip', '192.168.1.3')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        # Get parameters
        self.server_ip = self.get_parameter('server_ip').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info(f'Follower arm server IP: {self.server_ip}')
        
        # Initialize driver
        self.get_logger().info('Initializing follower arm driver...')
        self.driver = trossen_arm.TrossenArmDriver()
        self.driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            self.server_ip,
            False
        )
        
        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/follower/external_efforts',
            10
        )
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            '/leader/joint_positions',
            self.position_callback,
            10
        )
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/leader/joint_velocities',
            self.velocity_callback,
            10
        )
        
        # State variables
        self.teleop_active = False
        self.latest_velocities = None
        
        # Move to home position
        self.move_to_home()
        
        # Create timer for publishing follower efforts
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info('Follower arm node initialized and ready!')
    
    def move_to_home(self):
        """Move the follower arm to home position"""
        self.get_logger().info('Moving follower to home position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0]),
            2.0,
            True
        )
        self.get_logger().info('Follower at home position')
    
    def move_to_sleep(self):
        """Move the follower arm to sleep position"""
        self.get_logger().info('Moving follower to sleep position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.zeros(self.driver.get_num_joints()),
            2.0,
            True
        )
        self.get_logger().info('Follower at sleep position')
    
    def start_teleoperation(self):
        """Start teleoperation mode"""
        if not self.teleop_active:
            self.get_logger().info('Starting teleoperation mode...')
            self.driver.set_all_modes(trossen_arm.Mode.position)
            self.teleop_active = True
    
    def stop_teleoperation(self):
        """Stop teleoperation mode"""
        if self.teleop_active:
            self.get_logger().info('Stopping teleoperation mode...')
            self.teleop_active = False
            self.move_to_home()
            self.move_to_sleep()
    
    def timer_callback(self):
        """Publish follower external efforts at fixed rate"""
        # Always publish efforts (even when not teleoperating)
        # This allows leader to detect that follower is alive
        efforts = self.driver.get_all_external_efforts()
        
        # Convert VectorDouble to list
        efforts_list = [efforts[i] for i in range(len(efforts))]
        
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts_list
        self.effort_pub.publish(effort_msg)
    
    def position_callback(self, msg):
        """Receive positions from leader and apply to follower"""
        if not self.teleop_active:
            self.start_teleoperation()
        
        positions = np.array(msg.data)
        
        # Apply positions with velocities if available
        if self.latest_velocities is not None:
            self.driver.set_all_positions(
                positions,
                0.0,
                False,
                self.latest_velocities
            )
        else:
            self.driver.set_all_positions(
                positions,
                0.0,
                False
            )
    
    def velocity_callback(self, msg):
        """Store latest velocities from leader"""
        self.latest_velocities = np.array(msg.data)
    
    def destroy_node(self):
        """Cleanup before shutting down"""
        self.get_logger().info('Shutting down follower arm node...')
        if self.teleop_active:
            self.stop_teleoperation()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()