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


class LeaderArmNode(Node):
    def __init__(self):
        super().__init__('leader_arm_node')
        
        # Declare parameters
        self.declare_parameter('server_ip', '192.168.1.2')
        self.declare_parameter('force_feedback_gain', 0.1)
        self.declare_parameter('teleoperation_time', 20.0)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        # Get parameters
        self.server_ip = self.get_parameter('server_ip').value
        self.force_feedback_gain = self.get_parameter('force_feedback_gain').value
        self.teleoperation_time = self.get_parameter('teleoperation_time').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info(f'Leader arm server IP: {self.server_ip}')
        self.get_logger().info(f'Force feedback gain: {self.force_feedback_gain}')
        
        # Initialize driver
        self.get_logger().info('Initializing leader arm driver...')
        self.driver = trossen_arm.TrossenArmDriver()
        self.driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.server_ip,
            False
        )
        
        # Publishers and Subscribers
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/leader/joint_positions', 
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 
            '/leader/joint_velocities', 
            10
        )
        self.effort_sub = self.create_subscription(
            Float64MultiArray,
            '/follower/external_efforts',
            self.effort_callback,
            10
        )
        
        # State variables
        self.teleop_active = False
        self.start_time = None
        self.last_effort_time = None
        self.waiting_for_follower = True
        
        # Move to home position
        self.move_to_home()
        
        # Create timer for publishing leader state
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info('Leader arm node initialized and ready!')
    
    def move_to_home(self):
        """Move the leader arm to home position"""
        self.get_logger().info('Moving leader to home position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0]),
            2.0,
            True
        )
        self.get_logger().info('Leader at home position')
    
    def move_to_sleep(self):
        """Move the leader arm to sleep position"""
        self.get_logger().info('Moving leader to sleep position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.zeros(self.driver.get_num_joints()),
            2.0,
            True
        )
        self.get_logger().info('Leader at sleep position')
    
    def start_teleoperation(self):
        """Start teleoperation mode"""
        if not self.teleop_active:
            self.get_logger().info('Starting teleoperation mode...')
            self.driver.set_all_modes(trossen_arm.Mode.external_effort)
            self.teleop_active = True
            self.start_time = self.get_clock().now()
            self.waiting_for_follower = False
    
    def check_follower_connection(self):
        """Check if we're receiving data from follower"""
        if self.last_effort_time is None:
            return False
        
        time_since_effort = (self.get_clock().now() - self.last_effort_time).nanoseconds / 1e9
        return time_since_effort < 2.0  # Connection lost if no data for 2 seconds
    
    def stop_teleoperation(self):
        """Stop teleoperation mode"""
        if self.teleop_active:
            self.get_logger().info('Stopping teleoperation mode...')
            self.teleop_active = False
            self.move_to_home()
            self.move_to_sleep()
    
    def timer_callback(self):
        """Publish leader arm state at fixed rate"""
        if not self.teleop_active:
            # Wait for follower before starting
            if self.waiting_for_follower:
                if self.last_effort_time is None:
                    self.get_logger().info('Waiting for follower node...', throttle_duration_sec=5.0)
                    return
                else:
                    self.get_logger().info('Follower detected! Starting teleoperation.')
            
            self.start_teleoperation()
        
        # Check if teleoperation time has expired
        if self.teleop_active and self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= self.teleoperation_time:
                self.stop_teleoperation()
                return
        
        # Watchdog: Check if follower is still connected
        if self.teleop_active and not self.check_follower_connection():
            self.get_logger().error('Lost connection to follower! Stopping teleoperation.')
            self.stop_teleoperation()
            return
        
        if self.teleop_active:
            # Get current positions and velocities
            positions = self.driver.get_all_positions()
            velocities = self.driver.get_all_velocities()
            
            # Convert VectorDouble to list
            positions_list = [positions[i] for i in range(len(positions))]
            velocities_list = [velocities[i] for i in range(len(velocities))]
            
            # Publish positions
            pos_msg = Float64MultiArray()
            pos_msg.data = positions_list
            self.position_pub.publish(pos_msg)
            
            # Publish velocities
            vel_msg = Float64MultiArray()
            vel_msg.data = velocities_list
            self.velocity_pub.publish(vel_msg)
    
    def effort_callback(self, msg):
        """Receive external efforts from follower and apply to leader"""
        # Update watchdog timer
        self.last_effort_time = self.get_clock().now()
        
        if self.teleop_active:
            efforts = np.array(msg.data)
            # Apply force feedback with gain
            self.driver.set_all_external_efforts(
                -self.force_feedback_gain * efforts,
                0.0,
                False
            )
    
    def destroy_node(self):
        """Cleanup before shutting down"""
        self.get_logger().info('Shutting down leader arm node...')
        if self.teleop_active:
            self.stop_teleoperation()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderArmNode()
    
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