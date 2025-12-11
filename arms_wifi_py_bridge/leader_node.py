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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # UDP-like (drops packets if needed)
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Only keep the SINGLE latest message
        )
        
        # Publishers and Subscribers
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/leader/joint_positions', 
            qos_profile
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 
            '/leader/joint_velocities', 
            qos_profile
        )
        self.effort_sub = self.create_subscription(
            Float64MultiArray,
            '/follower/external_efforts',
            self.effort_callback,
            qos_profile
        )
        
        # State variables
        self.teleop_active = False
        self.start_time = None
        self.last_effort_time = None
        self.waiting_for_follower = True
        self.shutdown_requested = False  # Flag to prevent restart after emergency
        
        self.stopping_sequence_start = None

        # Move to home position
        self.move_to_home()
        
        # Create timer for publishing leader state
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info('Leader arm node initialized and ready!')
    
    def emergency_stop(self):
        """Emergency stop - immediately go to sleep position"""
        self.get_logger().error('EMERGENCY STOP - Moving to sleep position')
        self.teleop_active = False
        self.shutdown_requested = True  # Prevent any restart
        try:
            self.driver.set_all_modes(trossen_arm.Mode.position)
            self.driver.set_all_positions(
                np.zeros(self.driver.get_num_joints()),
                2.0,
                True
            )
            self.get_logger().info('Leader at sleep position')
        except Exception as e:
            self.get_logger().error(f'Failed to move to sleep: {e}')
    
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
            try:
                self.driver.set_all_modes(trossen_arm.Mode.external_effort)
                self.teleop_active = True
                self.start_time = self.get_clock().now()
                self.waiting_for_follower = False
            except Exception as e:
                self.get_logger().error(f'Failed to enter external_effort mode: {e}')
                self.emergency_stop()
                raise
    
    def check_follower_connection(self):
        """Check if we're receiving data from follower"""
        if self.last_effort_time is None:
            return False
        
        time_since_effort = (self.get_clock().now() - self.last_effort_time).nanoseconds / 1e9
        return time_since_effort < 2.0  # Connection lost if no data for 2 seconds
    
    def stop_teleoperation(self):
        """Stop teleoperation mode"""
        if not self.shutdown_requested:
            self.get_logger().info('Shutting down hardware...')
            self.shutdown_requested = True
            self.move_to_home()
            self.move_to_sleep()
    
    def timer_callback(self):
        """Publish leader arm state at fixed rate"""
        # Check if shutdown was requested (emergency stop or timeout)
        if self.shutdown_requested:
            return  # Stop all operations permanently
        
        # --- PHASE 3: CHECK FOR DELAY COMPLETION ---
        if self.stopping_sequence_start is not None:
            time_since_stop = (self.get_clock().now() - self.stopping_sequence_start).nanoseconds / 1e9
            if time_since_stop >= 2.0: # Wait 2 seconds
                self.get_logger().info('Graceful shutdown delay complete.')
                self.stop_teleoperation() # Physical move
            return # Don't do anything else while waiting

        # --- WAITING FOR CONNECTION ---
        if not self.teleop_active:
            if self.waiting_for_follower:
                self.get_logger().info('Waiting for follower node...', throttle_duration_sec=2.0)
                if self.check_follower_connection():
                    self.get_logger().info('Follower detected! Starting teleoperation.')
                    try:
                        self.start_teleoperation()
                    except Exception as e:
                        self.get_logger().error(f'Failed to start teleop: {e}')
                        self.emergency_stop()
            return

        # --- PHASE 1: CHECK TIMER ---
        if self.teleop_active and self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= self.teleoperation_time:
                self.get_logger().info('Timer finished. Stopping data transmission (Graceful Shutdown).')
                self.teleop_active = False # Stop publishing IMMEDIATELY
                self.stopping_sequence_start = self.get_clock().now() # Start the delay timer
                return

        # Watchdog: Check if follower is still connected
        if self.teleop_active and not self.check_follower_connection():
            self.get_logger().error('Lost connection to follower! Emergency stop.')
            self.emergency_stop()
            return
        
        # --- PHASE 2: PUBLISH DATA ---
        if self.teleop_active:
            try:
                # Get current positions and velocities
                positions = self.driver.get_all_positions()
                velocities = self.driver.get_all_velocities()
                
                # Convert VectorDouble to list
                positions_list = [positions[i] for i in range(len(positions))]
                velocities_list = [velocities[i] for i in range(len(velocities))]
                
                # SAFETY CHECK: Validate positions before publishing
                max_angle = 6.28  # ~2π radians
                if any(abs(p) > max_angle for p in positions_list):
                    self.get_logger().error(f'Leader position out of bounds: {positions_list}')
                    self.emergency_stop()
                    return
                
                # Publish positions
                pos_msg = Float64MultiArray()
                pos_msg.data = positions_list
                self.position_pub.publish(pos_msg)
                
                # Publish velocities
                vel_msg = Float64MultiArray()
                vel_msg.data = velocities_list
                self.velocity_pub.publish(vel_msg)
            except Exception as e:
                self.get_logger().error(f'Error in main loop: {e}')
                self.emergency_stop()
    
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
            self.emergency_stop()
        super().destroy_node()


import time  # <--- MAKE SURE THIS IS IMPORTED

def main(args=None):
    rclpy.init(args=args)
    node = LeaderArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:

        if node.shutdown_requested:
            node.get_logger().info('Clean exit (already shut down).')
        else:
            node.get_logger().info('Ctrl+C detected! initiating graceful shutdown...')
        
            # 1. Stop sending data IMMEDIATELY
            node.teleop_active = False
            node.shutdown_requested = True
            
            # 2. WAIT for Follower to park (The Graceful Delay)
            node.get_logger().info('Waiting 2.0s for follower to park...')
            time.sleep(2.0) 
            
            # 3. Park Leader (Home -> Sleep)
            node.get_logger().info('Parking leader arm...')
            try:
                node.move_to_home()
                node.move_to_sleep()
            except Exception as e:
                node.get_logger().error(f'Error during park: {e}')
            
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()