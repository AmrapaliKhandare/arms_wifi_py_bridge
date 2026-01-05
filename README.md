# arms_wifi_py_bridge

ROS 2–based Wi-Fi bridge for leader–follower teleoperation between two Trossen arms.

This repository provides a teleoperation layer that enables a **leader arm** running on a
ground station to control a **follower arm** mounted on a robot (ATMOS) using ROS 2
communication over a network.

---

## Firewall Configuration (UFW)
Check firewall status using:
```bash
sudo ufw status
```
If the firewall (ufw) is disabled, no additional configuration is required.
If the firewall is enabled, run the following on each machine:
```bash
sudo ufw allow from <ip-of-other-machine>
```
## ROS 2 Network Configuration
For reliable ROS 2 communication across machines, the following configuration is required.

ROS Distribution
* Both systems must use ROS 2 Humble.

ROS Domain ID
* Both systems must use the same ROS_DOMAIN_ID:

```bash
export ROS_DOMAIN_ID=<domain_id>
```

RMW Implementation
* Both systems must use the same RMW implementation:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Installation
Clone the repository into an existing ROS 2 workspace and build:
```bash
cd ~/ros2_ws/src
git clone https://github.com/AmrapaliKhandare/arms_wifi_py_bridge.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Launch Files

This repository provides three launch options depending on whether the leader and follower
run on the same machine or on different machines.

### Same Machine (Leader and Follower on One System)

Use this when both arms (or both endpoints) are connected to the same computer:

```bash
ros2 launch arms_wifi_py_bridge teleoperation.launch.py
```

This launch file starts both:
* Leader node
* Follower node

## Different Machines (Ground Station and Robot)
Use this configuration when the leader and follower are running on separate systems.
On the leader machine (ground station):
```bash
ros2 launch arms_wifi_py_bridge leader.launch.py
```
On the follower machine (robot / ATMOS):
```bash
ros2 launch arms_wifi_py_bridge follower.launch.py
```
## Usage Notes
* Ensure both arms are powered on and drivers/controllers are running before launching teleoperation nodes.
* Verify ROS 2 topic discovery across machines before commanding motion.
* Start with both arms in a safe configuration.

## Author
Developed by Amrapali Khandare  
Riviere Robot Lab, NYU
