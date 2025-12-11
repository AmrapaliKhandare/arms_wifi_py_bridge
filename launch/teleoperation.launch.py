from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    leader_ip_arg = DeclareLaunchArgument(
        'leader_ip',
        default_value='192.168.1.2',
        description='IP address of the leader arm'
    )
    
    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip',
        default_value='192.168.55.3',
        description='IP address of the follower arm'
    )
    
    force_feedback_gain_arg = DeclareLaunchArgument(
        'force_feedback_gain',
        default_value='0.1',
        description='Gain for force feedback from follower to leader'
    )
    
    teleoperation_time_arg = DeclareLaunchArgument(
        'teleoperation_time',
        default_value='10.0',
        description='Duration of teleoperation in seconds'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )
    
    # Create nodes
    leader_node = Node(
        package='arms_wifi_py_bridge',
        executable='leader_node',
        name='leader_arm_node',
        output='screen',
        parameters=[{
            'server_ip': LaunchConfiguration('leader_ip'),
            'force_feedback_gain': LaunchConfiguration('force_feedback_gain'),
            'teleoperation_time': LaunchConfiguration('teleoperation_time'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    follower_node = Node(
        package='arms_wifi_py_bridge',
        executable='follower_node',
        name='follower_arm_node',
        output='screen',
        sigterm_timeout='30',
        parameters=[{
            'server_ip': LaunchConfiguration('follower_ip'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    return LaunchDescription([
        leader_ip_arg,
        follower_ip_arg,
        force_feedback_gain_arg,
        teleoperation_time_arg,
        publish_rate_arg,
        leader_node,
        follower_node,
    ])