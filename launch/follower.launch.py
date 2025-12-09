from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for follower arm on Computer B"""
    
    # Declare launch arguments
    follower_ip_arg = DeclareLaunchArgument(
        'follower_ip',
        default_value='192.168.55.3',
        description='IP address of the follower arm'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )
    
    # Create follower node
    follower_node = Node(
        package='arms_wifi_py_bridge',
        executable='follower_node',
        name='follower_arm_node',
        output='screen',
        parameters=[{
            'server_ip': LaunchConfiguration('follower_ip'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    return LaunchDescription([
        follower_ip_arg,
        publish_rate_arg,
        follower_node,
    ])