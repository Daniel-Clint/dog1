from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明可配置参数
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='192.168.1.2',
        description='TCP server IP address'
    )
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='8888',
        description='TCP server port'
    )
    
    # 获取lidars1包的共享目录路径
    pkg_share = get_package_share_directory('lidars1')
    
    # 构建RViz配置文件路径
    rviz_config = os.path.join(pkg_share, 'image_view_ros2.rviz')
    
    # 节点配置
    tcp_client_node = Node(
        package='lidars1',
        executable='lidars1',
        name='lidars1',
        output='screen',
        parameters=[{
            'server_ip': LaunchConfiguration('server_ip'),
            'server_port': LaunchConfiguration('server_port')
        }]
    )
    
    # RViz可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        tcp_client_node,
        rviz_node
    ])