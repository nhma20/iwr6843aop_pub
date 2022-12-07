from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

username = os.path.expanduser('~')

def generate_launch_description():

    mmwave = Node(
        package="iwr6843aop_pub",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': username+'/ros2_ws/src/iwr6843aop_pub/cfg_files/xwr68xx_profile_30Hz.cfg'},
            {'cli_port': '/dev/ttyUSB0'},
            {'data_port': '/dev/ttyUSB1'}
         ]
    )


    return LaunchDescription([

        mmwave

    ])
