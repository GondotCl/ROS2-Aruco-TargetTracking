from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess


def generate_launch_description():

    params = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_zed_camera.yaml')
    
    for arg in sys.argv:
        if arg.startswith("project:="):
            project = arg.split(":=")[1]
            params = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_' + project + '.yaml')

    # Camera model (force value)
    camera_model = 'zedm'

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model
        }.items()
    )

    # static transformation
    zed_camera_frame = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 zedm_left_camera_optical_frame camera_link'
        ]],
        shell=True
    )

    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'zedm_left_camera_optical_frame', 'camera_link']
    # )

    return LaunchDescription([
        zed_camera_frame,

        zed_wrapper_launch,


        Node(
            package='camera_target_tracking',
            executable='aruco_detector',
            name='aruco_detector',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params],
        )
])
