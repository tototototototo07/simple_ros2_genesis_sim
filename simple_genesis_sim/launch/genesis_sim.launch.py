
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description(): 

    arg_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to model file'
    )

    arg_param_file = DeclareLaunchArgument(
        'param_file',
        default_value='',
        description='Path to the parameter file'
    )

    arg_use_gpu = DeclareLaunchArgument(
        'use_gpu', 
        default_value='false',
        description='simulate with gpu'
    )

    arg_recording = DeclareLaunchArgument(
        'recording',
        default_value='false',
        description='Record video or not'
    )

    arg_put_objects = DeclareLaunchArgument(
        'put_objects',
        default_value='false',
        description='Put objects in the environment'
    )

    env_objects_dir = PathJoinSubstitution([FindPackageShare('simple_genesis_sim'), 'env_objects'])

    genesis_sim_node = Node(
        package='simple_genesis_sim',
        executable='genesis_sim',
        output='screen',
        parameters=[LaunchConfiguration('param_file'),
                    {'model_path' : LaunchConfiguration('model_path')},
                    {'use_gpu'    : LaunchConfiguration('use_gpu')},
                    {'recording'  : LaunchConfiguration('recording')},
                    {'put_objects': LaunchConfiguration('put_objects')},
                    {'env_objects_dir': env_objects_dir},
                    {'use_sim_time': True}],
    )

    stop_recording_gui_node = Node(
        package='simple_genesis_sim',
        executable='stop_recording_gui',
        condition=IfCondition(LaunchConfiguration('recording')),
    )
    
    return LaunchDescription([
        arg_model_path,
        arg_param_file,
        arg_use_gpu,
        arg_recording,
        arg_put_objects,
        genesis_sim_node,
        stop_recording_gui_node,
    ])