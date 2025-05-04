import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():    

    packege_name: str = 'example_sim'
    model_path = PathJoinSubstitution([FindPackageShare(packege_name), 'config', 'model', 'urdf', 'example_car.urdf'])
    param_file_path = PathJoinSubstitution([FindPackageShare(packege_name), 'config', 'parameter.yaml'])


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

    sim_launch_file = PathJoinSubstitution([FindPackageShare('simple_genesis_sim'), 'launch', 'genesis_sim.launch.py'])



    pub_joint_node = Node(
        package=packege_name,
        executable='pub_joint',
        output='screen',
        parameters=[param_file_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        arg_use_gpu,
        arg_recording,
        arg_put_objects,
        pub_joint_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file),
            launch_arguments={
                'model_path' : model_path,
                'param_file' : param_file_path, 
                'use_gpu'    : LaunchConfiguration('use_gpu'),
                'recording'  : LaunchConfiguration('recording'),
                'put_objects': LaunchConfiguration('put_objects'),
            }.items()
        )
    ])

