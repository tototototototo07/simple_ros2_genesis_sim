import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():    



    param_file_path = PathJoinSubstitution([FindPackageShare('franka_sim'), 'config', 'parameter.yaml'])

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
    #sim_package_dir = get_package_share_directory('simple_genesis_sim')
    #sim_launch_file = os.path.join(sim_package_dir, 'launch', 'genesis_sim.launch.py')

    sim_launch_file = PathJoinSubstitution([FindPackageShare('simple_genesis_sim'), 'launch', 'genesis_sim.launch.py'])



    pub_joint_node = Node(
        package='franka_sim',
        executable='pub_joint',
        output='screen',
        parameters=[param_file_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        arg_use_gpu,
        arg_recording,
        pub_joint_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file),
            launch_arguments={
                'model_path' : 'xml/franka_emika_panda/panda.xml',
                'param_file' : param_file_path, 
                'use_gpu'    : LaunchConfiguration('use_gpu'),
                'recording'  : LaunchConfiguration('recording'),
            }.items()
        )
    ])

