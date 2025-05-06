import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg_path = get_package_share_directory('robot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # Нода robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # спавн робота
    node_robot_spawner = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = [
            '-entity', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',  # Приподнять над землей
            '-timeout', '30',
            '-robot_namespace', '/'
        ],
        output = 'screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Launch!
    return LaunchDescription([
        ExecuteProcess(
            cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
            output ='screen'
        ),
        node_robot_state_publisher,
        node_robot_spawner,
        diff_drive_spawner,
        joint_broad_spawner
    ])