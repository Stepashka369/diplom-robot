from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # urdf_path = PathJoinSubstitution([
    #     FindPackageShare('robot'),
    #     'urdf',
    #     'robot.urdf'
    # ])

    # robot_controllers = os.path.join(
    #     get_package_share_directory("robot"),
    #     "config",
    #     "head_controllers.yaml"
    # )

    # robot_controllers = PathJoinSubstitution([
    #     FindPackageShare("robot"),
    #     "config",
    #     "head_controllers.yaml",
    # ])

    # robot_description = PathJoinSubstitution([
    #     FindPackageShare("robot"),
    #     "urdf",
    #     "robot.urdf",
    # ])

    package_name='robot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')


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

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
    ])

    # return LaunchDescription([
    #     # запуск gazebo с пустым миров
    #     ExecuteProcess(
    #         cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
    #         output ='screen'
    #     ),
    #     # спавн робота
    #     Node(
    #         package = 'gazebo_ros',
    #         executable = 'spawn_entity.py',
    #         arguments = [
    #             '-entity', 'robot',
    #             '-file', urdf_path,
    #             '-x', '0.0', '-y', '0.0', '-z', '0.1',  # Приподнять над землей
    #             '-timeout', '30',
    #             '-robot_namespace', '/'
    #         ],
    #         output = 'screen'
    #     ),
        
        # Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[robot_controllers],
        #     output="both",
        # ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["joint_state_broadcaster"],
        # ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["forward_position_controller", "--param-file", robot_controllers],
        # ),
    # ])