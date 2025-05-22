# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from config.constants import Constants


# def generate_launch_description():
#     pkg_path = get_package_share_directory(Constants.PKG_NAME)
#     urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

#     with open(urdf_path, 'r') as f:
#         robot_description = f.read()
    
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': True,
#         }]
#     )

#     robot_spawner_node = Node(
#         package = 'gazebo_ros',
#         executable = 'spawn_entity.py',
#         arguments = [
#             '-entity', 'robot',
#             '-topic', 'robot_description',
#             '-x', '0.0', '-y', '0.0', '-z', '0.1',
#             '-timeout', '30',
#         ],
#         output = 'screen'
#     )

#     head_rotation_controller_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["head_rotation_controller"],
#     )

#     chassis_controller_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["chassis_controller"],
#     )

#     joint_state_broadcaster_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#     )

#     ir_sensor_broker_node = Node(
#         package='robot',
#         executable='ir_sensor_broker',
#         name='ir_sensor_broker',
#     )

#     ultrasonic_sensor_broker_node = Node(
#         package='robot',
#         executable='ultrasonic_sensor_broker',
#         name='ultrasonic_sensor_broker',
#     )

#     return LaunchDescription([
#         ExecuteProcess(
#             cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
#             output ='screen'
#         ),
#         robot_state_publisher_node,
#         robot_spawner_node,
#         head_rotation_controller_node,
#         chassis_controller_node,
#         joint_state_broadcaster_node,
#         ir_sensor_broker_node,
#         ultrasonic_sensor_broker_node
#     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from config.constants import Constants


def generate_launch_description():
    pkg_path = get_package_share_directory(Constants.PKG_NAME)
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # 1. Сначала объявляем все ноды
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
        output='screen'
    )

    robot_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-timeout', '30',
        ],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    head_rotation_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_rotation_controller"],
        output='screen'
    )

    chassis_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["chassis_controller"],
        output='screen'
    )

    ir_sensor_broker_node = Node(
        package='robot',
        executable='ir_sensor_broker',
        name='ir_sensor_broker',
    )

    ultrasonic_sensor_broker_node = Node(
        package='robot',
        executable='ultrasonic_sensor_broker',
        name='ultrasonic_sensor_broker',
    )

    # 2. Определяем последовательность запуска с задержками
    return LaunchDescription([
        # Запускаем Gazebo и robot_state_publisher первыми
        gazebo_process,
        robot_state_publisher_node,
        
        # Запускаем controller_manager
        controller_manager_node,
        
        # Ждем инициализации controller_manager перед спавном робота
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[
                            robot_spawner_node,
                            
                            # Сначала joint_state_broadcaster
                            joint_state_broadcaster_node,
                            
                            # Затем остальные контроллеры с задержкой
                            TimerAction(
                                period=2.0,
                                actions=[
                                    head_rotation_controller_node,
                                    chassis_controller_node,
                                    
                                    # И только потом сенсоры
                                    TimerAction(
                                        period=1.0,
                                        actions=[
                                            ir_sensor_broker_node,
                                            ultrasonic_sensor_broker_node
                                        ]
                                    )
                                ]
                            )
                        ]
                    )
                ]
            )
        )
    ])