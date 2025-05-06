# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

# import xacro


# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')

#     pkg_path = get_package_share_directory('robot')
#     urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

#     with open(urdf_path, 'r') as f:
#         robot_description = f.read()
    
#     # Чтение URDF
#     with open(urdf_path, 'r') as f:
#         robot_description = f.read()
    
#     # Нода robot_state_publisher
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': True  # True для Gazebo
#         }]
#     )

#     # Launch!
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use sim time if true'),

#         node_robot_state_publisher
#     ])