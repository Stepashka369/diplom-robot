# diplom-robot
1. Активировать ROS2 в терминале: source /opt/ros/humble/setup.bash
2. Сборка проекта (из /diplom-robot): colcon build --packages-select robot
3. Активция изменений: source install/setup.bash
4. Запуск проекта: ros2 launch your_pkg spawn_robot.launch.py