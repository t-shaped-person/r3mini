#r3mini

##package create

###r3mini
'''
ros2 pkg create r3mini --description "Meta package for r3mini." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies r3mini_bringup r3mini_cartographer r3mini_description r3mini_navigation r3mini_robot r3mini_teleop --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r3mini_bringup
'''
ros2 pkg create r3mini_bringup --description "Launch scripts for starting the r3mini." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies r3mini_description r3mini_node ydlidar_ros2_driver --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r3mini_cartographer
'''
ros2 pkg create r3mini_cartographer --description "Launch scripts for cartographer." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies cartographer_ros rviz2 --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r3mini_description
'''
ros2 pkg create r3mini_description --description "3D models of simulation and visualization for r3mini." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies robot_state_publisher rviz2 urdf --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r3mini_navigation
'''
ros2 pkg create r3mini_navigation --description "Launch scripts for navigation2." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies nav2_bringup rviz2 --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r3mini_robot
'''
ros2 pkg create r3mini_robot --description "Driver for r3mini." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies geometry_msgs message_filters rclcpp rosidl_default_runtime tf2_ros_py --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K" --node-name robot_control
'''

###r3mini_teleop
'''
ros2 pkg create r3mini_teleop --description "Teleoperation node using keyboard for r3mini." --license "Apache License, Version 2.0" --build-type ament_cmake --dependencies rclcpp geometry_msgs --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K" --node-name teleop_keyboard
'''
