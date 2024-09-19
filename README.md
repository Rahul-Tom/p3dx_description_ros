### Package based on https://github.com/MobileRobots/amr-ros-config/tree/master/description and https://github.com/RafBerkvens/ua_ros_p3dx

# p3dx_description_ros

P3DX description files compatible with ROS1 and ROS2

You must ignore this pkg when using rosdep!

Just add some cahnges
python ""ros2 launch p3dx_description_ros p3dx_description_ros2.launch.py 
ros2 launch nav2_bringup localization_launch.py map:=../robtics_lab_0709_1930_masked2.yaml
ros2 launch p3dx_description_ros navigation_launch.py --show-args""

