cmake_minimum_required(VERSION 3.5)
project(p3dx_description_ros)

cmake_policy(SET CMP0057 NEW)

find_package(urdf REQUIRED)

# find_package(catkin QUIET)
# find_package(ament_cmake QUIET)
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
install(DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# if(catkin_FOUND)
#   message("catkin found! Assuming ROS1 build.")
#   set(ros1_build 1)
# else()
#   message("catkin not found!")
#   if(ament_cmake_FOUND)
#     message("ament_cmake found! Assuming ROS2 build.")
#     set(ros_build 0)
#   else()
#     message( FATAL_ERROR "Unable to source ROS1 and ROS." )
#   endif()
# endif()

# if(ros1_build)
#   message("Building p3dx_description_ros for ROS1.")

#   catkin_package()

#   include_directories(
#     ${catkin_INCLUDE_DIRS}
#   )

# else()
#   message("Building p3dx_description_ros for ROS2.")
#   if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
#   endif()

#   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
#   endif()

#   install(DIRECTORY meshes urdf launch
#     DESTINATION share/${PROJECT_NAME}
#   )

#   ament_package()

# endif()

install(
  DIRECTORY meshes urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
# Mark executable scripts
install(PROGRAMS
  src/dynamic_object_detector_2d_lidar.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
