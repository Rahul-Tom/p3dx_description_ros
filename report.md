# Transformation

Transformation is the mathematical representation and computation of relations between various coordinated system of the robot in 3D space in terms of translation, rotation and relative motion. An example from our case is that laser mounted on the top of the robot. The coordinate frame of the base of the robot is known as base_link and of the lidar know as lidar_link. If the lidar_link is located 10 c.m forward and 20 c.m above from base_link with same orientation, the data obtained from lidar(distance measurement) must be translated to above mentioned values to measure the obstacle distance from the center of base of the robot.


Figure 1 base_link and laser-link relation

The transformation between various coordinated frame (transformation tree) can be done by using TF2 ROS2 package. TF2 provides the transformation tree not only for static frames, where no relative motion between frames, but also for non-static frames, such as frame has relative motion with other frames. TF2 uses a tree structure which allow only one parent frame for any child frame. But the parent may have more than one child. The tree structure of TF2 guarantee that there is only a single traversal that links any two coordinate frames together, and assumes that all edges in the tree are directed from parent to child frame (nodes). We use Robot State Publisher, a ROS2 package that interacts with the tf2 package, to publish all the necessary transforms that can be directly inferred from the URDF file, which contains the XML description about the robot’s geometry and structure. The figure below is the visualization of transformation between various coordinates published by robot_state_publisher package in RVIZ(a visualization tool in ROS).


Figure 2 Static coordinate frames of P3DX robot with lidar sensor in RVIZ

All the transformation in ROS comply to REP 105 - Coordinate Frames for Mobile Platforms and REP 103 - Standard Units of Measure and Coordinate Conventions. The former one useful specifically for the naming convention while the latter one is for measuring unit and convention of coordinate frames should followed in ROS.

# Nav2

Nav2 is the high-grade and high-performance navigation stack using ROS2 as the core middleware. Nav2 provides different autonomous stacks like perception, planning, control, localization and visualization to navigate the mobile robot through complex environment to complete user defined task. Nav2 strengthen robots to avoid not only static but also dynamic obstacle in the environment. Nav2 use of behavior trees to orchestrate navigation algorithms to be highly configurable and intelligent via different independent modular task servers like path planning and obstacle avoidance (1). ROS2’s interface like services and action enable the communication of different servers with behavior tree. Properly configured behavior tree enables robot to perform unique and complex tasks in the environment where it is deployed.

Behavior tree server host the behavior tree by loading the custom BT files in XML format. The high-level architecture diagram of Nav2 is provided here. The BT written in XML format, TF transformation confirmation to REP-105, the map source and relevant sensor data are the expected inputs of Nav2 to perform various task by providing valid velocity command to motors of the robot.


Figure 3 Nav2 high level architecture

# Behavior Tree

Behavior tree (BT) is way to structure the switching between different task of the robot or any other autonomous agent. Autonomous agent should be reactive and modular. Modularity allows us to develop, test and reuse components independently. This is useful when working with a complex system. Reactivity is the ability to swiftly and efficiently prioritizing task with the drastic changes in the environment in a short span. BT evolve as an alternative to the Finite State Machine (FSM), which also serve the same functionality but have tradeoff between modularity and reactivity. 1

BT is a direct rooted tree with internal root, known as control flow nodes and have at least on child, and leaf nodes does not have child, known as execution nodes. The only node without parent is called the root and all other nodes have a parent. A parent node must be a control flow node and may have more than one child. The child either be control flow node(s) or execution node(s).

The main four category in control flow nodes are Sequence, Fallback, Parallel, and Decorator. On other hand Action and Condition are two divisions in execution nodes.

The execution starts with a signal called tick from the root node with a frequency. The ticking starts from left most child. According to the control flow node and return from execution nodes the tick route next children or not.

Action node in execution node return success if the action is correctly completed, running if the action is ongoing and failure if the action has failed. Condition node return success if the condition given is satisfied and returns failure if the condition given is not satisfied. But it never returns running unlike in action node.

Example: ComputePathToPose, FollowPath, Spin, ClearCostmapService

Sequence node executes an algorithm by start ticking its children from left. If the child returns success it will tick next child and so on. Sequence node will return success to its parent if and only if all the children return success. If the child returns failure or running the sequence node will return the same to its parent. The key points are that to return the success from Sequence node all its children must return success, while failure of any of the child cause to return failure to its parent.

When it comes to Fallback node, it executes algorithm by start ticking from left most child. If the child returns failure it will tick next child and so on and Fallback node returns failure to its parent if and only if all children return failure. If the child return success or failure the Fallback node will return success or failure accordingly to is parent. The important points in Fallback nodes are it will return success to its parent if any of the child returns success and it returns failure if and only if all the children return failure.

Parallel node start ticking all the children simultaneously. If it has N children and M is user defined number less than N, it will return success if M children return success, it returns Failure if N − M +1 children return Failure, and it returns Running otherwise.

The Decorator node is a single-child control flow node that selectively ticks its child based on established rules and modifies the child's return status in accordance with user-defined rules.

| **Node** | **Symbol** | **Succeeds** | **Failure** | **Running** | **Next Child Ticking** |
| --- | --- | --- | --- | --- | --- |
| **Sequence** |     | If all children succeed | If one child fails | If one child returns Running | If the precedent child returns success |
| **Fallback** |     | If one child succeeds | If all children fail | If one child returns Running | If the precedent child returns failure |
| **Parallel** |     | If ≥ M children succeed | If > N − M children fail | else | Ticking all children simultaneously |
| **Decorator** |     | Custom | Custom | Custom | Only one child |
| **Condition** |     | If true | If false | Never | Have no child |
| **Action** |     | Upon completion | If unable to complete | During completion | Have no child |

# Nav2 Specific Control Flow Nodes

Nav2 uses some custom BT along with the common behavior tree.

## PipelineSequence

It is similar to sequence node, but it will re-ticks it‘s previous children if the current child is still running. This will last until any node returns failure or last node returns success. If during re-ticking if any of the child returns running but last child is success, then PipelineSequnce node halt the running node(s) and it will return success to its parent. Pipelinesequence node will return failure if any of the child return failure during ticking or reticking.

## Recovery

Recovery node has only two children and it will return success if and only if its first child returns success and returns failure either if first second child returns failure or if number of retries exceed than the given threshold.

## RoundRobin

RoundRobin nodes ticks its children in a round robin method to get success from a child. As the child return success, the RounRobin node also returns success to its parent. But if all the children return failure the RoundRobin node will returns failure to its parent. The ticking continues from left most child and when a child returns success all the children will be hated and the RoundRobin returns success, but the state information will be saved. So, using this state information it will tick the child next to on who returns success rather than starting from first child. The first tick only get ticked again either if the last child failed or if it goes for next round after last child succeed.

# Odometry

Based on robot’s motion, odometry system provides a locally accurate estimate of a robot’s pose and velocity. It is essential for localization, mapping, navigation and control. The range of sensors used for odometry measurement are wheel encoders, inertial measurement unit (IMU), Lidar, camera (VIO) and so on. Each sensor has its own limit leading to inaccurate odometry information over time, distance or any other factor. So rather than relaying on one sensor for odometry, use of multiple sensors at time using sensor fusing gives better odometry result. We are using wheel encoders and lidar for odometry information.

## Wheel Encoders

Encoders are the sensors attached to the wheels of the robots provides speed as well as direction of rotation of wheels and can be used for dead reckoning and this technique is called wheel odometry. P3DX robot comes with high resolution optical quadrature shaft encoders. Two light source and their counterpart for sensing placed 90-degree phase difference between them. The shaft comes with slits so that as it rotates light will be passed through slits and opaque region in between slits blocks the light. According to the number of times light detected at light sensors placed opposite to light source number the rotation will be measure. The quadrature sensors (placed with 90-degree phase difference) helps to detect the degree of rotation. Wheel odometry of P3dx robot is provided by rosaria package. [rosaria](https://wiki.ros.org/ROSARIA) package publishes [nav_msgs/Odometry](https://docs.ros.org/en/hydro/api/nav_msgs/html/msg/Odometry.html) message on the topic called /RosAria/Pose.

## Lidar Odometry

It uses Lidar sensor to measure the distance to the objects in the environment. By comparing consecutive scans, called scan matching, we can measure the change in robot’s position and orientation. Lidar odometry overcome several limitation of wheel odometry such as slippage issue, wheel failures, compounding small error for large distance. It uses Iterative Closest Point (ICP) algorithm to scan matching and uses a package called [ros2_laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git). The following is the launch file, without import items, which uses ros2_laser_scan_matcher package. The important here to note is the parameters.

- **base_frame**: Which frame to use for the robot base
- **odom_frame**: Which frame to use for the odom
- **laser_frame**: Which frame to use for the laser
- **publish_odom**: Name of the topic where to publish the computed odometry.
- **publish_tf**: If true it will publish odom->base_link tf.


Figure 4 Launch file for lidar odometry with ros2_laser_scan_matcher package

This package will publishes [nav_msgs/Odometry](https://docs.ros.org/en/hydro/api/nav_msgs/html/msg/Odometry.html) message on the topic called lase_scan_matcher-odom. Since publish_tf parameter is set to true it will also publish the transform of odom->base_link.

## Sensor Fusion

The technique of fusing data from different sensor source to provide more accurate information is called sensor fusion. The nav_msgs/Odmetry messages from wheel odometry and Lidar odometry is fused using [robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package. robot_localization is a collection of non-linear state estimation nodes  for robots moving in 3D space such as ekf_localization_node and ukf_localization_node. The package accept arbitrary number of sensor inputs with nav_msgs/Odometry, sensor_msgs/Imu, geometry_msgs/PoseWithCovarianceStamped and geometry_msgs/TwistWithCovarianceStamped messages. For our project we uses ekf \_node which is an implementation of Extended Kalman Filter (EKF).


Figure 5 Launch file's robot_localization_node snippet

The parameter for the ekf_node is located inside the config directory and in the ekf.yaml file from the launch file directory. The parameter file added in the Appendix i:


Figure 6 Parameter file of ekf_node

- **frequency:** The frequency, in Hz, at which the filter will output a position estimate.
- **two_d_mode:** Weather to use 2D or 3D information for state estimation.
- **Publish_accleration:** set to true so that the node will publish linear acceleration on /accel/filtered topic
- **map_frame:** The map frame
- **base_link_frame:** Which frame to use for the robot base
- **odom_frame**: Which frame to use for the odom
- **world_frame:** which frame to use for world. It would be either odom_frame or

map_frame.

- **odom0**: The odometry frame wheel encoders published by RosAria package.
- **odom1**: The odometry frame laser scan matcher published by ros2_laser_scan_matcher package
- **\_config:** It contain x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az. In our case the filter will only use the x, y, z, and the vyaw values of odom_\*

# Localization

Localization is very important in robot navigation as it localize robot in working environment, absolute localization, as well as it gives relative distance to the goal or target, relative localization. For our case we use two localization methods. The first one is AMCL comes with Nav2 Package, and it needs map of the environment. On other hand when the map of the environment is not readily available, we can use SLAM. Localization not only gives the position of robot in the environment but also the pose or orientation.

# Simultaneous Localization and Mapping (SLAM)

SLAM is technique in which robot can map its environment and simultaneously localize itself it that environment. It is extremely useful when robot exposed to to an unknow environment or map of the environment drastically changed. We are using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox.git) package from Steve Macenski. We use Lidar as our main sensor. The installation of slam_toolbox is given in the Appendix. We used online asynchronous slam. Online means robot continuously builds and updates a map of the environment and tracks its location in real-time while it moves through the environment. Asynchronous means it process newest available data for processing. The slam_toolbox subscribe to /scan topic with sensor_msgs/LaserScan message and /tf which provides odom to base link transform. On other hand the publishing topics are /map with nav_msgs/OccupancyGrid message, /pose with geometry_msgs/PoseWithCovarianceStamped message and tf which provide map to odom transform.



Figure 7 slam_toolbox in launch file

The launch file’s important part for slam_toolbox is as above figure. The important part is params file. The parameter file is added to the Appendix A. The parameters developer need to touch or adjust are odom_frame, map_frame, base_frame, scan, max_laser_range.


Figure 8 Occupancy Grid Map of Robotics lab, HFU

The above figure is the occupancy grid map of the HFU’s Robotics lab at 15/09/2024 using slam_toolbox. The white color indicates the free space, black color indicates the space are occupied, and the gray color indicates the unknown region (either occupied or free).

# References

(1.). Retrieved from <https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9341207&tag=1>
