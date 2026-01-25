HOW TO EXECUTE
To execute the program 3 terminals are needed, in the first one we use the launch file already 
created in bme_gazebo_sensors to start the simulation Rviz + Gazebo
-ros2 launch bme_gazebo_sensors spawn_robot.launch.py
In the second one we use the launch file "assignment2_rt.launch.py" to start 3 nodes: distance_check
threshold_service and user_twist_publisher.
-ros2 launch assignment2_rt_py assignment2_rt.launch.py
In the last one only one node is executed because it needs the stdin for the user's inputs:
-ros2 run assignment2_rt_cpp UI

DESCRIPTION
The project implements a ROS2 simulation that manages a robot to move in an environment with the 
presence of obstacles. The robot has sensors that allows it to move safely without crashing.

  1. bme_gazebo_sensors
Start the simulation (Gazebo) and the 3D visualization (Rviz), other nodes are actually executed:
gz_bridge, robot_state_publisher.

  2. UI Node (ui_node.cpp)
This node is executed separately in a terminal because it needs the user input. It provides a simple
command-line interface to manually control the robot.
It stores the linear and angular velocity of the user as a geometry_msgs Twist on the /user_cmd_vel
topic.

  3. user_twist_publisher node (user_twist_publisher.cpp)
The user input does not directly command the robot, because it needs the cmd_vel at a constant
frequency. The UI node block the execution waiting for the user input, while the
user_twist_publisher node stores the latest command and publish it on the /cmd_vel for 5 seconds, if
the user does not insert any new command meanwhile, it sends a null velocities.
-publisher_-> publishes the Twist on the /cmd_vel topic to control the robot.
-subscription_-> subscribes to the /user_cmd_vel topic coming from the UI node.
-safety_sub_-> subscribes to the the /safety_alert topic coming from the distance_check node, it is a
              boolean that alerts the node, if it is true the robot begins the safety maneuver.
-service_-> create a service to get the average of the last 5 inputs.

  4. threshold_service node (threshold_service.cpp)
This node simply creates the service to get and modify in runtime the threshold, which is the minimal
safety distance between the robot and the obstacles. In particular it creates a parameter threshold
with the initial value of 0.5, it can modified by the service set_threshold. The threshold is
declared as a parameter so other nodes can update the value when it is changed by the user.

  5. distance_check node (distance_check.py)
To check the distance from the obstacles this node is subscribed to the /scan topic, where multiple
information are published, such as the array ranges, which contains all the values of the distances
perceived by the scanner in 360 degrees; the angle_min or the angle_increment. Whenever an obstacles
is below the threshold value the node sends two messages on the /safety_alert topic and on the
/custom_message topic, the first one alert the user_twist_publisher to start the safety maneuver,
while the second one publish on a topic the direction and the distance of the closest oblstacles and
the value of the threshold.
-subscription-> subscribes to the /scan topic
-safety_pub-> publishes the boolean message on the /safety_alert topic
-custom_pub-> publishes the custom message on the /custom_message topic
-param_sub-> subscribes to the /param_events topic to check the changes of the threshold parameters


PROJECT DIRECTORIES
/ros_ws
  /src
    /assignment2_rt_cpp
      /src all cpp code
      /msg custom message CustomMessage.msg
      /srv custom services GetAverage.srv SetThreshold.srv
    /assignment2_rt_py
      /assignment2_rt_py all python code
      /launch launch file assignment2_rt.launch.py

MSG
  1. CustomMessage.msg
float64 distance
uint8 direction
float64 threshold

SRV
  1. GetAverage.srv
---                     #no request
float32 linear_average  #response
float32 angular_average
bool success
string message

  2. SetThreshold.srv
float32 threshold       #request
---
bool success            #response
string message
