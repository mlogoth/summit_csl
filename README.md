summit_csl
=============

Packages for the simulation of the Summit XL HL modified by the team of Control System Lab, National Technical University of Athens.

![alt text](https:////github.com/mlogoth/summit_csl/pa10_urdf.png)

Tested on Ubuntu 14.04 LTS with ROS Indigo and Gazebo 2.2.3.

<h1> Dependencies </h1>
- robotnik_msgs [link](https://github.com/RobotnikAutomation/robotnik_msgs)
- move_base 
- amcl
- gmapping

<h1> Packages </h1>

<h2>summit_xl_gazebo</h2>

In order to start the models in gazebo you have to launch:
```sh
roslaunch summit_xl_gazebo summit_xl_csl.launch
```
This file includes:
```XML
<launch>

     <env name="LC_NUMERIC" value="C" />
    <arg name="launch_rviz" default="true"/>
    <arg name="world" default="$(find summit_xl_gazebo)/worlds/summit_xl_office.world"/> 

    <!-- startup simulated world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(arg world)"/>
        <arg name="gui" value="true"/>
        <arg name="debug" value="false"/>
    </include>


    <!-- spawn Summit XL Mobile Platform -->
	<include file="$(find summit_xl_gazebo)/launch/summit_xl_csl_spawn.launch">
        <!-- Robot Name ID -->
        <arg name="id_robot" value="summit_xl"/>
        <!-- Robot Pose -->
        <arg name="x_init_pose" value="0.0"/>
        <arg name="y_init_pose" value="0.0"/>
        <arg name="a_init_pose" value="0.0"/>
        <!-- Launch Robot Localization Nodes -->
        <arg name="launch_robot_localization" value="true"/>
        <arg name="launch_mapserver" value="true"/>
        <arg name="launch_amcl" value="true"/>
        <!-- Define The map file  -->
        <arg name="map_file" value="$(find summit_xl_localization)/maps/willow_garage/willow_garage.yaml"/>
        <!-- Launch Robot Navigation Nodes -->
        <arg name="launch_move_base" value="true"/>
        <!-- Slam -->
        <arg name="launch_gmapping" value="false"/>
        <arg name="robot_localization_mode" value="odom"/><!-- odom, complete (including gps)-->
        <!-- set ros_planar_move_plugin true for omnidirectional -->
        <arg name="ros_planar_move_plugin" value="true"/>
        <arg name="use_kinect" value="true"/>
	</include>
	
	<!-- launch rviz -->
	<node if="$(arg launch_rviz)" name="rviz" pkg="rviz" type="rviz" required="true" args="-d  $(find summit_xl_gazebo)/rviz/ntua.rviz"/>

</launch>

```
The above launch file:
 - starts the willow_garage Gazebo world
 - spawns the summit xl robot model
 - starts the robot controllers
 - reads a map [willow_garage.yaml] from disk and offers it via a ROS service 
 - starts the amcl localization package 
 - starts the move_base navigation package
 - launches rviz
 
If you want to use the Summit XL HL with Rubber wheels: 
```XML 
<arg name="ros_planar_move_plugin" value="false"/> 
```

__You can use the topic__ ```${id_robot}/robotnik_base_control/cmd_vel``` __to control the Summit XL robot or send simple goals using by either publishing in topic__ ```/${id_robot}/move_base_simple/goal``` __or using 2D nav goal in rviz.__


<h2>summit_xl_control</h2>

<p>This package contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager. It allows to launch the joint controllers for the Summit XL (4 axes skid steering + 2 axes ptz), Summit XL OMNI (4 axes skid steering, 4 axes swerve drive), Summit X-WAM (4 axes skid steering, 4 axes swerve drive, 1 linear axis for scissor mechanism).

The Summit XL simulation stack follows the gazebo_ros controller manager scheme described in
http://gazebosim.org/wiki/Tutorials/1.9/ROS_Control_with_Gazebo</p>

<h2>summit_xl_description</h2>

The urdf, meshes, and other elements needed in the description are contained here. This package includes the description of the Summit XL HL mobile platforms. The package includes also some launch files to publish the robot state and to test the urdf files in rviz. 
The ``` summit_xl_csl.urdf.xacro``` that is used contains two laser sensors (front, rear) and one kinect v2 sensor.

<h2>summit_xl_localization</h2>

This package contains launch files to use the EKF of the robot_localization package with the Summit XL robots. It contains a node to subscribe to gps data and publish it as odometry to be used as an additional source for odometry.

<h2>summit_xl_navigation</h2>

This package contains all the configuration files needed to execute the AMCL and SLAM navigation algorithms in simulation.

<h2>summit_xl_pad</h2>

This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform including speed level control. The joystick output is feed to a mux (http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)

The node allows to load different types of joysticks (PS4, PS3, Logitech, Thrustmaster). New models can be easily added by creating new .yaml files. If modbus_io node is available, the digital outputs (ligths, axes, etc.) can also be controlled with the pad. If ptz camera is available, the pan-tilt-zoom can also be commanded with the pad. 

