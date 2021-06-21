NOTE: 
Your environment should not have black since it can absorb the laser scan. Also avoid using reflexive stuff. If your laser scan data of a wall or obstacle doesn’t appear how it should be in rviz then consider changing it.
iRobot Create odom sucks. Please measure your wheel because you might run into problems with amcl (since amcl is laser + odom) and navigation because of incorrect odom. Make changes to the iRobot package if necessary. For more info: https://github.com/AutonomyLab/create_autonomy/issues/32


## Installation

### iRobot stuff:
```shell
sudo apt-get install python-rosdep python-catkin-tools
cd ~
mkdir -p create_ws/src  # create new workspace/directory call create_ws and src in it
cd create_ws  # go to create_ws from src
catkin init # initialize
cd ~/catkin_ws/src  # go into the src (source)
git clone https://github.com/AutonomyLab/create_autonomy.git  # clone all stuff from create_autonomy
cd ~/catkini_ws   # go to create_ws from src
rosdep update  # Install dependencies
rosdep install --from-paths src -i  # Install dependencies

cd ~/catkin_ws # go to create_ws if you aren’t there already

catkin_make # build (catkin_make)

sudo usermod -a -G dialout $USER # In order to connect to Create over USB, ensure your user is in the dialout group

source ~/catkin_ws/devel/setup.bash # source your workspace
```


You can find the iRobot package and more about Publisher and Subscriber and etc. at https://github.com/AutonomyLab/create_autonomy.

### Teleop

```shell
$ sudo apt-get install ros-<YOUR DISTRO>-teleop-twist-keyboard 
# Ex: For indigo, 
$ sudo apt-get install ros-indigo-teleop-twist-keyboard 
```

### Navigation

```shell
$ sudo apt-get install ros-<YOUR DISTRO>-navigation
```


## Connect

Source catkin workspace and roslaunch to get started.

```shell
$ roslaunch ara_irobot ara_irobot.launch
```

In RVIZ, add the following visualization topics:

- Add `LaserScan` and put `/scan` for the topic.
- Add `Map` and put `/map` for the topic.
- Add `RobotModel` so you know where it will be.


Add what whatever else you want!

## Sensor Information

**Visual Odometry**
Device Name: Intel RealSense T265
Device Serial No: 952322110878

**Depth RGB Image**
Device Name: Intel RealSense D435I
Device Serial No: 042222071741

**Laser Scanner**
ID: H1101786


Roscore 
Roslaunch ca_driver create.2_launch
Rosrun hokuyo_node hokuyo_node
Rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link laser 100
Roslaunch amcl amcl_diff.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
Rosrun rviz rviz
In rviz add topic particle cloud


ONCE YOU ARE SATISFIED WITH YOUR MAP
Amcl and save map:
Rosrun map_server map_saver -f <map name>  // save map /<directory>/directry/mapname
Roslaunch amcl amcl_diff.launch // run localization
Rosrun map_server map_server <map name>.yaml  // send map to amcl

For moving robot:
Create your own cmd_vel file for moving:
First try to get irobot to rotate and go straight to a goal. Then, consider obstacle avoidance
Will update more on this
            OR
Move_base:

To create the move_base.launch file you go to:
http://wiki.ros.org/navigation/Tutorials/RobotSetup
The only difference is:
catkin_create_pkg my_robot_name_2dnav move_base
Instead of
catkin_create_pkg my_robot_name_2dnav move_base my_tf_configuration_dep my_odom_configuration_dep my_sensor_configuration_dep
You also do not need my_robot_configuration.launch
Starting at 2.3 you can follow the instruction manually
    OR
Create a folder in src called param
Then create these files:

base_local_planner_param.yaml

TrajectoryPlannerROS:
 max_vel_x: 0.45
min_vel_x: 0.1
max_vel_theta: 1.0
min_in_place_vel_theta: 0.4

acc_lim_theta: 3.2
acc_lim_x: 2.5
acc_lim_y: 2.5

holonomic_robot: true

costmap_common_param.yaml

obstacle_range: 2.5
raytrace_range: 3.0
#footprint: [[x0, y0], [x1, y1], ... [xn, yn]]
robot_radius: ir_of_robot
inflation_radius: 0.4

observation_sources: laser_scan_sensor

laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}

global_costmap_param.yaml

global_costmap:
  global_frame: /map
  robot_base_frame: base_link
  update_frequency: 5.0
  static_map: true

local_costmap_param.yaml

 local_costmap:
global_frame: odom
robot_base_frame: base_link
update_frequency: 5.0
publish_frequency: 2.0
static_map: true
rolling_window: true
width: 6.0
height: 6.0
resolution: 0.05

Finally create a launch file call move_base.launch: *note that I have map_server and amcl in there so if you already have amcl and map_server running you should delete it from launch file
<launch>

   <master auto="start"/>
   
   <!-- Run the map server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find my_robot_name_2dnav)/map/my_map.yaml"/>
  <!--- Run AMCL -->
<include file="$(find amcl)/examples/amcl_diff.launch" />

   <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
   <!-- change controller frequency in case it is running in circles-->
    <rosparam file="$(find my_robot_name_2dnav)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find my_robot_name_2dnav)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find my_robot_name_2dnav)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find my_robot_name_2dnav)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find my_robot_name_2dnav)/param/base_local_planner_params.yaml" command="load" />
 </node>

</launch>



In rviz:
Pose array: particle cloud
Polygon: Robot Footprint /local_costmap/footprint
Gridcells: obstacle /local_costmap/obstacle
Grillcells: inflated obstacle /local_costmap/inflate_obstacle
Path: Global Plan /trajectoryplannersROS/global_plan
Path: Local Plan /trajectoryplannersROS/local_plan
Path: Planner Path /NavfROS/plan
Pose: Current Goal /current_goal
Right click 2d nav, properties, move_simple/goal



RESET COMPUTER
Navigate to Settings. ...
Select "Update & security"
Click Recovery in the left pane. ...
Click Get started under Reset this PC.
For pictures go to:
https://www.laptopmag.com/articles/reset-windows-10-pc

Tutorial on download ubuntu:
https://www.youtube.com/watch?v=qNeJvujdB-0

Create boot:
Download ubuntu 16.04
Select 64-bit PC (AMD64) desktop image

Download rufus
https://rufus.akeo.ie/

Open create and format hard disks partition
Go to the largest space on and right click and select compress and choose a large portion of space to split

Plug in your usb (MAKE SURE UR USB IS EMPTY)
Open rufus
In Device select your usb name
For partition scheme
Select bios or uefi
For file system select fat32 or ___
Select the disk image at the bottom right it will open a folder, choose the ubuntu iso that you download t the beginning.
Press start

Once you are done, restart the computer with the usb still on
When laptop opens, press f2 or f10 or f12 with ctrl or alt it varies in computer
When that is select your usb and start download ubuntu


C cd ~/catkin_ws catkin_make 




