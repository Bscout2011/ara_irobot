# ARA iRobot

Repository for mobile robot research and development using an iRobot.

## Installation

### iRobot stuff:
```shell
sudo apt-get install python-rosdep python-catkin-tools
cd ~
mkdir -p catkin_ws/src  # create new workspace/directory call catkin_ws and src in it
cd catkin_ws  # go to catkin_ws from src
catkin init # initialize
cd ~/catkin_ws/src  # go into the src directory (source)
git clone https://github.com/Bscout2011/ara_irobot.git  # this repo
git clone https://github.com/AutonomyLab/create_robot.git  # drivers for iRobot control
git clone https://github.com/IntelRealSense/realsense-ros.git  # drivers for Visual Odometry and RGB Depth Cameras
cd ~/catkin_ws   # go to catkin_ws from src
rosdep update  # Install dependencies
rosdep install --from-paths src -i  # Install dependencies

cd ~/catkin_ws # go to catkin_ws if you aren’t there already

catkin build # build (catkin_make)

sudo usermod -a -G dialout $USER # In order to connect to Create over USB, ensure your user is in the dialout group

source ~/catkin_ws/devel/setup.bash # source your workspace
```

You can find the iRobot package and more about Publisher and Subscriber and etc. at https://github.com/AutonomyLab/create_autonomy.

### Teleop

```shell
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard 
```

### Navigation

```shell
sudo apt-get install ros-$ROS_DISTRO-navigation
```

## Connect

Source catkin workspace and roslaunch to get started.

```shell
roslaunch ara_irobot irobot_bringup.launch
```

In a second terminal, open RVIZ.

```shell
rviz -d src/ara_irobot/launch/irobot.rviz
```

In a third terminal, startup tele-operation.

```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Now control the robot using the keyboard.

## Generating a Map

A mobile robot with a range sensor can map out a room. 

In a new terimal start the simultaneous localization and mapping (SLAM) service.

```shell
rosrun gmapping slam_gmapping
```

The `map` topic now displays an occupancy grid as observed by the laser scanner. In RVIZ, set the `Fixed Frame` option to `map`. Drive the robot around to get a decent map.

Once satisfied with a completed map, you can save it. In a new terimal run the command:

```shell
rosrun map_server map_saver [-f <mapname>]
```

Reference the `map/arf_lab` file for an example.

## Navigating around a Map

Given a map, the robot can navigate around to various locations while avoiding static and dynamic obstacles.

In a new terminal, launch the `move_base` nodes.

```shell
roslaunch ara_irobot move_base.launch
```

## Sensor Information

*Putting device serial into launch file works better.*

**Visual Odometry**  
Device Name: Intel RealSense T265  
Device Serial No: 952322110878

**Depth RGB Image**  
Device Name: Intel RealSense D435I  
Device Serial No: 042222071741

**Laser Scanner**  
ID: H1101786
