# IGVC2018
## Let's get started with our Gazebo Simulation!

### Navigation (Scroll down for Vision)

Here's some packages you need to install aside from ROS/Gazebo:

Gmapping: ```sudo apt-get install ros-kinetic-slam-gmapping```\
Turtlebot: ```sudo apt-get install ros-kinetic-turtlebot```\
Map_server: ```sudo apt-get install ros-kinetic-map-server```\
Amcl: ```sudo apt-get install ros-kinetic-amcl```\
Move_base: ```sudo apt-get install ros-kinetic-move-base```

Run the following commands to setup your workspace:

```
mkdir -p ~/mybot_ws/src
cd ~/mybot_ws/src
git clone -b simulation https://github.com/ut-ras/IGVC2018.git
cd ..
catkin_make
```

Edit your bashrc file: ```gedit ~/.bashrc```

Scroll down to the very end and add the following:
```
source /opt/ros/kinetic/setup.bash
source ~/mybot_ws/devel/setup.bash
```

Check that you have sourced properly:
```echo $ROS_PACKAGE_PATH /home/youruser/catkin_ws/src:/opt/ros/kinetic/share```

Mapping your first environment:

```
roslaunch mybot_gazebo mybot_world.launch
roslaunch mybot_navigation gmapping_demo.launch
roslaunch mybot_description mybot_rviz_gmapping.launch
roslaunch mybot_navigation mybot_teleop.launch
```

Drive around until the map is deemed satisfactory…
DOONT close the gazebo/rviz stuff yet:

```
rosrun map_server map_saver -f ~/mybot_ws/src/mybot_navigation/maps/test_map
```

Save rviz file if you want.
It would go inside: ~/mybot_ws/src/mybot_description/rviz

Steps to use AMCL package:

```
roslaunch mybot_gazebo mybot_world.launch
roslaunch mybot_navigation amcl_demo.launch
roslaunch mybot_description mybot_rviz_amcl.launch
```

Under Rviz tools at the top, click 2D Nav Goal then click and hold wherever you want!

### Vision

To test the visual processing within the simulation please create your workspace if you have not done so yet:

```
mkdir -p ~/mybot_ws/src
cd ~/mybot_ws/src
git clone -b simulation https://github.com/ut-ras/IGVC2018.git
cd ..
catkin_make
```

Next you'll want to edit your bashrc file: ```gedit ~/.bashrc```

Scroll down to the very end and add the following:
```
source /opt/ros/kinetic/setup.bash
source ~/mybot_ws/devel/setup.bash

export GAZEBO_MODEL_PATH=${HOME}/.gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=${HOME}/mybot_ws/src/mybot_gazebo/models:$GAZEBO_MODEL_PATH
```

We have to build our models within Gazebo. To do this, run the simple script in the models folder:
```
cd ~/mybot_ws/src/IGVC2018/mybot_gazebo/models
sudo ./install_models.sh
```

Run the following launch files:

```
roslaunch mybot_gazebo igvc_world.launch
roslaunch mybot_description mybot_rviz.launch
roslaunch vision camerafeed.launch
```

And now you're set! Feel free to play around with the camera angles and other parameters. This is primarily for testing vision software only. No path planning or cmd_vel commands are currently being published.
