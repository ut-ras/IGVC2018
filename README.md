# IGVC
Welcome to the UT RAS IGVC repo. This file will give installation/usage instructions. If you want to develop on this repo see our [contributing guide](other_file.md).

# Setup

First you need to download and install ros by following these two guides:

1. [Install Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (If you're on 17.04 you'll need to [Install Lunar](http://wiki.ros.org/lunar/Installation/Ubuntu) instead)
2. [Configure Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Then clone our repo into the src folder:

```
  cd ~/catkin_ws/src
  git clone https://github.com/ut-ras/IGVC2018.git
```

To install dependencies for the vision package
```
cd ~/catkin_ws/src/IGVC2018
sudo apt install $(grep -vE "^\s*#" depend  | tr "\n" " ")
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
**replace the device_path parameter in the launch file with the correct id for your system**
```

To run the vision package run each of the following in multiple terminal prompts
```
roslaunch vision camerafeed.launch
rosrun rviz rviz
```
On rviz add an image topic in the bar on the right and select the camera feed in the dropdown menu
