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
 ...and install the needed dependencies:

```
sudo apt install $(grep -vE "^\s*#" depend  | tr "\n" " ")
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
catkin_make
source install/setup.bash
source devel/setup.bash
```

If you want to be able to edit the motor control code on the TM4C you'll need to [install the Energia IDE](http://energia.nu/download/) and the necessary libraries:

```
cd <sketches_dir>/libraries
rosrun rosserial_tivac make_libraries_energia .
```

