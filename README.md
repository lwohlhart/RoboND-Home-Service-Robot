# Home Service Robot

This repository hosts a ROS catkin workspace with components to operate a Home Service Robot.

##  Workspace Setup

Clone this respository to your workspace:

```
cd ~
git clone https://github.com/lwohlhart/RoboND-Home-Service-Robot.git catkin_ws
```

Make sure you have `wstool` installed: (see http://wiki.ros.org/wstool for more information)
```
sudo apt install python-wstool
```

Go to the workspace and install the package dependencies
```
cd ~/catkin_ws/src
wstool update
rosdep install --from paths .
```

