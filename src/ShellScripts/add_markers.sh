
#!/bin/sh

WORLD_PATH=$( cd "$(dirname "$0")/../World" ; pwd -P )
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$WORLD_PATH/lw_house.world gui:=false" &
sleep 5

xterm -e "roslaunch launch/amcl.launch map_file:=$WORLD_PATH/lw_house_map.yaml" &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "rosrun add_markers add_markers"


