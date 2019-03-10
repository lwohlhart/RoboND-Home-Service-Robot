
#!/bin/sh

PARENT_DIR=$( cd "$(dirname "$0")/.." ; pwd -P )
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PARENT_DIR/World/lw_house.world gui:=false" &
sleep 5

xterm -e "roslaunch launch/amcl.launch map_file:=$PARENT_DIR/World/lw_house_map.yaml" &
sleep 5

xterm -e "rosrun rviz rviz -d $PARENT_DIR/RvizConfig/home_service.rviz" &
sleep 5

xterm -e "rosrun pick_objects pick_objects" & 
sleep 2

xterm -e "rosrun add_markers add_markers"


