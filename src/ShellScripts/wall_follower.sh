
#!/bin/sh

WORLD_PATH=$( cd "$(dirname "$0")/../World" ; pwd -P )
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$WORLD_PATH/lw_house.world" &
sleep 10 

xterm -e "roslaunch wall_follower gmapping_lw_house.launch" &
sleep 10

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

xterm -e "rosrun wall_follower wall_follower"

