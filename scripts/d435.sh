#!/usr/bin/env sh


echo "Starting visual odom"
roslaunch vo_orb d435.launch &
wait
echo "All processes done!"
