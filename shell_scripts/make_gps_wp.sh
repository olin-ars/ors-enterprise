#!/bin/sh
rostopic pub /raw_waypoints geometry_msgs/Pose2D "x: $1
y: $2
theta: 1" 
