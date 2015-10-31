# Because Olin's network doesn't handle hostnames properly, we need to tell ROS
# to use IP addresses for communicating instead. To do this manually, use 
# ifconfig or ipconfig to find the hostname.

# Note that I think this script needs to be run in each terminal window you want to use.
export ROS_IP=$(hostname --all-ip-addresses)


# Also, because the laptop should not be running a copy of roscore during full operation, 
# we need to tell it to look for Enterprise's ros server.
export ROS_MASTER_URI=http://192.168.17.130:11311

echo "ran"