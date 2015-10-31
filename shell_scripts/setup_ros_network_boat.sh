# Because Olin's network doesn't handle hostnames properly, we need to tell ROS
# to use IP addresses for communicating instead. To do this manually, use 
# ifconfig or ipconfig to find the hostname.

# Execute this file on enterprise's computer via SSH.

# Note that I think this line needs to be run in each terminal window you want to use.
export ROS_IP=$(hostname --all-ip-addresses)

echo "ran"