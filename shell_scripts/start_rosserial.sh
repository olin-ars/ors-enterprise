echo "rosserial will continue trying to run in the backgroud, restarting itself if it dies."

# Because Olin's network doesn't handle hostnames properly, we need to tell ROS
# to use IP addresses for communicating instead. To do this manually, use 
# ifconfig or ipconfig to find the hostname.

# This line is technically redundant if you have already run setup_ros_network_boat.sh
export ROS_IP=$(hostname --all-ip-addresses)

# The trailing & means that this terminal (or SSH session) will continue being useful.
# Perhaps the output should be piped to /dev/null so it doesn't clutter stuff up.
watch rosrun rosserial_python serial_node.py /dev/ttyACM0 &

# This hasn't been used. At some point, we need to have at least 3 of these.
# watch rosrun rosserial_python serial_node.py /dev/ttyACM1 & 