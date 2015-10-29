echo WARNING: this script is not quite right for anything that isn't a FIT-PC.
echo rosserial will continue trying to run in the backgroud, restarting itself if it dies.
export ROS_IP=192.168.17.130
watch rosrun rosserial_python serial_node.py /dev/ttyACM0 & 
# watch rosrun rosserial_python serial_node.py /dev/ttyACM1 & 