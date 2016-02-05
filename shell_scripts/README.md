# Shell Scripts

* For the development computer
  * `./connect_to_fitpc.exp`
    * This connects to the FitPC and automatically enters the password. It will soon be replaced with a version that doesn't need expect installed.
    
  * `source setup_ros_network_laptop.sh`
    * This sets up environment variables for ROS on your laptop to listen to roscore on the boat. It needs to be _source_'d in any terminal window that you run ROS commands in

* For the FitPC
  * `switch_fitpc_to_OLIN-ROBOTICS.sh`
    * This should not be necessary, unless the FitPC is currently connected to ORS-NETWORK and we want to turn off ORS-NETWORK but don't trust the FitPC to automatically switch to Olin-Robotics
  * `switch_fitpc_to_ORS-NETWORK.sh`
    * Use this script if we turned on the ORS router but the FitPC isn't connecting, either because it is already on Olin-Robotics or because it is being grumpy.
