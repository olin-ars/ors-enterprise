#!/bin/sh
rostopic pub /operating_mode std_msgs/Int16 "data: $1"