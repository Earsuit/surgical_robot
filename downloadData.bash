#!/bin/sh
spawn scp -pr earsuit@10.13.111.88:/home/earsuit/catkin_ws/src/surgical_robot/modules/lse/data .
expect "assword:"
send "940413\r"
interact