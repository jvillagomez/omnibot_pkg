#!/usr/bin/env python
import os

def kill_ros():
    os.system("killall roslaunch")
    os.system("rosnode kill -a")
    os.system("killall roscore")
    return 0
