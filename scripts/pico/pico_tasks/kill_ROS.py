#!/usr/bin/env python
import os

# Shout out https://answers.ros.org/question/237862/rosnode-kill/
nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n","")

for node in nodes:
    os.system("rosnode kill "+ node)


os.system("killall roscore")