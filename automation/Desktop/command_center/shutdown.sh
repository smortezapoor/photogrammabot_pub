#!/bin/bash
source ~/Desktop/command_center/base.bash

echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Setting turret to Relax"
rosservice call /pgbot_turret_pose "posename: 'relax'"

shutdown now
