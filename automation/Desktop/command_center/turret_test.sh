#!/bin/bash
source ~/Desktop/command_center/base.bash
echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Sending turret to forward0"
rosservice call /pgbot_turret_pose "posename: 'forward0'" &&

echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Sending turret to forward30"
rosservice call /pgbot_turret_pose "posename: 'forward30'" &&

echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Sending turret to forward60"
rosservice call /pgbot_turret_pose "posename: 'forward60'" &&

echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Sending turret to Neutral"
rosservice call /pgbot_turret_pose "posename: 'neutral'" &&

echo "Sending turret to Home"
rosservice call /pgbot_turret_pose "posename: 'home'" &&

echo "Setting turret to Relax"
rosservice call /pgbot_turret_pose "posename: 'relax'"
