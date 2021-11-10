#!/usr/bin/env python
import rospy
from pgbot_turret_control.srv import CommandTurret
from pgbot_turret_control.srv import GoToPose
import math

def turret_pose_callback(data):
    global poses, speed, command_turret_proxy, turret_mount_yaw_degrees



    if(data.posename not in poses and not data.posename.startswith('manual')):
        rospy.logerr('Pose name is not recognized: {0}'.format(data.posename))
        return

    # TODO - cut the out of range shooting
    _pan = 0
    _tilt = 0
    if data.posename.startswith('manual'):
        info_parse = data.posename.split('|')
        _pan = float(info_parse[1])
        _tilt = float(info_parse[2])
    else:
        _pan = poses[data.posename]['pan']
        _tilt = poses[data.posename]['tilt']


    _final_pan = _pan + turret_mount_yaw_degrees

    # _tilt = ((_tilt)  % 360) 
    _final_pan = (_final_pan) 

    rospy.loginfo('New message received. Sending turret to posename: {0} - pan: {1}, tilt: {2}'.format(data.posename, _final_pan , _tilt ))


    return command_turret_proxy.call(_final_pan , _tilt, speed).res

if __name__ == '__main__':
    rospy.init_node('pgbot_turret_pose_control', anonymous=True)
    rospy.loginfo(' Node pgbot_turret_pose_control started to work')

    try:

        rospy.loginfo('Waiting for the control service to come online')
        rospy.wait_for_service('/command_turret')

        rospy.loginfo('Control service is online')
        command_turret_proxy = rospy.ServiceProxy('/command_turret', CommandTurret)


        rospy.loginfo('Creating pgbot_turret_pose service.')
        rospy.Service('pgbot_turret_pose', GoToPose, turret_pose_callback)
        rospy.loginfo('Created pgbot_turret_pose service.')
        poses = rospy.get_param('~poses')
        speed = rospy.get_param('~speed')
        turret_mount_yaw_degrees = rospy.get_param('~turret_mount_yaw_degrees')
        initialization_move = rospy.get_param('~initialization_move')

        if(initialization_move):
            pose = lambda: None
            pose.posename='neutral'
            turret_pose_callback(pose)
            pose.posename='forward0'
            turret_pose_callback(pose)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()