#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from pgbot_camera_control.srv import CameraCommand


def shoot(timeout):
    global pin_shoot, pin_focus

    try:
        # Focus
        

        # Wait 1 second
        # rospy.sleep(rospy.Duration(1))

        # Shoot
       

        # Wait half a second
        rospy.sleep(rospy.Duration(0.5))

        # Reset shutter

        # rospy.sleep(rospy.Duration(0.2))

        # Reset focus

        rospy.sleep(rospy.Duration(0.2))
    except Exception as ex:
        rospy.logerr(ex)
        return -1

    return 0

def camera_conrtol_callback(data):
    res = shoot(data.timeout)

    return res
    


if __name__ == '__main__':
    try:
        rospy.init_node('pgbot_camera_control', anonymous=True)
        rospy.loginfo('Node pgbot_camera_control started to work')

        pin_focus = 15
        pin_shoot = 18

        try:
            rospy.loginfo('Setting GPIO to BCM mode')

            rospy.loginfo('Set GPIO to BCM mode')
            
            rospy.loginfo('Setting pin_focus to out - GPIO{0}'.format(pin_focus))

            rospy.loginfo('Set pin_focus to out - GPIO{0}'.format(pin_focus))
            
            rospy.loginfo('Setting pin_shoot to out - GPIO{0}'.format(pin_shoot))

            rospy.loginfo('Set pin_shoot to out - GPIO{0}'.format(pin_shoot))

            rospy.loginfo('Reset pin_focus')


            rospy.loginfo('Reset pin_shoot')

            


            rospy.loginfo('Creating command_camera service.')
            rospy.Service('command_camera', CameraCommand, camera_conrtol_callback)
            rospy.loginfo('Created command_camera service.')

        except rospy.ROSInterruptException as ex:
            rospy.logerr(ex)

        rospy.spin()
    except Exception as ex:
        rospy.logerr(ex)
