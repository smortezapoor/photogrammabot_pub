#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from pgbot_camera_control.srv import CameraCommand
import RPi.GPIO as GPIO


def shoot(timeout):
    global pin_shoot, pin_focus

    try:
        # Focus
        GPIO.output(pin_focus, True)

        # Wait 3 second
        rospy.sleep(rospy.Duration(1.5))

        # Shoot
        GPIO.output(pin_shoot, True)

        # Wait 0.5 second
        rospy.sleep(rospy.Duration(0.3))

        # Reset shutter
        GPIO.output(pin_shoot, False)
        rospy.sleep(rospy.Duration(0.2))

        # Reset focus
        GPIO.output(pin_focus, False)
        rospy.sleep(rospy.Duration(0.2))
    except Exception as ex:
        rospy.logerr(ex)
        return -1

    return 0

def camera_conrtol_callback(data):
    res = shoot(data.timeout)

    return res
    

def ping_camera():
    global pin_focus
    
    try:
        # Focus
        GPIO.output(pin_focus, True)

        # Wait 1 second
        rospy.sleep(rospy.Duration(1))

        # Reset focus
        GPIO.output(pin_focus, False)
        
    except Exception as ex:
        rospy.logerr(ex)
        return -1

    return 0


if __name__ == '__main__':
    try:
        rospy.init_node('pgbot_camera_control', anonymous=True)
        rospy.loginfo('Node pgbot_camera_control started to work')

        pin_focus = 15
        pin_shoot = 18

        try:
            rospy.loginfo('Setting GPIO to BCM mode')
            GPIO.setmode(GPIO.BCM)
            rospy.loginfo('Set GPIO to BCM mode')
            
            rospy.loginfo('Setting pin_focus to out - GPIO{0}'.format(pin_focus))
            GPIO.setup(pin_focus, GPIO.OUT)
            rospy.loginfo('Set pin_focus to out - GPIO{0}'.format(pin_focus))
            
            rospy.loginfo('Setting pin_shoot to out - GPIO{0}'.format(pin_shoot))
            GPIO.setup(pin_shoot, GPIO.OUT)
            rospy.loginfo('Set pin_shoot to out - GPIO{0}'.format(pin_shoot))

            rospy.loginfo('Reset pin_focus')
            GPIO.output(18, False)

            rospy.loginfo('Reset pin_shoot')
            GPIO.output(18, False)
            


            rospy.loginfo('Creating command_camera service.')
            rospy.Service('command_camera', CameraCommand, camera_conrtol_callback)
            rospy.loginfo('Created command_camera service.')


            #start to ping    
            while not rospy.is_shutdown():
                ping_camera()
                rospy.loginfo("Camera pinged. Sleeping for 5 mins.")
                rospy.sleep(duration=rospy.Duration(300))

        except rospy.ROSInterruptException as ex:
            rospy.logerr(ex)

        rospy.spin()
    except Exception as ex:
        rospy.logerr(ex)
        GPIO.cleanup()