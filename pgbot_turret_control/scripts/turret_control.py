#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_msgs.srv import SetSpeed
from pgbot_turret_control.srv import CommandTurret
from sensor_msgs.msg import JointState
import math

pan_horn_joint_state = {}
tilt_horn_joint_state = {}

def joint_states_callback(data):
    global pan_horn_joint_state, tilt_horn_joint_state
    if data.name == []:
        rospy.logerr("No robot_state messages received!\n")
    
    if ('pan_horn_joint' not in data.name and 'tilt_horn_joint' not in data.name):
        return

    if('pan_horn_joint' in data.name):
        index = data.name.index('pan_horn_joint')
        pan_horn_joint_state['position'] = data.position[index]

        pan_horn_joint_state['velocity'] = data.velocity[index]

    if('tilt_horn_joint' in data.name):
        index = data.name.index('tilt_horn_joint')
        tilt_horn_joint_state['position'] = data.position[index]

        tilt_horn_joint_state['velocity'] = data.velocity[index]

def turret_pose_callback(data):
    global pan_publisher, tilt_publisher, pan_speed_proxy, tilt_speed_proxy, return_joint_state_proxy, pan_relax_proxy, tilt_relax_proxy
    global pan_state_publisher, tilt_state_publisher, pan_velocity_publisher, tilt_velocity_publisher, degree_tole

    if(data.pan == -1 or data.tilt == -1):
        pan_relax_proxy.call()
        tilt_relax_proxy.call()
        rospy.loginfo("Relax mode activated")
        return 0

    pan_rad = math.radians(data.pan)
    tilt_rad = math.radians(data.tilt)
    rospy.loginfo('New message received. Sending turret to pan: {0}, tilt: {1} with speed {2}'.format(pan_rad, tilt_rad, data.speed))
    
    
    pan_speed_proxy(data.speed)
    tilt_speed_proxy(data.speed)

    rospy.sleep(rospy.Duration(0.5))

    tilt_publisher.publish(0)

    rospy.sleep(rospy.Duration(1))

    pan_publisher.publish(pan_rad)

    rospy.sleep(rospy.Duration(1))

    tilt_publisher.publish(tilt_rad)



    reached = -1
    unreachable = False
    counter = 0

    while (reached != 0 and not unreachable):
        try:
            counter+=1
            
            pan_current_rad =   pan_horn_joint_state['position']
            tilt_current_rad =  tilt_horn_joint_state['position']
            pan_current = math.degrees(pan_current_rad)
            tilt_current = math.degrees(tilt_current_rad)
            pan_current_vel =   pan_horn_joint_state['velocity']
            tilt_current_vel =  tilt_horn_joint_state['velocity']

            pan_state_publisher.publish(pan_current)
            pan_velocity_publisher.publish(pan_current_vel)
            tilt_state_publisher.publish(tilt_current)
            tilt_velocity_publisher.publish(tilt_current_vel)

            rospy.loginfo('Pan {0} --> {1}  -- Tilt {2} --> {3}'.format(pan_current, data.pan, tilt_current, data.tilt))

            if(abs(data.pan - pan_current) < degree_tole and abs(data.tilt - tilt_current) < degree_tole):
                reached = 0
                rospy.loginfo('Goal reached!')
            elif pan_current_vel == 0 and tilt_current_vel == 0:
                pan_publisher.publish(pan_rad)
                tilt_publisher.publish(tilt_rad)
            
            rospy.sleep(rospy.Duration(0.5))
            if(counter == 30):
                break
        except Exception as e:
            print e
            unreachable = True

    res = (-2 if unreachable else reached)

    return reached

if __name__ == '__main__':
    rospy.init_node('pgbot_turret_control', anonymous=True)
    rospy.loginfo(' Node pgbot_turret_control started to work')

    try:

        degree_tole = float(rospy.get_param('~degree_tole'))

        pan_publisher = rospy.Publisher('/pan_horn_joint/command', Float64, queue_size=1)
        pan_state_publisher = rospy.Publisher('/pan_horn_joint/state', Float64, queue_size=10)
        pan_velocity_publisher = rospy.Publisher('/pan_horn_joint/velocity', Float64, queue_size=10)
        tilt_publisher = rospy.Publisher('/tilt_horn_joint/command', Float64, queue_size=1)
        tilt_state_publisher = rospy.Publisher('/tilt_horn_joint/state', Float64, queue_size=10)
        tilt_velocity_publisher = rospy.Publisher('/tilt_horn_joint/velocity', Float64, queue_size=10)

        rospy.loginfo('Waiting for the servo services to come online')

        rospy.wait_for_service('/pan_horn_joint/set_speed')
        rospy.wait_for_service('/pan_horn_joint/relax')
        rospy.loginfo('Pan (ID1) service online')

        rospy.wait_for_service('/tilt_horn_joint/set_speed')
        rospy.wait_for_service('/tilt_horn_joint/relax')
        rospy.loginfo('Tilt (ID2) service online')

        pan_speed_proxy = rospy.ServiceProxy('/pan_horn_joint/set_speed', SetSpeed)
        pan_relax_proxy = rospy.ServiceProxy('/pan_horn_joint/relax', Relax)
        tilt_speed_proxy = rospy.ServiceProxy('/tilt_horn_joint/set_speed', SetSpeed)
        tilt_relax_proxy = rospy.ServiceProxy('/tilt_horn_joint/relax', Relax)
        rospy.Subscriber('joint_states', JointState, joint_states_callback)


        rospy.loginfo('Creating command_turret service.')
        rospy.Service('command_turret', CommandTurret, turret_pose_callback)
        rospy.loginfo('Created command_turret service.')

    except rospy.ROSInterruptException:
        pass

    rospy.spin()