#!/usr/bin/env python
import rospy
from kobuki_msgs.msg import ButtonEvent
from pgbot_kobuki_interface.msg import InterfaceButton
import roslaunch

button0 = {}
button1 = {}
button2 = {}

buttons = [button0, button1, button2]

def button_callback(btnEv):
    buttons[btnEv.button]['state'] = btnEv.state

    if btnEv.state == btnEv.PRESSED:
        buttons[btnEv.button]['pressed_time'] = rospy.get_time()
    elif btnEv.state == btnEv.RELEASED:
        release_time = rospy.get_time()
        timediff = release_time - buttons[btnEv.button]['pressed_time']
        decision(btnEv.button, timediff)


def decision(btn_num, timediff):
    global button_publisher
    rospy.loginfo('btn: {0}, timedif: {1}'.format(btn_num, timediff))
    btnMsg = InterfaceButton()
    btnMsg.button = btn_num
    btnMsg.timeactive = timediff
    button_publisher.publish(btnMsg)

    if btn_num == ButtonEvent.Button0:
        pass
    elif btn_num == ButtonEvent.Button1:
        pass
    elif btn_num == ButtonEvent.Button2:
        pass


if __name__ == '__main__':
    rospy.init_node('pgbot_kobuki_interface', anonymous=True)
    rospy.loginfo(' Node pgbot_kobuki_interface started to work')

    try:
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_callback)
        button_publisher = rospy.Publisher('/pgbot_kobuki_interface/buttons', InterfaceButton, queue_size=10)


    except rospy.ROSInterruptException:
        pass

    rospy.spin()