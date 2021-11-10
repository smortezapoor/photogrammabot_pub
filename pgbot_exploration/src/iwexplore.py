#!/usr/bin/env python

import rospy
import actionlib
import tf
import numpy as np
import roslaunch
import rospkg

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid, Odometry
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from occupancy_grid_python import OccupancyGridManager
from geometry_msgs.msg import Pose, Point, PoseStamped
from pgbot_turret_control.srv import GoToPose
from pgbot_camera_control.srv import CameraCommand
from pgbot_kobuki_interface.msg import InterfaceButton
from kobuki_msgs.msg import ButtonEvent, Sound
import tf2_ros
import math
from std_srvs.srv import *
import json
import os

from kobuki_msgs.msg import Sound, Led

rviz_id = 0

shutdown_init = False

def shutdown():
    global move_base, command_turret_proxy, shutdown_init

    if shutdown_init == False:
        shutdown_init = True
        
        rospy.loginfo('Canceling all goals')
        move_base.cancel_all_goals()

        rospy.loginfo('Moving the turret to the home position')
        res_turret = command_turret_proxy.call('home')

        rospy.loginfo('Relaxing turret')
        res_turret = command_turret_proxy.call('relax')

        rospy.loginfo('Shutting down the RPi')
        os.system("shutdown now")


def go_to_point(x_target, y_target, drtn, theta_target=0):
    """ Move to a location relative to the indicated frame """
    global move_base

    print 'Inside go_to_point()'

    rospy.loginfo("navigating to: ({},{},{})".format(
        x_target, y_target, theta_target))

    goal = create_goal_message(x_target, y_target, theta_target, 'map')

    # start moving
    move_base.send_goal(goal)

    # allow TurtleBot up to 60 seconds to complete task
    success = move_base.wait_for_result(rospy.Duration(drtn))

    # if not successfull, cancel goal
    if not success:
        rospy.loginfo(
            "Canceling the target as it took more than {0} secs".format(drtn))
        move_base.cancel_goal()

    # output status
    state = move_base.get_state()
    rospy.loginfo("State      : {}".format(state))

    return state

def kobuki_buttons_callback(button_info):
    global flag_start_operation, flag_shutdown, flag_pause, move_base
    rospy.loginfo('Button pressed. - Btn{0} for {1} secs'.format(button_info.button, button_info.timeactive))

    if button_info.button == ButtonEvent.Button0:
        if button_info.timeactive < 2.0:
            flag_start_operation =  True
            rospy.loginfo('Setting {0} to {1}'.format('flag_start_operation', flag_start_operation))
        elif button_info.timeactive > 4.0 and button_info.timeactive < 20:
            flag_shutdown = True
    elif button_info.button == ButtonEvent.Button1:
        flag_pause = not flag_pause
        if flag_pause:
            move_base.cancel_all_goals()
        rospy.loginfo('Setting {0} to {1}'.format('flag_pause', flag_pause))
    elif button_info.button == ButtonEvent.Button2:
        rospy.loginfo('Starting shutdown procedure')
        shutdown()



        # cancel
        pass

def create_goal_message(x_target, y_target, theta_target, frame='map'):
    """Create a goal message in the indicated frame"""

    print 'Inside create_goal_message()'

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target

    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    return goal


def output_pic_to_rviz(array, scale, color, current_goal=False):

    print 'Inside output_to_rviz()'

    global publisher_waypoints, publisher_current_goal
    global rviz_id

    # make MarkerArray message
    markerArray = MarkerArray()

    # loop throgh all instances of the array
    for index in range(len(array)):
        marker = Marker()
        marker.id = rviz_id
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        duration = 0
        marker.lifetime = rospy.Duration(duration)
        marker.scale = scale
        marker.color = color
        # x and y are inverted due to nature of the map
        marker.pose.position.x = array[index][1]
        marker.pose.position.y = array[index][0]

        quat = tf.transformations.quaternion_from_euler(0, 0, array[index][2])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        markerArray.markers.append(marker)
        # incremment rviz_id
        rviz_id = rviz_id + 1

    # Publish the MarkerArray

    publisher_waypoints.publish(markerArray)


def output_to_rviz(array, scale, color, current_goal=False):

    print 'Inside output_to_rviz()'

    global publisher_waypoints, publisher_current_goal
    global rviz_id

    # make MarkerArray message
    markerArray = MarkerArray()

    # loop throgh all instances of the array
    for index in range(len(array)):
        marker = Marker()
        marker.id = rviz_id
        marker.header.frame_id = "map"
        marker.type = marker.CYLINDER
        marker.text = str(index)
        marker.action = marker.ADD
        duration = (120 if current_goal else 0.0)
        marker.lifetime = rospy.Duration(duration)
        marker.scale = Point(scale.x*1.5, scale.y*1.5, scale.z)
        marker.color = ColorRGBA(0, 0, 0, 0.8)
        # x and y are inverted due to nature of the map
        marker.pose.position.x = array[index][1]
        marker.pose.position.y = array[index][0]

        marker_text = Marker()
        marker_text.id = rviz_id + 1
        marker_text.header.frame_id = "map"
        marker_text.type = marker_text.TEXT_VIEW_FACING
        marker_text.text = str(index)
        marker_text.action = marker_text.ADD
        duration = (120 if current_goal else 0.0)
        marker_text.lifetime = rospy.Duration(duration)
        marker_text.scale = scale
        marker_text.color = color
        # x and y are inverted due to nature of the map
        marker_text.pose.position.x = array[index][1]
        marker_text.pose.position.y = array[index][0]
        marker_text.pose.position.z = 0.5

        marker_compen = Marker()
        marker_compen.id = rviz_id + 2
        marker_compen.header.frame_id = "map"
        marker_compen.type = marker_compen.ARROW
        marker_compen.action = marker_compen.ADD
        duration = 0
        marker_compen.lifetime = rospy.Duration(duration)
        marker_compen.scale = Point(scale.x*1.5, scale.y*.5, scale.z*.5)
        marker_compen.color = ColorRGBA(0, 0, 0, 0.8)
        # x and y are inverted due to nature of the map
        marker_compen.pose.position.x = array[index][1]
        marker_compen.pose.position.y = array[index][0]

        quat = tf.transformations.quaternion_from_euler(0, 0, array[index][2])
        marker_compen.pose.orientation.x = quat[0]
        marker_compen.pose.orientation.y = quat[1]
        marker_compen.pose.orientation.z = quat[2]
        marker_compen.pose.orientation.w = quat[3]

        markerArray.markers.append(marker_compen)

        markerArray.markers.append(marker)
        markerArray.markers.append(marker_text)
        # incremment rviz_id
        rviz_id = rviz_id + 3

    # Publish the MarkerArray
    if current_goal:
        publisher_current_goal.publish(markerArray)
    else:
        publisher_waypoints.publish(markerArray)


def controller(goals):
    global move_base, publisher_waypoints, tfBuffer, command_turret_proxy, command_camera_proxy, clear_costmap, camera_poses, flag_pause
    if not rospy.is_shutdown():

        rospy.loginfo('Inside controller()')

        missing_goals = 0
        last_goal_missed = False

        color = ColorRGBA()
        dimention = 0.15

        # # frontiers
        # scale needs [x, y, z] atributes
        scale = Point(dimention * 2, dimention * 2, dimention * 2)

        # color nneds [r, g, b, a] atributes
        color = ColorRGBA(1.0, 0.0, 0.0, 0.80)
        output_to_rviz(goals, scale, color)

        for goal in goals:

            while flag_pause:
                rospy.loginfo("In pause state")
                rospy.sleep(1)

            current_goal = []
            current_goal.append(goal)
            scale_current = Point(dimention*1.5, dimention*1.5, dimention*1.5)
            output_to_rviz(current_goal, scale_current, ColorRGBA(
                0.0, 0.0, 1.0, 0.80), current_goal=True)

            goal_theta = goal[2]

            clear_costmap()

            status = go_to_point(goal[1], goal[0], 0, goal_theta)

            color_final = ColorRGBA()
            dimention = 0.35

            # # frontiers
            # scale needs [x, y, z] atributes
            scale_final = Point(dimention, dimention, dimention)

            if status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal achieved')
                last_goal_missed = False
                missing_goals = 0

                trans_map_robot = None
                while trans_map_robot == None:

                    try:
                        trans_map_robot = tfBuffer.lookup_transform(
                            'map', 'base_link', rospy.Time())
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr('Cannot find necessary map > robot tf')

                    # Take photo

                    quart = trans_map_robot.transform.rotation
                    eul = tf.transformations.euler_from_quaternion(
                        [quart.x, quart.y, quart.z, quart.w])
                    compensation = goal_theta - eul[2]
                    compensation_deg = math.degrees(compensation)

                    for camera_pose in camera_poses:

                        rospy.loginfo('Taking picture from pan: {0}, tilt: {1} with compensation {2}'.format(
                            camera_pose['pan'], camera_pose['tilt'], compensation_deg))
                        _final_pan = camera_pose['pan'] + compensation_deg
                        res_turret = command_turret_proxy.call(
                            'manual|{0}|{1}'.format(_final_pan, camera_pose['tilt']))

                        if res_turret.res == 0:
                            send_sound(2)

                            res_camera = command_camera_proxy.call(30)

                            current_photo = []
                            current_photo.append(
                                [trans_map_robot.transform.translation.y, trans_map_robot.transform.translation.x, math.radians(_final_pan) + eul[2] - (math.pi/2.0) ])
                            output_pic_to_rviz(array=current_photo, scale=Point(
                                1.0, 0.1, 0.1), color=ColorRGBA(0.0, 1.0, 0.0, 0.80), current_goal=False)
                        else:
                            pass  # error

                        if res_turret.res == 0 and res_camera.result == 0:
                            rospy.loginfo('Picture taken successfully')
                    # Take photo end

                # color nneds [r, g, b, a] atributes
                color_final = ColorRGBA(0.0, 1.0, 0.0, 0.80)

            else:
                rospy.loginfo('Goal missed')
                missing_goals += 1
                last_goal_missed = True

                if last_goal_missed and missing_goals > 10:
                    set_led1(Led.RED)

                # color nneds [r, g, b, a] atributes
                color_final = ColorRGBA(1.0, 0.0, 0.0, 0.80)

            command_turret_proxy.call('forward0')

            output_to_rviz(current_goal, scale_final, color_final)

            rospy.sleep(2)


def explore_callback(frontiers):
    global time_log_explore, time_log_move_base, number_of_frontiers
    time_log_explore = rospy.get_time()
    rospy.loginfo("Explore message received at: {0}".format(time_log_explore))

    number_of_frontiers = len(frontiers.markers)/2
    rospy.loginfo("The current frontier count: {0}".format(
        number_of_frontiers))


def explore_move_base_result_callback(result):
    global time_log_explore, time_log_move_base
    time_log_move_base = rospy.get_time()
    rospy.loginfo(
        "Move_base result message received at: {0}".format(time_log_move_base))


def create_goals():

    goals = []
    # Subscribe to the nav_msgs/OccupancyGrid topic
    ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                               subscribe_to_updates=False)  # default False

    size_x = 40
    size_y = 40

    g_matrix = [[0 for x in range(size_x)] for y in range(size_y)]
    g_world_matrix = [[0 for x in range(size_x)] for y in range(size_y)]

    for x in range(0, size_x):
        for y in range(0, size_y):
            cost = -1
            theta_position_compensation = 0

            final_xy = x - (size_x / 2), y - (size_y / 2)
            theta_position_compensation = -1

            try:
                cost = ogm.get_cost_from_world_x_y(final_xy[0], final_xy[1])
            except Exception as ex:
                pass

            if(cost == 0):
                g_matrix[x][y] = True
                g_world_matrix[x][y] = (
                    final_xy[1], final_xy[0], theta_position_compensation)

            else:
                g_matrix[x][y] = False
                g_world_matrix[x][y] = (
                    final_xy[1], final_xy[0], theta_position_compensation)

    min_planned_x = -1
    max_planned_x = -1

    min_planned_y = -1
    max_planned_y = -1

    for planned_x in range(len(g_matrix)):
        for planned_y in range(len(g_matrix[0])):
            if min_planned_x == -1 and g_matrix[planned_x][planned_y] == True:
                min_planned_x = planned_x

            if g_matrix[planned_x][planned_y] == True:
                max_planned_x = planned_x

            if min_planned_y == -1 and g_matrix[planned_x][planned_y] == True:
                min_planned_y = planned_y

            if g_matrix[planned_x][planned_y] == True:
                max_planned_y = planned_y

    avg_planned_x = (1.0 * max_planned_x + min_planned_x) / 2.0
    avg_planned_y = (1.0 * max_planned_y + min_planned_y) / 2.0

    range_planned_x = max_planned_x - min_planned_x

    quart_1_planned_x = (min_planned_x, math.floor(
        range_planned_x/4.0) + min_planned_x)
    quart_2_planned_x = (quart_1_planned_x[1], avg_planned_x)
    quart_3_planned_x = (quart_2_planned_x[1], math.ceil(
        3 * range_planned_x / 4.0) + min_planned_x)
    quart_4_planned_x = (quart_3_planned_x[1], max_planned_x)

    range_planned_y = max_planned_y - min_planned_y

    quart_1_planned_y = (min_planned_y, math.floor(
        range_planned_y/4.0) + min_planned_y)
    quart_2_planned_y = (quart_1_planned_y[1], avg_planned_y)
    quart_3_planned_y = (quart_2_planned_y[1], math.ceil(
        3 * range_planned_y / 4.0) + min_planned_y)
    quart_4_planned_y = (quart_3_planned_y[1], max_planned_y)

    for planned_x in range(len(g_matrix)):
        for planned_y in range(len(g_matrix[0])):
            dest_x = planned_x
            dest_y = (planned_y if planned_x %
                      2 == 0 else (size_y - planned_y - 1))

            if g_matrix[dest_x][dest_y] == True:

                diff_x = planned_x - avg_planned_x
                diff_y = planned_y - avg_planned_y

                arc1 = (diff_x if 1.0 * diff_x != 0 else diff_x + 0.0001)
                arc2 = (diff_y if 1.0 * diff_y != 0 else diff_y + 0.0001)

                angle = 0

                # Set the angle of the point here. This is the zero of the robot orientation when standing on a goal

                to_add = (g_world_matrix[dest_x][dest_y][0],
                          g_world_matrix[dest_x][dest_y][1], angle)

                goals.append(to_add)

    return goals


def create_explore_node():
    rp = rospkg.RosPack()
    package_path = rp.get_path('pgbot_exploration')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid, [package_path + "/launch/explore_lite.launch"])
    launch.start()

    return launch


def send_sound(sound_num, count=1):
    # ON = 0
    # OFF = 1
    # RECHARGE = 2
    # BUTTON = 3
    # ERROR = 4
    # CLEANINGSTART = 5
    # CLEANINGEND = 6
    global sound_publisher

    sound = Sound()
    sound.value = sound_num

    for i in range(0, count):
        sound_publisher.publish(sound)
        rospy.sleep(1)


def set_led1(led_num):
    global led1_publisher, rospy
    led = Led()
    led.value = led_num
    led1_publisher.publish(led)
    rospy.loginfo('Changed the color of LED1 to {0}'.format(led_num))

def set_led2(led_num):
    global led2_publisher, rospy
    led = Led()
    led.value = led_num
    led2_publisher.publish(led)
    rospy.loginfo('Changed the color of LED2 to {0}'.format(led_num))

def explore():
    global time_log_explore, time_log_move_base, number_of_frontiers, flag_pause
    rate = rospy.Rate(float(1))

    insurrance_time = 25

    exploration_attempts = 2

    while exploration_attempts != 0:
        initialized = False

        while flag_pause:
            rospy.loginfo("In pause state")
            rospy.sleep(1)

        rospy.loginfo("Running clear costmaps")
        clear_costmap()
        rospy.loginfo("Run clear costmaps")

        rospy.loginfo("Running explore node")
        explore_process = create_explore_node()
        rospy.loginfo("Run explore node")

        rospy.sleep(10)

        while not rospy.is_shutdown():

            if time_log_explore != -1 and time_log_move_base != -1 and number_of_frontiers != None:
                if not initialized:
                    rospy.loginfo(
                        "Started to monitor the explore_lite behavior")
                    initialized = True

                current_time = rospy.get_time()

                proceeding_time = max(
                    time_log_explore, time_log_move_base) + insurrance_time

                rospy.loginfo("Current time: {0}, Frontier time: {1}, Move base time: {2}".format(
                    current_time, time_log_explore, time_log_move_base))

                if current_time > proceeding_time:
                    rospy.loginfo(
                        "The exploration was idle for {0} seconds. Proceeding to next phase.")
                    break

                if flag_pause:
                    rospy.loginfo("In pause state")
                    break

            rate.sleep()

        rospy.loginfo("Terminating explore node")
        explore_process.shutdown()
        exploration_attempts -= 1


if __name__ == '__main__':

    try:
        # initialyze node
        rospy.init_node('iw_explore_node')
        
        
        flag_shutdown = False
        flag_start_operation = False
        flag_pause = False        
        
        
        publisher_waypoints = rospy.Publisher(
            'visualization_marker_array_waypoints', MarkerArray, queue_size=10000)
        publisher_current_goal = rospy.Publisher(
            'visualization_marker_array_current_goal', MarkerArray, queue_size=1)
        # initialyze listener
        # tflistener = tf.TransformListener()

        camera_poses_str = rospy.get_param('~camera_poses')
        if camera_poses_str is None or camera_poses_str is '':
            rospy.logerr('Camera poses are not found. Add CAMERA_POSES to environment variables.')
        rospy.loginfo(camera_poses_str)

        camera_poses = json.loads(camera_poses_str)
        rospy.loginfo(camera_poses[0])

        sim = rospy.get_param('~sim')
        tfBuffer = tf2_ros.Buffer()
        tflistener = tf2_ros.TransformListener(tfBuffer)

        rospy.loginfo("Server connection create")
        # initialyze action client
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        move_base.wait_for_server()
        rospy.loginfo("Server connection to move_base stablished")

        time_log_explore = rospy.get_time()
        time_log_move_base = rospy.get_time()
        number_of_frontiers = -1
        rospy.Subscriber("/explore/frontiers", MarkerArray, explore_callback)
        rospy.Subscriber("/move_base/current_goal", PoseStamped,
                         explore_move_base_result_callback)

        
        rospy.Subscriber("/pgbot_kobuki_interface/buttons", InterfaceButton, kobuki_buttons_callback)
        rospy.loginfo("Subscriber to interface registered")

        rospy.loginfo(
            'Waiting for the pgbot_turret_pose service to come online')
        rospy.wait_for_service('/pgbot_turret_pose')

        rospy.loginfo('pgbot_turret_pose service is online')
        command_turret_proxy = rospy.ServiceProxy(
            '/pgbot_turret_pose', GoToPose)

        rospy.loginfo(
            'Waiting for the command_camera service to come online')
        rospy.wait_for_service('/command_camera')

        rospy.loginfo('pgbot_turret_pose service is online')
        command_camera_proxy = rospy.ServiceProxy(
            '/command_camera', CameraCommand)

        sound_publisher = rospy.Publisher(
            '/mobile_base/commands/sound', Sound, queue_size=10)

        led1_publisher = rospy.Publisher(
            '/mobile_base/commands/led1', Led, queue_size=1)

        led2_publisher = rospy.Publisher(
            '/mobile_base/commands/led2', Led, queue_size=1)

        rospy.loginfo('clear_costmap service is online')
        clear_costmap = rospy.ServiceProxy(
            '/move_base/clear_costmaps', Empty)

        rospy.loginfo(
            'Waiting for the pgbot_turret_pose service to come online')
        rospy.wait_for_service('/move_base/clear_costmaps')

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # what function to call when you ctrl + c


        rate = rospy.Rate(1)
    
        rate.sleep()
        rate.sleep()
        rate.sleep()

        set_led1(Led.GREEN)

        while not rospy.is_shutdown():
            if sim or flag_start_operation == True:
                rospy.loginfo("Start signal received")
                send_sound(Sound.OFF, 3)
                break


            send_sound(2)
            rospy.loginfo("Waiting for start signal")
            rate.sleep()

        set_led1(Led.ORANGE)
        set_led2(Led.ORANGE)

        explore()


        set_led2(Led.RED)

        goals = create_goals()

        controller(goals)

        set_led1(Led.GREEN)
        set_led2(Led.GREEN)


    except rospy.ROSInterruptException as ex:
        rospy.logfatal(ex)

    rospy.sleep(1)
    rospy.spin()
