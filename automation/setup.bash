export ROS_DISTRO=melodic
sudo apt install ros-$ROS_DISTRO-turtlebot-*
sudo apt install git chrony -y
sudo apt install ros-$ROS_DISTRO-turtlebot-gazebo ros-$ROS_DISTRO-turtlebot-apps ros-$ROS_DISTRO-turtlebot-bringup ros-$ROS_DISTRO-arbotix ros-$ROS_DISTRO-yocs-velocity-smoother ros-$ROS_DISTRO-explore-lite ros-$ROS_DISTRO-robot-localization  ros-$ROS_DISTRO-map-server  ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-move-base  ros-$ROS_DISTRO-eband-local-planner 

sudo chmod 666 /dev/ttyUSB* 

sudo apt install ros-$ROS_DISTRO-rplidar-ros ros-$ROS_DISTRO-kobuki-* ros-$ROS_DISTRO-ecl-streams ros-$ROS_DISTRO-depthimage-to-laserscan ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-yocs-velocity-smoother


# necessary only for the robot
cd /dev &&
sudo chmod og+rwx gpio* &&
cd  &&

sudo chmod 666 /dev/ttyUSB*