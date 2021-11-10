sudo apt update -qq 

export ROS_DISTRO=melodic &&
sudo apt install ros-$ROS_DISTRO-turtlebot* ros-$ROS_DISTRO-astra-* -y &&
sudo apt install git chrony -y &&

sudo apt-get install ntpdate -y &&

sudo ntpdate ntp.ubuntu.com &&

sudo apt install ros-$ROS_DISTRO-turtlebot-gazebo &&
sudo apt install ros-$ROS_DISTRO-turtlebot-apps &&
sudo apt install ros-$ROS_DISTRO-turtlebot-bringup &&
sudo apt install ros-$ROS_DISTRO-arbotix &&
sudo apt install ros-$ROS_DISTRO-yocs-velocity-smoother &&
sudo apt install ros-$ROS_DISTRO-explore-lite &&
sudo apt install ros-$ROS_DISTRO-robot-localization &&
sudo apt install ros-$ROS_DISTRO-map-server &&
sudo apt install ros-$ROS_DISTRO-eband-local-planner &&


cd /dev &&
sudo chmod og+rwx gpio* &&
cd  &&

sudo chmod 666 /dev/ttyUSB*