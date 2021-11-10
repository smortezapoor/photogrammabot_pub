export ROS_DISTRO=melodic
sudo apt install ros-$ROS_DISTRO-turtlebot* ros-$ROS_DISTRO-astra-* -y
sudo apt install git chrony -y

mkdir ~/tmp
cd ~/tmp
wget https://raw.githubusercontent.com/orbbec/astra/master/install/orbbec-usb.rules
sudo cp orbbec-usb.rules /etc/udev/rules.d/.



sudo apt install ros-$ROS_DISTRO-turtlebot-gazebo
sudo apt install ros-$ROS_DISTRO-turtlebot-apps
sudo apt install ros-$ROS_DISTRO-turtlebot-bringup
sudo apt install ros-$ROS_DISTRO-arbotix
sudo apt install ros-$ROS_DISTRO-yocs-velocity-smoother
sudo apt install ros-$ROS_DISTRO-explore-lite
sudo apt install ros-$ROS_DISTRO-robot-localization &&
sudo apt install ros-$ROS_DISTRO-map-server &&
sudo apt install ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-move-base &&
sudo apt install ros-$ROS_DISTRO-eband-local-planner &&


sudo chmod 666 /dev/ttyUSB* 

sudo apt install ros-$ROS_DISTRO-rplidar-ros