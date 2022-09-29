This is an example of code for takeoff, move and land the a drone with PX4 and MAVROS.

- Simulation setup:

1- ROS Noetic:
http://wiki.ros.org/noetic/Installation/Ubuntu

2- Mavros: 
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins

3- PX4:
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

4- QGroundControl:
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html


- Launch the GAZEBO simulation:

cd ~/PX4-Autopilot
source /opt/ros/noetic/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch 


- Open QGroundControl to monitore the drone


- Run the example code:

python3 flight_example.py


- Code also tested with drone model S500 (Holybro), Pixhawk 4 and RaspberryPi on board:

ssh -C -Y robot@10.1.100.2 (user@raspberrypi_IP)
sudo chmod a+rw /dev/ttyACM0 (USB connection between Pixhawk and RaspberryPi)
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0 gcs_url:=udp://@10.1.100.3 (laptop's IP with QGroundControl)

 

