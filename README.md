This is an example of code for takeoff, move and land the drone with PX4 and MAVROS.

Simulation setup:

1- ROS Noetic:
http://wiki.ros.org/noetic/Installation/Ubuntu

2- Mavros: 

```
$ sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ chmod a+x install_geographiclib_datasets.sh
$ ./install_geographiclib_datasets.sh
$ sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins
```

3- PX4:

```
$ git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

4- QGroundControl:
- https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

5- Launch the GAZEBO simulation:

```
$ cd ~/PX4-Autopilot
$ source /opt/ros/noetic/setup.bash
$ DONT_RUN=1 make px4_sitl_default gazebo
$ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
$ roslaunch px4 mavros_posix_sitl.launch 
```

6- Open QGroundControl to monitore the drone

7- Run the example code:

```
$ python3 flight_example.py
```

8- Code also tested with drone model S500 (Holybro), Pixhawk 4 and RaspberryPi on board:

```
$ ssh -C -Y robot@10.1.100.2 (user@raspberrypi_IP)
$ sudo chmod a+rw /dev/ttyACM0 (USB connection between Pixhawk and RaspberryPi)
$ roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0 gcs_url:=udp://@10.1.100.3 (laptop's IP with QGroundControl)
```

or, based on mavproxy:

```
$ sudo mavproxy.py --master=/dev/serial0 --out=udp:192.168.43.109:14550 --baudrate 57600 (RX/TX connection and laptop's IP with QGroundControl)
```

## Controller/Commander nodes

```
$ roslaunch my_package my_launch.launch
```

## Docker

1 - In Terminal 1, run:

```
$ xhost local:root
```

2- In Terminal 1, run the Gazebo Simulation:

```
docker run -it \
    --name=ct1 \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    viniciusabrao/px4_drone:v0 \
    bash -c 'source /opt/ros/noetic/setup.bash \
    && source /PX4-Autopilot/Tools/setup_gazebo.bash /PX4-Autopilot /PX4-Autopilot/build/px4_sitl_default \
    && export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/PX4-Autopilot:/PX4-Autopilot/Tools/sitl_gazebo \
    && roslaunch px4 mavros_posix_sitl.launch' 
```

2- Open QGroundControl (local installation) and configure the mission as in the video below (from minute 1:32 to 2:13):

https://www.youtube.com/watch?v=T_qq-OQP8CQ

3- Run the Controller/Commander nodes:

```
$ docker exec -it ct1 bash -c 'source devel/setup.bash && roslaunch my_package my_launch.launch'
```

4- After the simulation, stop and delete the docker image:

```
$ docker kill $(docker ps -aq) 

$ docker container prune -f 

$ docker rmi -f viniciusabrao/px4_drone:v0
```

