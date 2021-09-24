# Installing RPLidar A1/A2 ROS node

1. clone rplidar_ros repo into catkin_workspace/src <br> `git clone https://github.com/Slamtec/rplidar_ros.git`
2. `cd ~/catkin_workspace`
3. `catkin_make` [assuming you are using ROS1]
4. Make sure you have a read/write permissions on the usb <br> `ls -l /dev | grep ttyUSB*` should output something like `crw-rw-rw- 1 root dialout 188, 0 Sep 23 22:11 /dev/ttyUSB0`
5. If the `ttyUSB0` permissions look like this `crw-rw----` run the create_udev_rules.sh script in rplidar_ros/scripts/ folder. 
6. run `roslauch rplidar_ros rplidar.launch`. Now if you list ros topics bu executing `rostopic list` you should see a `/scan` topic in the list. 
7. To visualize the laser scan run `roslaunch rplidar_ros view_rplidar.launch`
