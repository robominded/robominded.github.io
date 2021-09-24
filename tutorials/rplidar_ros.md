# Installing RPLidar A1/A2 ROS Node

1. clone rplidar_ros repo into catkin_workspace/src <br> `git clone https://github.com/Slamtec/rplidar_ros.git`
2. `cd ~/catkin_workspace`
3. `catkin_make` [assuming you are using ROS1]
4. Make sure you have a read/write permissions on the usb <br> `ls -l /dev | grep ttyUSB*` should output something like `crw-rw-rw- 1 root dialout 188, 0 Sep 23 22:11 /dev/ttyUSB0`
5. If the `ttyUSB0` permissions look like this `crw-rw----` run the create_udev_rules.sh script in rplidar_ros/scripts/ folder. 
6. unplug and plug the lidar USB connections
7. run `roslaunch rplidar_ros rplidar.launch`. Now if you list ros topics bu executing `rostopic list` you should see a `/scan` topic in the list. 
8. To visualize the laser scan run `rviz`
9. In Rviz Displays window 
   . change Fixed Frame `map` to Fixed Frame `laser`
   . click on add and select laserScan. Then click ok. 
   . expand LaserScan and select `/scan` in the topic field
10. you can replace step 7 to 9 by running `roslaunch rplidar_ros view_rplidar.launch`
