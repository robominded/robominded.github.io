0. To get the lidar to work follow https://github.com/robominded/robominded.github.io/blob/main/tutorials/rplidar_ros.md
1. **On Ubuntu 18.04** run `sudo apt-get install qt4-qmake qt4-dev-tools` in terminal to install qt4: a tool to generate graphical user interfaces (GUIs). 
2. **On Ubuntu 20.04** qt4 was removed from Ubuntu 20.04. Therefore we need to add the PPA rock-core/qt4 manually 
<br> `sudo add-apt-repository ppa:rock-core/qt4` and then `sudo apt update`. 
<br> Then run `sudo apt-get install qt4-qmake qt4-dev-tools`
2.  `cd ~/catkin_ws/src`
3.  **On Ubuntu 18.04** `git -b melodic-devel clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`
4.  **On Ubuntu 20.04** `git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`
5.  Set the corrdinate Frame parameters according to http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot
    - `vim ~/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch`
    - replace `<arg name="base_frame" default="base_footprint"/>` with <arg name="base_frame" default="base_link"/>` 
    - replace `<arg name="odom_frame" default="nav"/>` with `<arg name="odom_frame" default="base_link"/>`
    - replace `<!--<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/>-->` with `<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100"/>`
6. `vim ~/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch`
    - repalce  `<param name="/use_sim_time" value="true"/>` with `<param name="/use_sim_time" value="false"/>
7.  **On Ubuntu 18.04** I had to create a symbolic link before being able to build the catkin workspace
     - `cd /usr/include` and `sudo ln -s opencv4/ opencv` 
8. `cd ~/catkin_ws` 
9. `catkin_make`
10. spin the lidar with `roslaunch rplidar_ros rplidar.launch`
11. get Hector slam to work with `roslaunch hector_slam_launch tutorial.launch`
12. Next Saving the map
