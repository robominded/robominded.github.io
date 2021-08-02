# Install YdlidarX4 on Linux and run it with ROS
0. Assumption: recent version of ROS1 is installed 
1. Install driver 
  - `git clone https://github.com/YDLIDAR/YDLidar-SDK.git`
  - `cd YDLidar-SDK/build`
  - `cmake ..`
  - `make`
  - `sudo make install`
2. Clone the ydlidar_ros node 
  - `cd <ROS Workspace>/src`
  - `git clone https://github.com/YDLIDAR/ydlidar_ros.git`
  - `cd ..`
  - Run `catkin_make` to build the node
  - `source devel/setup.bash` [assuming you are using bash] 
  - `roscd ydlidar_ros/startup/`
  - `sudo chmod 777 ./*`
  - `sudo sh initenv.sh`
3. Run the lidar
  - Open Rviz to visualize the laser beams detection points `roslauch ydlidar_ros lidar_view.launch`
  - Run the lidar `roslaunch ydlidar_ros X4.launch` 
