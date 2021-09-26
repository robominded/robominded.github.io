# Save a Map in ROS Noetic or Melodic
0. Assuming a map is generated. Follow these [instractions](https://github.com/robominded/robominded.github.io/blob/main/tutorials/Run%20Hector%20Slam%20on%20Jetson%20Nano.md) to generate one if needed.
1. **For ROS Noetic:** Install the needed packages `sudo apt install ros-noetic-map-server`
2. **For ROS Melodic:** Install the needed packages `sudo apt install ros-Melodic-map-server`
3. Make a map folder `mkdir ~/catkin_ws/maps`
4. `cd ~/catkin_ws/maps`
5. Save the map `run map_server map_saver -f map_name`
6. Stop all the ROS processes. Press CTRL + C on all terminal windows. 
# Load the Map
1. make sure that ros master is running if not run `roscore` in a terminal
2. open new terminal window and load the map using `rosrun map_server map_server map_name.yaml
3. to visulaize the map open Rviz. In a new terminal run `rviz`
4. From the Desplays panel in the Rviz application click Add then select Map and click ok. 
5. the map should show up in Rviz. 
**Note:** you can edit the `map_name.pgm` file using any image editting application (e.g., gimp) to enhance the quality of the map. 
