# Mobile Autonomous Robot Vehicle for Investigation and Navigation

## Introduction
MARVIN is a comprehensive platform (built by Luis Andrade Proaño) designed to facilitate the development and deployment of autnomous vehicles with Ackermann steering , providing a set of essential modules and tools to enable various capabilities. The framework consists of the following core components:
- Manual control over the vehicle.
- Real time mapping of the enviroment sourrounding the robot.
- Localization once the map is saved in the robots system. 
- Autonomous navigation of the vehicle with object avoidance.
- Simulation enviroment designed for gazebo.
- Graphical User Interface (GUI).
- Code developed for ROS2 foxy

MARVIN is designed to be versatile, enabling researchers and developers to experiment with different robot types with the same steering system, test various algorithms, and to navigate complex enviroments  in real-world scenarios. By providing these core modules and simulation environments, MARVIN is another tool for investigation in the field of robotics.

## Requirements:

- Operating System: Ubuntu Linux Jammy Jellyfish (22.04)
- Jetson Nano Developer Kit, Jetson Nano 4Gb
- Operating System: Ubuntu Linux Bionic Beaver 18.04
- For the simulation consider according the computational requirements .

## Dependencies:
For the simulation:
- ROS2 Foxy. Installation: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html 
- Verify gazebo installation. 
- Verify the acquisition of the gazebo controls library. If it is not installed:
```sh
sudo apt-get install ros-foxy-gazebo-ros2-control
```

## Installation for the simulation
1.- Clone the github repository in a workspace source folder. (marvin_ws/src)
```sh
git clone https://github.com/RAMEL-ESPOL/Chappy.git
```
2.- Go to workspace directory and Install dependencies
```sh
cd ..
rosdep install -y --from-paths src --ignore-src
```
3.- Build the pacakge.
```sh
colcon build
```
## Usage
0.- Navigate to the workspace directory (The previous step to perform in all new cmd windows opened)
```sh
source install/setup.bash
```
1.- Launch the simulation.
```sh
ros2 launch marvin_sim_gazebo gazebo_spawn.launch.py world:=./src/marvin_sim_gazebo/worlds/obstacles.world use_sim_time:=true
```
2.- Open a new cmd window, source in the ws and launch SLAM.
```sh
ros2 launch marvin_navigation slam.launch.py use_sim_time:=true
```
In Rviz activate the map node and spec

3.- Open a new cmd window, source in the ws and launch the controller.
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard  
```
After you've mapped the area, go in Rviz to Panels -> add new Panel and click on SlamToolboxPlugin
Make sure that you save the map with the name "map_serial"

4.- After saving, finish with Ctrl-c the window where is you where running SLAM and in Rviz don't forget to Remove the Map plugin

5.- Open a new cmd window, source in the ws and launch the Navigation.
```sh
ros2 launch marvin_navigation navigation.launch.py use_sim_time:=true
```
To make the robot navigate autonomously use the 2D Goal Pose tool in Rviz to select a point in the map.

You can now move the robot freely through the map. You can also (during the path) use manual control to change directions, but the robot will stop at the Goal that you previously set.


## Installation for the Jetson Nano
1.- Clone the github repository in a workspace source folder. (marvin_ws/src)
```sh
git clone https://github.com/RAMEL-ESPOL/Chappy.git
```
2.- Go to workspace directory /src and initialize the DockerFile (dont worry, the factory image of the Jetson Nano comes with Docker pre-installed).
```sh
cd ..
docker build . -t marvin:latest
```
After the docker has been built, you have to run it with the following conditions
3.- Build the pacakge.
```sh
docker run -it --device=/dev/myserial --device=/dev/rplidar --device=/dev/video0 --device=/dev/video1 --device=/dev/video2 --device=/dev/video3 --device=/dev/video4 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix marvin:latest /bin/bash
```
This will let you run the Docker and create a container to work in. 

Exit the docker with the "exit" command. 
Now that the container is created, all the files and changes that you've made, will remain there.
To open the container you must first now its name. 
Get it with: 
```sh
docker ps -a
```
The last name is the last container that was active. 
4.- Start the container
```sh
docker start <container name> 
```
5.- Run the container
```sh
docker run exec -it <container name> /bin/bash
```

##Quality of life improvements 
For an easier time running the Jetson nano, first enable remote control (the following video gives the proper instrucions): 

```sh
https://www.youtube.com/watch?v=Rgmw6kPmhXI
```
After enabling the Jetson Nano for remote control, download from the main page "RealVNC Viewer"

Everytime you open a new cmd window, you must execute the running container to acces it
```sh
docker run exec -it <container name> /bin/bash
```

## Usage
0.- Inside the container you'll be in the MARVIN workspace already built. (Important step to perform in all new cmd windows opened)
```sh
source install/setup.bash
```
1.- Launch the control and LiDAR of the robot
```sh
ros2 launch marvin_car_bringup marvin_car_bringup.launch.py
```

2.- Open a new cmd window, source in the ws and launch SLAM.
```sh
ros2 launch marvin_car_lidar gmapping.launch.py
```
In Rviz activate the map node and spec

3.- Open a new cmd window, source in the ws and launch the controller.
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard  
```
After you've mapped the area.
4.- Save your map
```sh
ros2 launch marvin_car_nav save_map_launch.py  
```
Close the Map Plugin and the cmd Windows that's running the mapping
5.- Launch the Navigation.
```sh
ros2 launch marvin_car_nav navigation_dwa_launch.py
```
To make the robot navigate autonomously use the 2D Pose Estimate to select the point in the map where the real robot is, drag the click to specify its orientation.
Use the 2D Goal Pose tool in Rviz to select a point in the map.

You can now move the robot freely through the map.


