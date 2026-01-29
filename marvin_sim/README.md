# MARVIN Simulation Environment

This guide covers the setup and execution steps for the MARVIN simulation using Gazebo and ROS2 Humble.

## Software Requirements
- **Operating System:** Ubuntu Linux Jammy Jellyfish LTS (22.04.3)
- **ROS Distribution:** ROS2 Humble

## Prerequisites

1.  **Update system and install git:**
    ```sh
    sudo apt update
    sudo apt install git
    ```

2.  **Install ROS2 Humble:**
    Follow the official guide: [Ubuntu Install Debians](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    
    Add sourcing to your shell:
    ```sh
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

3.  **Install Gazebo & Dependencies:**
    ```sh
    sudo apt install ros-humble-gazebo-ros-pkgs
    sudo apt install ros-humble-xacro
    sudo apt install ros-humble-joint-state-publisher
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install python3-colcon-common-extensions
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    sudo apt install ros-humble-twist-mux
    ```

## Installation

1.  **Clone the repository:**
    ```sh
    git clone https://github.com/RAMEL-ESPOL/MARVIN.git
    ```

2.  **Setup Meshes for Gazebo:**
    Navigate to the description package and copy the meshes to the Gazebo models folder:
    ```sh
    # Create destination directory if it doesn't exist
    mkdir -p ~/.gazebo/models/marvin_model

    #Navigate in the directory
    cd MARVIN/marvin_sim/src/marvin_description
    
    # Copy contents of "meshes" folder to ~/.gazebo/models/marvin_model
    cp -r meshes/* ~/.gazebo/models/marvin_model/
    ```

3.  **Update URDF Mesh Paths:**
    Before building, verify the mesh paths in the URDF file. Open `src/marvin_description/urdf/marvin.urdf.xacro` and check the comments at the beginning of the file. You may need to adjust the `<mesh filename="...">` paths so Gazebo can correctly find the STL files (e.g., using absolute paths or ensuring `package://` is resolved correctly).

4.  **Build the Workspace:**
    Go to `marvin_sim` folder:
    ```sh
    cd ../.. # Back to marvin_sim
    colcon build
    ```

## Execution Process

**Note:** Run `source install/setup.bash` in every new terminal window.

### 1. Launch Simulation
```sh
ros2 launch marvin_sim_gazebo gazebo_spawn.launch.py world:=./src/marvin_sim_gazebo/worlds/museum.world use_sim_time:=true
```

### 2. SLAM (Mapping)
Open a new terminal:
```sh
source install/setup.bash
ros2 launch marvin_navigation slam.launch.py use_sim_time:=true
```
*   Use a joystick or teleop to move the robot.
*   In Rviz, use the **SlamToolboxPlugin** to save the map as "marvin_world".
*   Stop SLAM (Ctrl+C) and remove the Map plugin from Rviz when done.

### 3. Navigation
Open a new terminal:
```sh
source install/setup.bash
ros2 launch marvin_navigation navigation.launch.py use_sim_time:=true
```
*   Use the **2D Goal Pose** tool in Rviz to send the robot to a target location autonomously.
