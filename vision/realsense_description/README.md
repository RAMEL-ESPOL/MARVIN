# Realsense2_Description

<details open>
<summary> <b>Brief Review<b></summary>


IMPORTANT NOTE:  You need to add to the src folder the realsense_gazebo_plugin package that you could download from PAL Robotics or my repository

This is the original and modified folder for the [Intel Realsense ROS repo](https://github.com/IntelRealSense/realsense-ros).  We only took the realsense2_description folder.  The project here will let you know how to import and use realsense cameras in gazebo.

Below a few image examples of the outcome.

<p align="center">
<img src = "docs/imgs/realsense_rviz.PNG?raw=true" width="40%"/>
<img src = "docs/imgs/realsense_gazebo.PNG?raw=true" width="48%"/>
</p>

The project tree:

<p align="center">
<img src = "docs/imgs/tree.PNG?raw=true" width="45%"/>
</p>

This applications function as follows.
- First we will excecute the launch the file to view the d435 model
- Next the rviz and gazebo applications will launch
- When rviz and gazebo are launched you can view the D435 camera  in gazebo and rviz
- Then when everything is loaded you could add objects in gazebo and see these objects and differents topics in rviz
- Play then with the paramenters as you wish

</details>

<details open>
<summary> <b>Using realsense2_description package<b></summary>

- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
    source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/realsense2_description
    git clone https://github.com/issaiass/realsense_gazebo_plugin
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
~~~
- You must see that `roscore` and all configurations loading succesfully.
- When everything ends, you must see gazebo and rviz loaded and the intel realsense d435 camera with an empty world.
- Next put an object in front of the camera and play in rviz with the paramenters
- Remember that the frames are inside the camera so you will see in rviz initially some artifacts (camera chassis internals), you must zoom if you do not want to see it

<p align="center">
<img src = "docs/imgs/gazebo.PNG?raw=true" width="80%"/>
</p>


<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

[<img src= "https://img.youtube.com/vi/mJILStNnYyU/0.jpg" />](https://youtu.be/mJILStNnYyU)
</p>

The video only shows the application running, not the explanation of the code.

</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this youtube video.

<p align="center">

[<img src= "https://img.youtube.com/vi/hpUCG6K5muI/0.jpg" />](https://youtu.be/hpUCG6K5muI)

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

- No issues

</details>

<details open>
<summary> <b>Future Work<b></summary>

- This package is maintained by the intel company.  Always prefer to see the intel realsense ros github repo for new implementations and models.

</details>

<details open>
<summary> <b>Contributiong<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.
