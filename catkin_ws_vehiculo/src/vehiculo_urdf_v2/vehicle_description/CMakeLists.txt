cmake_minimum_required(VERSION 3.0.2)

project(vehicle_description)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
)
catkin_package()

include_directories( ${catkin_INCLUDE_DIRS})


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

