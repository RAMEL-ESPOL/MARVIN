cmake_minimum_required(VERSION 2.8.3)
project(vehicle_gazebo)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    tf
)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/ackermann_controller scripts/cmd_vel_to_ackermann_drive
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_executable(odom_broadcaster src/odom_broadcaster.cpp)
target_link_libraries(odom_broadcaster ${catkin_LIBRARIES})
