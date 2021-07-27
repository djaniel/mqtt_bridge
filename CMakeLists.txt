cmake_minimum_required(VERSION 3.0.2)
project(mqtt_bridge)
catkin_python_setup()
find_package(catkin REQUIRED
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_generation 
   tf2
   tf2_ros)
## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    deca_anchor.msg
    deca_distances.msg
    deca_data.msg
    deca_pos.msg
    decawave.msg
 )
 
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package( CATKIN_DEPENDS message_runtime)


install(PROGRAMS
  scripts/mqtt_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_target(install_depends ALL COMMAND "pip3" "install" "--user" "-r" "${PROJECT_SOURCE_DIR}/requirements.txt")
