cmake_minimum_required(VERSION 3.0.2)
project(planar_3dof_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  joint_state_controller
  robot_state_publisher
  rqt_gui
  message_generation
  geometry_msgs
)

catkin_python_setup()


add_message_files(
      FILES
      EndEffector.msg 
    )

generate_messages(
      DEPENDENCIES
      std_msgs
      geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime 

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS ${python_scripts_DIR}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

