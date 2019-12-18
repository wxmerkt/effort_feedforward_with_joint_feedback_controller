cmake_minimum_required(VERSION 2.8.12)
project(effort_feedforward_with_joint_feedback_controller)

set(CATKIN_DEPENDENCIES
  angles  
  controller_interface
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

# Generate Messages
add_message_files(
  FILES
  EffortFeedforwardWithJointFeedback.msg
)
generate_messages(DEPENDENCIES std_msgs)

# Export catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS pluginlib
)
  
###########
## Build ##
###########
add_library(${PROJECT_NAME} src/EffortFeedforwardWithJointFeedbackController.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++14 -Wall)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES EffortFeedforwardWithJointFeedbackController.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
