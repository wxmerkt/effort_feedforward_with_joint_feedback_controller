cmake_minimum_required(VERSION 3.0.2)
project(effort_feedforward_with_joint_feedback_controller)
set(CMAKE_CXX_STANDARD 14)

set(CATKIN_DEPENDENCIES
  angles  
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
  ipab_controller_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wpedantic -Wextra)

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
