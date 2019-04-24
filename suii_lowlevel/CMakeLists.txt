cmake_minimum_required(VERSION 2.8.3)
project(suii_lowlevel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  suii_status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(suii_lowlevel 
	src/suii_lowlevel.cpp
	src/lowlevelcontroller.cpp
)

#add_dependencies(slambot_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(suii_lowlevel suii_lowlevel_generate_messages_cpp)

target_link_libraries(suii_lowlevel
   ${catkin_LIBRARIES}
 )

install(TARGETS suii_lowlevel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
