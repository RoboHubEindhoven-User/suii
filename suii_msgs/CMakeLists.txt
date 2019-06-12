cmake_minimum_required(VERSION 2.8.3)
project(suii_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
    FILES
    VisionScanObject.msg
    SuiiTask.msg
    SuiiTaskList.msg
)

add_service_files(
    FILES
    DistanceToGoal.srv
    ManipulationAction.srv
    ManipulationPose.srv
    NavigationGoal.srv
    NavigationRepose.srv
    VisionScan.srv
    ItemPick.srv
    ItemPlace.srv 
    ItemFindhole.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES suii_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
