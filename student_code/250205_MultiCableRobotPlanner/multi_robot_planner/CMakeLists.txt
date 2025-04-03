cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
  message_generation
  sensor_msgs
  actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pair.msg
  RobotData.msg
  RobotDataList.msg
  StartGoalPair.msg
  StartGoalPairList.msg
  ScoringMetric.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_robot_planner
  CATKIN_DEPENDS message_runtime costmap_2d geometry_msgs nav_msgs roscpp rospy std_msgs tf2_ros sensor_msgs actionlib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

## Declare executables
add_executable(global_planner_node src/global_planner_node.cpp src/main.cpp)
add_executable(start_goal_publisher_node src/start_goal_publisher_node.cpp)
add_executable(start_goal_publisher_node_1_robot src/start_goal_publisher_node_1_robot.cpp)
add_executable(start_goal_publisher_node_2_robots src/start_goal_publisher_node_2_robots.cpp)
add_executable(start_goal_publisher_node_3_robots src/start_goal_publisher_node_3_robots.cpp)
add_executable(start_goal_publisher_node_4_robots src/start_goal_publisher_node_4_robots.cpp)
add_executable(start_goal_publisher_node_6_robots src/start_goal_publisher_node_6_robots.cpp)
add_executable(start_goal_publisher_node_6_robots_long_house src/start_goal_publisher_node_6_robots_long_house.cpp)
add_executable(start_goal_publisher_node_6_robots_long_house_custom_order src/start_goal_publisher_node_6_robots_long_house_custom_order.cpp)

## Add dependencies for the executables
add_dependencies(global_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(start_goal_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link against catkin libraries
target_link_libraries(global_planner_node ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_1_robot ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_2_robots ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_3_robots ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_4_robots ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_6_robots ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_6_robots_long_house ${catkin_LIBRARIES})
target_link_libraries(start_goal_publisher_node_6_robots_long_house_custom_order ${catkin_LIBRARIES})
#############
## Install ##
#############

## Mark executables for installation
install(TARGETS global_planner_node start_goal_publisher_node start_goal_publisher_node_1_robot
        start_goal_publisher_node_2_robots start_goal_publisher_node_3_robots
        start_goal_publisher_node_4_robots
        start_goal_publisher_node_6_robots start_goal_publisher_node_6_robots_long_house
        start_goal_publisher_node_6_robots start_goal_publisher_node_6_robots_long_house_custom_order
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation (if applicable)
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation (if applicable)
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multi_robot_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

