cmake_minimum_required(VERSION 3.0.2)
project(cable_layer)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  dynamic_reconfigure
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

generate_dynamic_reconfigure_options(cfg/CableLayer.cfg)

# Add messages
add_message_files(
  FILES
  CablePoints.msg
)

# Add services
add_service_files(
  FILES
  UpdateCablePoints.srv
)

# Add actions
add_action_files(
  	DIRECTORY action
  	FILES ExchangeCablePoints.action
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cable_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib dynamic_reconfigure message_runtime std_msgs geometry_msgs
)

# Specify include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exchange_cable_points_client src/exchange_cable_points_client.cpp)

# Declare the library
add_library(${PROJECT_NAME}
  src/cable_layer.cpp
)	

# Link the library against catkin libraries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(exchange_cable_points_client
  ${catkin_LIBRARIES}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install the plugin description XML file
install(FILES config/cable_layer_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Mark include directories for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Add dependencies for the executable
add_dependencies(exchange_cable_points_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

