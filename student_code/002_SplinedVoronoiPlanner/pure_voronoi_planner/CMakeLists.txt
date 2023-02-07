cmake_minimum_required(VERSION 2.8.3)
project(pure_voronoi_planner)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  rospy
  std_msgs
  sensor_msgs

  costmap_2d
  splined_voronoi

  # nlopt
  message_generation
  dynamic_reconfigure
)

find_package(OpenCV)

find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  config/PureVoronoiPlanner.cfg
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  INCLUDE_DIRS include
  # LIBRARIES com
  # CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure splined_voronoi

  DEPENDS OpenMP
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(

  # include
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(boost_voronoi src/boost_voronoi.cpp)
target_link_libraries(boost_voronoi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(boost_voronoi ${catkin_EXPORTED_TARGETS})

add_library(dynamicvoronoi src/dynamicvoronoi/dynamicvoronoi.cpp src/dynamicvoronoi/bucketedqueue.cpp)
target_link_libraries(dynamicvoronoi ${catkin_LIBRARIES})
add_dependencies(dynamicvoronoi ${catkin_EXPORTED_TARGETS})

add_library(dynamicvoronoi_interface src/dynamicvoronoi_interface.cpp)
target_link_libraries(dynamicvoronoi_interface
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  dynamicvoronoi
)

add_library(planning_utils src/planning_utils.cpp)
target_link_libraries(planning_utils ${catkin_LIBRARIES})
add_dependencies(planning_utils ${catkin_EXPORTED_TARGETS})

add_library(pure_voronoi_planner src/pure_voronoi_planner.cpp)
target_link_libraries(pure_voronoi_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  boost_voronoi
  planning_utils
)
add_dependencies(pure_voronoi_planner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


add_executable(compare_voronoi_results src/compare_voronoi_results.cpp)
target_link_libraries(compare_voronoi_results ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} boost_voronoi dynamicvoronoi_interface)
add_dependencies(compare_voronoi_results ${catkin_EXPORTED_TARGETS})

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
