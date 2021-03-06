cmake_minimum_required(VERSION 2.8.3)
project(cliff_planners)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)


find_package(catkin REQUIRED COMPONENTS
  cliffmap_ros
  geometry_msgs
  roscpp
  costmap_2d
  smp_ros
  std_msgs
  tf
  visualization_msgs
  nav_core
)

find_package(MRPT REQUIRED)

find_package(Boost REQUIRED filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cliff_planners
  CATKIN_DEPENDS cliffmap_ros geometry_msgs roscpp smp_ros std_msgs tf nav_core visualization_msgs costmap_2d
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(cliff_planners src/down_the_cliff_planner.cpp)
add_dependencies(cliff_planners ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cliff_planners ${catkin_LIBRARIES} ${MRPT_LIBRARIES})

add_executable(standalone_planner src/standalone_planner.cpp)
add_dependencies(standalone_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(standalone_planner cliff_planners ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cliff_planners.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
