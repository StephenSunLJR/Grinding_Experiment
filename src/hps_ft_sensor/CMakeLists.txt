cmake_minimum_required(VERSION 3.4)
project(hps_ft_sensor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hps_sensor_driver
  CATKIN_DEPENDS geometry_msgs roscpp
)

set(HPS_VENDOR_LIBRARY
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libhps_ftc.so.1.0.0)
if(NOT EXISTS ${HPS_VENDOR_LIBRARY})
  message(FATAL_ERROR "Missing HPS vendor library: ${HPS_VENDOR_LIBRARY}")
endif()

add_library(hps_ftc_vendor SHARED IMPORTED GLOBAL)
set_target_properties(hps_ftc_vendor PROPERTIES
  IMPORTED_LOCATION ${HPS_VENDOR_LIBRARY}
  IMPORTED_SONAME libhps_ftc.so.1
)

add_library(hps_sensor_driver src/hps_sensor.cpp)
target_include_directories(hps_sensor_driver PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(hps_sensor_driver
  hps_ftc_vendor
  ${catkin_LIBRARIES}
)

add_executable(hps_wrench_publisher src/hps_wrench_publisher.cpp)
target_include_directories(hps_wrench_publisher SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(hps_wrench_publisher PRIVATE include)
target_link_libraries(hps_wrench_publisher
  hps_sensor_driver
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

install(TARGETS hps_sensor_driver hps_wrench_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# Keep the versioned file and SONAME symlinks together. roslaunch adds the
# workspace lib directory to LD_LIBRARY_PATH in devel/install spaces.
install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
