##### 小猛  ######
cmake_minimum_required(VERSION 3.4) #cmake要求的最小版本
project(unified_controllers)  #项目名称

set(CMAKE_BUILD_TYPE Release)  #设置生成的类型
set(CMAKE_CXX_STANDARD 14)   #C++标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) #是否要求C++标准

#添加外部依赖包，头文件和库
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED) #这个根据自己的要求改
#添加自己定义的msg文件
add_message_files(FILES
  unified_robot.msg
  print_pose_robot.msg
)
#消息生成函数
generate_messages(
  DEPENDENCIES
  std_msgs
)
generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
  cfg/dual_arm_compliance_param.cfg
  cfg/teleop_param.cfg
  cfg/teleop_gripper_param.cfg
)
#用于为catkin提供构建、生成pkg_config和CMAKE文件所需要的信息。应该是为了后面使用该包的人准备的
#INCLUDE_DIRS-声明给其它package的include路径
#LIBRARIES-声明给其他package的库
#CATKIN_DEPENDS-本包依赖的catkin package
#DEPENDS-本包依赖的非catkin package
#CFG_EXTRAS-其它配置参数
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unified_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

#将指定的源文件生成库文件
add_library(unified_controllers
  src/unified.cpp
  src/print_pose.cpp
  src/CoriolisMatrix.cpp
  src/FrictionTorque.cpp
  src/GravityVector.cpp
  src/MassMatrix.cpp
  src/move_to_position.cpp
  src/unified_power.cpp
  src/unified_compared.cpp
)
#add_dependencies(<target> [<target-dependency>]) 生成target所需要的依赖，被依赖的项目先构建
add_dependencies(unified_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)
add_executable(trajectory_recorder_node src/trajectory_recorder_node.cpp)
#设置要连接的库文件的名称
target_link_libraries(unified_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(trajectory_recorder_node PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
#指定目标包含的头文件的路径
target_include_directories(unified_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(unified_controllers PUBLIC
  include
)


## Installation 编译后文件的安装路径
install(TARGETS unified_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES pid_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(unified_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(unified_controllers
    FILES ${SOURCES}
    DEPENDS unified_controllers
  )
endif()

