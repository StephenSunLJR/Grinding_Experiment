##### 小猛  ######
cmake_minimum_required(VERSION 3.4) #cmake要求的最小版本
project(variable_stiffness_controllers)  #项目名称

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
  std_srvs
  std_msgs
  kdl_parser
  orocos_kdl
)

# Pinocchio is optional (only needed by icvs_energy_mujoco, currently disabled)
find_package(pinocchio QUIET)
find_package(ruckig 0.19.6 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED) #这个根据自己的要求改
#添加自己定义的msg文件
add_message_files(FILES
  compliance_robot.msg
  BidirectionalDiagnostics.msg
)
#消息生成函数
generate_messages(
  DEPENDENCIES
  std_msgs
)

#用于为catkin提供构建、生成pkg_config和CMAKE文件所需要的信息。应该是为了后面使用该包的人准备的
#INCLUDE_DIRS-声明给其它package的include路径
#LIBRARIES-声明给其他package的库
#CATKIN_DEPENDS-本包依赖的catkin package
#DEPENDS-本包依赖的非catkin package
#CFG_EXTRAS-其它配置参数
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES variable_stiffness_controllers
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
    std_srvs
    std_msgs
  DEPENDS Franka
)

#将指定的源文件生成库文件
add_library(variable_stiffness_controllers
  src/Dynamics.cpp 
  src/icvs.cpp
  src/CoriolisMatrix.cpp
  src/FrictionTorque.cpp
  src/GravityVector.cpp
  src/MassMatrix.cpp
  src/icvs_emg.cpp
  src/standardimpedance.cpp
  src/Calculate_torque.cpp
  src/icvs_smooth.cpp
  src/icvs_energy.cpp
  src/bidirectional_energy_flow_control.cpp
  src/bidirectional_energy_flow_control_main.cpp
  src/pure_joint_position_hold_controller.cpp
  src/interactive_control.cpp
  src/matrix_utils_for_interactive.cpp
  src/A_DLS_adapt.cpp
)
  #src/interactive_control.cpp

#add_dependencies(<target> [<target-dependency>]) 生成target所需要的依赖，被依赖的项目先构建
add_dependencies(variable_stiffness_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)

#设置要连接的库文件的名称
# target_link_libraries(icvs_energy_mujoco PUBLIC
#   pinocchio::pinocchio
# )
target_link_libraries(variable_stiffness_controllers PUBLIC
  ruckig::ruckig
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
)

target_compile_features(variable_stiffness_controllers PUBLIC cxx_std_20)
#指定目标包含的头文件的路径
target_include_directories(variable_stiffness_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(variable_stiffness_controllers PUBLIC
  include
)

# Model_example.cpp contains its own main() and must not be linked into the
# controller plugin library (doing so hijacks executables that link the plugin).
add_executable(franka_model_example src/Model_example.cpp)
target_link_libraries(franka_model_example
  variable_stiffness_controllers
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(bidirectional_energy_flow_control_test
    test/bidirectional_energy_flow_control_test.cpp
  )
  if(TARGET bidirectional_energy_flow_control_test)
    target_link_libraries(bidirectional_energy_flow_control_test
      variable_stiffness_controllers
      ${catkin_LIBRARIES}
    )
  endif()
endif()



## Installation 编译后文件的安装路径
install(TARGETS variable_stiffness_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES variable_stiffness_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/franka_to_geometry_messages.py
  scripts/record_diagnostics.py
  scripts/record_bidirectional_diagnostics.py
  scripts/analyze_exp1.py
  scripts/analyze_exp2a.py
  scripts/analyze_exp3b.py
  scripts/plot_utils.py
  scripts/live_dashboard.py
  scripts/force_injector.py
  scripts/analyze_exp2b.py
  scripts/safe_ctrl_c_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
  add_format_target(variable_stiffness_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(variable_stiffness_controllers
    FILES ${SOURCES}
    DEPENDS variable_stiffness_controllers
  )
endif()

