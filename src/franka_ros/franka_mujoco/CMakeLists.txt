cmake_minimum_required(VERSION 3.0.2)
project(franka_mujoco)

# 1. 查找依赖的 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  franka_hw
)

catkin_package(
  CATKIN_DEPENDS roscpp hardware_interface controller_manager franka_hw
)

set(MUJOCO_ROOT_DIR "/home/sun/.mujoco/mujoco-3.1.5")

# 寻找系统的 OpenGL 和 GLFW 库
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# 记得把 GLFW 的头文件目录加进去
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MUJOCO_ROOT_DIR}/include
  ${GLFW_INCLUDE_DIRS}
)
# 3. 指定头文件所在的文件夹
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MUJOCO_ROOT_DIR}/include
)

add_library(mujoco_lib SHARED IMPORTED)
set_target_properties(mujoco_lib PROPERTIES
   IMPORTED_LOCATION "${MUJOCO_ROOT_DIR}/lib/libmujoco.so"
)
# 编译你的自定义控制器插件
add_library(mujoco_impedance_controller mujoco_impedance_controller.cpp)
target_link_libraries(mujoco_impedance_controller ${catkin_LIBRARIES})

# 4. 把那两个 cpp 文件编译成 ROS 节点
add_executable(franka_mujoco_node
  franka_mujoco_hw.cpp
  franka_mujoco_node.cpp
)

# 5. 将节点与 ROS系统、MuJoCo 引擎连接起来
target_link_libraries(franka_mujoco_node
  ${catkin_LIBRARIES}
  mujoco_lib
  ${OPENGL_LIBRARIES}     # 新增：OpenGL 渲染核心
  ${GLFW_LIBRARIES}       # 新增：GLFW 窗口管理
)