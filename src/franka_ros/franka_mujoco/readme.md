构建franka的mujoco仿真器，并在仿真器中进行仿真控制的实现流程：
1.导入franka的.xml文件
2.构建franka_mujoco_hw
    franka_mujoco_hw.h:在内存中开辟一组变量，一头连接ROS，另一头连接Mujoco
    franka_mujoco_hw.cpp:最关键的数据搬运逻辑，实现数据从控制器到mujoco中的传递