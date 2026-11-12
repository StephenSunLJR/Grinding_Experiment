extern "C" {
    #include <mujoco/mujoco.h>
    #include <GLFW/glfw3.h> // 引入图形库
}
#include "franka_mujoco_hw.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "franka_mujoco_node");
    ros::NodeHandle nh;
    
    FrankaMujocoHW robot;
    //实例化硬件接口FrankaMujocoHW
    if (!robot.init(nh, nh)) {
        ROS_ERROR("Failed to initialize hardware interface.");
        return 1;
    }

    // 拿到 MuJoCo 的核心数据指针
    mjModel* m = robot.getModel();//将mj_model_中存放着的地址号复制一份，给变量m
    mjData* d = robot.getData();  //

    // ==========================================
    // 1. 初始化 GLFW 图形窗口
    // ==========================================
    if (!glfwInit()) {
        ROS_ERROR("Could not initialize GLFW");
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simulation - Franka Emika Panda", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // ==========================================
    // 2. 初始化 MuJoCo 的渲染数据结构
    // ==========================================
    mjvCamera cam;                      // 相机
    mjvOption opt;                      // 渲染选项 (可以开启受力箭头等)
    mjvScene scn;                       // 场景
    mjrContext con;                     // GPU 渲染上下文

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 调一个好看的默认上帝视角
    cam.azimuth = 90;
    cam.elevation = -25;
    cam.distance = 2.5;
    cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 0.5;

    // ==========================================
    // 3. 开启 ROS 与图形渲染的混合主循环
    // ==========================================
    controller_manager::ControllerManager cm(&robot, nh);//将编译好的被选择的控制器(如icvs_energy_mujoco)(.so动态库)加载进内存。
    
    // 因为 GLFW 的 while 循环会霸占主线程，必须开启异步 Spinner 来处理 ROS 通信
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    ros::Time last_time = ros::Time::now();

    // 当 ROS 正常且窗口没被点击关闭时，一直循环
    while (ros::ok() && !glfwWindowShouldClose(window)) {
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;
        last_time = current_time;

        // A. 跑 ROS 控制逻辑
        robot.read(current_time, period);
        cm.update(current_time, period);//调用控制器中的update逻辑
        robot.write(current_time, period);

        // B. 跑物理引擎逻辑 (时间推演)
        mj_step(m, d);//mujoco物理引擎

        // C. 跑图形渲染逻辑
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        // 更新场景并投射到屏幕
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ==========================================
    // 4. 关闭清理
    // ==========================================
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();

    return 0;
}