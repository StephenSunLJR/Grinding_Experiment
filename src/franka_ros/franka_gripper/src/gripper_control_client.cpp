#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// 包含需要的 Action 消息头文件
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h> // Homing 也是常用的

// 为 Action 客户端类型创建别名，方便使用
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction> MoveClient;
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient;
typedef actionlib::SimpleActionClient<franka_gripper::HomingAction> HomingClient;

// --- 函数：移动夹爪到指定宽度 ---
bool moveGripper(double width, double speed = 0.1) {
    // 1. 创建 MoveAction 客户端
    MoveClient move_client("/panda/franka_gripper/move", true); // true -> 自动在单独线程中 spin

    // 2. 等待 Action 服务器启动 (可以设置超时)
    ROS_INFO("Waiting for MoveAction server to start...");
    if (!move_client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("MoveAction server did not come up within timeout!");
        return false;
    }
    ROS_INFO("MoveAction server started.");

    // 3. 创建并填充 Goal
    franka_gripper::MoveGoal goal;
    goal.width = width;
    goal.speed = speed;

    // 4. 发送 Goal
    ROS_INFO("Sending move goal (width: %.3f m, speed: %.3f m/s)...", goal.width, goal.speed);
    move_client.sendGoal(goal);

    // 5. 等待结果 (可以设置超时)
    bool finished_before_timeout = move_client.waitForResult(ros::Duration(15.0));

    // 6. 处理结果
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = move_client.getState();
        ROS_INFO("MoveAction finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
             franka_gripper::MoveResultConstPtr result = move_client.getResult();
             if (result && result->success) {
                ROS_INFO("Move successful!");
                return true;
             } else {
                ROS_WARN("Move reported success=false. Error: %s", result ? result->error.c_str() : "N/A");
                return false;
             }
        } else {
             ROS_WARN("Move did not succeed.");
             return false;
        }
    } else {
        ROS_WARN("MoveAction did not finish before the time out.");
        return false;
    }
}

// --- 函数：执行抓取 ---
bool graspGripper(double width, double force, double speed = 0.05, double inner_epsilon = 0.005, double outer_epsilon = 0.005) {
    // 1. 创建 GraspAction 客户端
    GraspClient grasp_client("/panda/franka_gripper/grasp", true);

    // 2. 等待 Action 服务器启动
    ROS_INFO("Waiting for GraspAction server to start...");
    if (!grasp_client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("GraspAction server did not come up within timeout!");
        return false;
    }
    ROS_INFO("GraspAction server started.");

    // 3. 创建并填充 Goal
    franka_gripper::GraspGoal goal;
    goal.width = width;
    goal.force = force;
    goal.speed = speed;
    goal.epsilon.inner = inner_epsilon;
    goal.epsilon.outer = outer_epsilon;

    // 4. 发送 Goal
    ROS_INFO("Sending grasp goal (width: %.3f m, force: %.1f N, speed: %.3f m/s)...",
             goal.width, goal.force, goal.speed);
    grasp_client.sendGoal(goal);

    // 5. 等待结果
    bool finished_before_timeout = grasp_client.waitForResult(ros::Duration(15.0));

    // 6. 处理结果
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = grasp_client.getState();
        ROS_INFO("GraspAction finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            franka_gripper::GraspResultConstPtr result = grasp_client.getResult();
            if (result && result->success) {
                ROS_INFO("Grasp successful!");
                return true;
            } else {
                 ROS_WARN("Grasp reported success=false. Error: %s", result ? result->error.c_str() : "N/A");
                 return false;
            }
        } else {
            ROS_WARN("Grasp did not succeed.");
            return false;
        }
    } else {
        ROS_WARN("GraspAction did not finish before the time out.");
        return false;
    }
}

// --- 函数：执行回零 ---
bool homeGripper() {
    HomingClient homing_client("/panda/franka_gripper/homing", true);
    ROS_INFO("Waiting for HomingAction server to start...");
     if (!homing_client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("HomingAction server did not come up within timeout!");
        return false;
    }
    ROS_INFO("HomingAction server started.");

    franka_gripper::HomingGoal goal; // Homing Goal 通常是空的

    ROS_INFO("Sending homing goal...");
    homing_client.sendGoal(goal);

    bool finished_before_timeout = homing_client.waitForResult(ros::Duration(20.0)); // Homing 可能需要更长时间

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = homing_client.getState();
        ROS_INFO("HomingAction finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
             franka_gripper::HomingResultConstPtr result = homing_client.getResult();
             if (result && result->success) {
                ROS_INFO("Homing successful!");
                return true;
             } else {
                ROS_WARN("Homing reported success=false. Error: %s", result ? result->error.c_str() : "N/A");
                return false;
             }
        } else {
            ROS_WARN("Homing did not succeed.");
            return false;
        }
    } else {
        ROS_WARN("HomingAction did not finish before the time out.");
        return false;
    }
}


// --- 主函数：示例调用 ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_control_client_node");
    ros::NodeHandle nh;

    ROS_INFO("Gripper control client node started.");

    // --- 示例序列 ---
    try {
        // 1. 执行回零 (通常在启动后需要做一次)
        ROS_INFO("=== Performing Homing ===");
        if (!homeGripper()) {
             ROS_ERROR("Homing failed. Exiting.");
             return 1;
        }
        ros::Duration(1.0).sleep(); // 等待1秒

        // 2. 张开夹爪到最大宽度 (假设最大为 0.08m)
        ROS_INFO("=== Moving to max width ===");
        moveGripper(0.08, 0.1);
        ros::Duration(2.0).sleep(); // 等待2秒

        // 3. 尝试抓取一个宽度为 0.02m 的物体，施加 15N 的力
        ROS_INFO("=== Attempting Grasp ===");
        graspGripper(0.02, 15.0, 0.05); // width, force, speed
        ros::Duration(3.0).sleep(); // 等待3秒

        // 4. 再次张开夹爪
        ROS_INFO("=== Moving to open width again ===");
        moveGripper(0.06, 0.1);
        ros::Duration(2.0).sleep();

    } catch (const std::exception& e) {
        ROS_ERROR("An exception occurred: %s", e.what());
        return 1;
    }

    ROS_INFO("Gripper control sequence finished.");
    return 0;
}