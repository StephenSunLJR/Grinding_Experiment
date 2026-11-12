#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>//用于文件写入
#include <string>
#include <vector>
#include <iomanip> //用于设置输出精度

class TrajectoryRecorder
{ 
public:
  TrajectoryRecorder(ros::NodeHandle& nh):
    nh_(nh),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    is_first_transform_(true)
    {
        //获取参数
        nh_.param<std::string>("base_frame",base_frame_,"panda_link0");
        nh_.param<std::string>("end_effector_frame",end_effector_frame_,"panda_link8");
        nh_.param<double>("listen_rate",listen_rate_,100.0);
        nh_.param<std::string>("path_topic",path_topic_,"/end_effector_path");
        nh_.param<int>("max_poses",max_poses_,1000000);

        ROS_INFO_STREAM("RealtimeTrajectoryVisualizer initialized with params: ");
        ROS_INFO_STREAM("-Listening Rate:"<< listen_rate_<<" Hz");
        ROS_INFO_STREAM("-Base Frame:"<< base_frame_);
        ROS_INFO_STREAM("-End Effector Frame:"<< end_effector_frame_);  
        ROS_INFO_STREAM("-Path Topic:"<< path_topic_);
        ROS_INFO_STREAM("-Max Poses:"<< max_poses_);
        //初始化Path消息发布器
        path_pub_=nh_.advertise<nav_msgs::Path>(path_topic_,10);

        path_msg_.header.frame_id=base_frame_;
        ROS_INFO("Waiting for the first TF transform...");
    }

    //主监听与发布循环
    void run()
    {
        ros::Rate rate(listen_rate_);

        while(ros::ok())
        {
            try
            {
                //查询最新的变换
                geometry_msgs::TransformStamped transformStamped=tf_buffer_.lookupTransform(base_frame_,end_effector_frame_,ros::Time(0));
                
                ROS_INFO_STREAM_THROTTLE(1.0, "[DEBUG INSIDE NODE] Transform Time: " << transformStamped.header.stamp <<
                                 ", Pos X: " << transformStamped.transform.translation.x);
                
                //如果是第一次成功查询到变换


                if(is_first_transform_){
                    ROS_INFO("TF transforms are available. Start publishing path...");
                    is_first_transform_=false;
                }
                // --- 创建一个新的 PoseStamped 消息 ---
                geometry_msgs::PoseStamped current_pose;
                current_pose.header.stamp = transformStamped.header.stamp; // 使用 TF 的时间戳
                current_pose.header.frame_id = base_frame_; // 姿态也是相对于基坐标系
                current_pose.pose.position.x = transformStamped.transform.translation.x;
                current_pose.pose.position.y = transformStamped.transform.translation.y;
                current_pose.pose.position.z = transformStamped.transform.translation.z;
                // Path 消息需要包含姿态信息，我们直接使用 TF 中的姿态
                current_pose.pose.orientation = transformStamped.transform.rotation;
                //Path 消息需要包含姿态信息，我们直接使用TF中的姿态
                path_msg_.poses.push_back(current_pose);

                // --- 限制 Path 消息中姿态的数量 (可选) ---
                if (path_msg_.poses.size() > max_poses_) {
                    path_msg_.poses.erase(path_msg_.poses.begin()); // 删除最旧的点
                }

                // --- 更新 Path 消息的时间戳并发布 ---
                path_msg_.header.stamp = current_pose.header.stamp; // 使用最新的时间戳
                path_pub_.publish(path_msg_);
            }
            catch(const tf2::TransformException& ex){
                //如果变换暂时不可用，发出警告
                ROS_WARN_STREAM_THROTTLE(2.0,"TF lookup failed between'"<<base_frame_<<"'and '"<<end_effector_frame_<<"'");
                is_first_transform_=true;//重置标志位
                ros::Duration(0.1).sleep();
            }
            rate.sleep();
            ros::spinOnce();
        }
    }
private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher path_pub_;//消息发布器
  nav_msgs::Path path_msg_;//消息对象

  std::string base_frame_;
  std::string end_effector_frame_;
  double listen_rate_;
  std::string path_topic_;
  int max_poses_;//用于限制路径长度

  bool is_first_transform_;

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"trajectory_recorder_node");
  ros::NodeHandle nh("~");

  try{
    TrajectoryRecorder visualizer(nh);
    visualizer.run();
  }
  catch(const std::exception& e){
    ROS_ERROR_STREAM("An exception occurred: "<<e.what());
    return 1;
  }catch(...){
    ROS_ERROR_STREAM("An unknown exception occurred.");
    return 1;
  }
  return 0;
}