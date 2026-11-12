// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/fuzzy_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
using namespace std;

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3
namespace franka_example_controllers {

bool FuzzyExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                               ros::NodeHandle& node_handle) {
  // std::string arm_id;
  // if (!node_handle.getParam("arm_id", arm_id)) {
  //   ROS_ERROR_STREAM("FuzzyExampleController: Could not read parameter arm_id");
  //   return false;
  // }
  // std::vector<std::string> joint_names;
  // if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
  //   ROS_ERROR(
  //       "FuzzyExampleController: Invalid or no joint_names parameters provided, "
  //       "aborting controller init!");
  //   return false;
  // }

  // auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  // if (model_interface == nullptr) {
  //   ROS_ERROR_STREAM(
  //       "FuzzyExampleController: Error getting model interface from hardware");
  //   return false;
  // }
  // try {
  //   model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
  //       model_interface->getHandle(arm_id + "_model"));
  // } catch (hardware_interface::HardwareInterfaceException& ex) {
  //   ROS_ERROR_STREAM(
  //       "FuzzyExampleController: Exception getting model handle from interface: "
  //       << ex.what());
  //   return false;
  // }

  // auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  // if (state_interface == nullptr) {
  //   ROS_ERROR_STREAM(
  //       "FuzzyExampleController: Error getting state interface from hardware");
  //   return false;
  // }
  // try {
  //   state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
  //       state_interface->getHandle(arm_id + "_robot"));
  // } catch (hardware_interface::HardwareInterfaceException& ex) {
  //   ROS_ERROR_STREAM(
  //       "FuzzyExampleController: Exception getting state handle from interface: "
  //       << ex.what());
  //   return false;
  // }

  // auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  // if (effort_joint_interface == nullptr) {
  //   ROS_ERROR_STREAM(
  //       "FuzzyExampleController: Error getting effort joint interface from hardware");
  //   return false;
  // }
  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
  //   } catch (const hardware_interface::HardwareInterfaceException& ex) {
  //     ROS_ERROR_STREAM(
  //         "FuzzyExampleController: Exception getting joint handles: " << ex.what());
  //     return false;
  //   }
  // }
position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;

                                               }

void FuzzyExampleController::starting(const ros::Time& /*time*/) {
  // franka::RobotState robot_state=state_handle_->getRobotState();
  
  for(int i=0;i<7;i++)
  {
    initial_pose_[i]=robot_state.q[i];
  }
  elapsed_time_=ros::Duration(0.0);

  Kp[0]=k_gains_[4];
  Kd[0]=d_gains_[4];
  Ki[0]=i_gains_[4];
  Kp[1]=k_gains_[6];
  Kd[1]=d_gains_[6];
  Ki[1]=i_gains_[6];
  
  //  FILE*Fuzzy_PID_date1=fopen("/home/liu/liu_test/fuzzy_pid_date1.text","w");
  //   fprintf(Fuzzy_PID_date1," tau5\t joint5_real\t joint5_desired\t Kp_0\t Kd_0\t error_0\t time \n");
  //   fclose(Fuzzy_PID_date1);

  //   FILE*Fuzzy_PID_date2=fopen("/home/liu/liu_test/fuzzy_pid_date2.text","w");
  //   fprintf(Fuzzy_PID_date2," tau7\t joint7_real\t joint7_desired\t Kp_1\t Kd_1\t error_1\t time \n");
  //   fclose(Fuzzy_PID_date2);
}

void FuzzyExampleController::update(const ros::Time& time,
                                                 const ros::Duration& period) {
      if(elapsed_time_<=ros::Duration(1e-4))
      {
              Init_FuzzyExampleController(0.6 ,0.6,20,1,0.1,0.01,0.01,10);
      }
      elapsed_time_+=period;                                   
    double delta_angle = std::sin(M_PI/5*elapsed_time_.toSec())*0.2;
    double omega = std::sin(M_PI/5*elapsed_time_.toSec())*0.2*M_PI/5;

    std::array<double, 7> pose_desired = initial_pose_;
    for (size_t i = 0; i < 7; ++i) {
        if (i==4||i==6) {
            pose_desired[i] = initial_pose_[i] + (i-3)*delta_angle;
            dpose_desired[i] = -omega;
        } else {
            pose_desired[i] = initial_pose_[i] ;
        }
    }
    
    franka::RobotState robot_state = state_handle_->getRobotState();
    double alpha = 0.99;
    for (size_t i = 0; i < 7; i++) {
        dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }
    target[0]=pose_desired[4];//获取第四个关节的期望值
    actual[0]=robot_state.q[4];//获取第四个关节的实际值

    target[1]=pose_desired[6];//获取第六个关节的期望值
    actual[1]=robot_state.q[6];//获取第六个关节的实际值
    //建立模糊规则
    int deltaKpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
	                          {PB,PB,PM,PS,PS,ZO,NS},
						                {PM,PM,PM,PS,ZO,NS,NS},
	                          {PM,PM,PS,ZO,NS,NM,NM},
	                          {PS,PS,ZO,NS,NS,NM,NM},
	                          {PS,ZO,NS,NM,NM,NM,NB},
	                          {ZO,ZO,NM,NM,NM,NB,NB}};
    int deltaKiMatrix[7][7]={ {NB,NB,NM,NM,NS,ZO,ZO},
	                            {NB,NB,NM,NS,NS,ZO,ZO},
						                  {NB,NM,NS,NS,ZO,PS,PS},
	                            {NM,NM,NS,ZO,PS,PM,PM},
	                            {NM,NS,ZO,PS,PS,PM,PB},
	                            {ZO,ZO,PS,PS,PM,PB,PB},
	                            {ZO,ZO,PS,PM,PM,PB,PB}};
	int deltaKdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},
	                         {PS,NS,NB,NM,NM,NS,ZO},
						               {ZO,NS,NM,NM,NS,NS,ZO},
	                         {ZO,NS,NS,NS,NS,NS,ZO},
	                         {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
	                         {PB,NS,PS,PS,PS,PS,PB},
	                         {PB,PM,PM,PM,PS,PS,PB}};
  float e_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float de_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kp_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Ki_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kd_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
  
  // float e_mg_paras[]={-3,2,-2,0.25,-1,0.25,0,0.25,1,0.25,2,0.25,3,0.25};
  // float de_mf_paras[]={-3,2,-2,0.25,-1,0.25,0,0.25,1,0.25,2,0.25,3,0.25};
  // float Kp_mf_paras[]={-3,2,-2,0.25,-1,0.25,0,0.25,1,0.25,2,0.25,3,0.25};
  // float Ki_mf_paras[]={-3,2,-2,0.25,-1,0.25,0,0.25,1,0.25,2,0.25,3,0.25};
  // float Kd_mf_paras[]={-3,2,-2,0.25,-1,0.25,0,0.25,1,0.25,2,0.25,3,0.25};

  setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",Kp_mf_paras,"trimf",Ki_mf_paras,"trimf",Kd_mf_paras);
  //setMf("gaussmf",e_mf_paras,"gaussmf",de_mf_paras,"gaussmf",Kp_mf_paras,"gaussmf",Ki_mf_paras,"gaussmf",Kd_mf_paras);
  setRuleMatrix(deltaKpMatrix,deltaKiMatrix,deltaKdMatrix);
  realize(target[0],actual[0],0);
  k_gains_[4]=Kp[0];
  d_gains_[4]=Kd[0];
  i_gains_[4]=Ki[0];

  realize(target[1],actual[1],1);
  k_gains_[6]=Kp[1];
  d_gains_[6]=Kd[1];
  i_gains_[6]=Ki[1];
  // cout<<"主函数"<<endl;
  // cout<<target[1]<<"  "<<actual[1]<<endl;
  // cout<<Kp<<"  "<<Kd<<endl;
  // cout<<k_gains_[5]<<"  "<<d_gains_[5]<<endl;
  std::array<double, 7> tau_d_calculated;

  for (size_t i = 0; i < 7; ++i) {
    
    tau_d_calculated[i] = k_gains_[i] * (pose_desired[i] - robot_state.q[i]) + d_gains_[i] * (dpose_desired[i]-dq_filtered_[i]);
  }

  // cout<<"输出力矩"<<endl;
  // cout<<k_gains_[4]<<"  "<<d_gains_[4]<<" "<<i_gains_[4]<<endl;
  // cout<<tau_d_calculated[4]<<endl;
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d_saturated[i]);
    }
    
  //   //  FILE*Fuzzy_PID_date1=fopen("/home/liu/liu_test/fuzzy_pid_date1.text","a+");
  //   // fprintf(Fuzzy_PID_date1,"  %f\t  %f\t %f\t %f\t %f\t %f\t %f \n",tau_d_saturated[4],actual[0],target[0],k_gains_[4],d_gains_[4],actual[0]-target[0],elapsed_time_.toSec());
  //   // fclose(Fuzzy_PID_date1);
  //   // FILE*Fuzzy_PID_date2=fopen("/home/liu/liu_test/fuzzy_pid_date2.text","a+");
  //   // fprintf(Fuzzy_PID_date2,"  %f\t %f\t %f\t %f\t %f\t %f\t %f \n",tau_d_saturated[6],actual[1],target[1],k_gains_[6],d_gains_[6],actual[1]-target[1],elapsed_time_.toSec());
  //   // fclose(Fuzzy_PID_date2);
}

void FuzzyExampleController::realize(float t, float a,int index)
{
  float u_e[N],u_de[N],u_u[N];
	int u_e_index[3],u_de_index[3];//假设一个输入最多激活3个模糊子集
	float delta_Kp,delta_Ki,delta_Kd;
	float delta_u;
	// target=t;
	// actual=a;
  //e=target-actual;
  e[index]=t-a;
	de[index]=e[index]-e_pre_1[index];
	e[index]=Ke*e[index];
	de[index]=Kde*de[index];
  //  cout<<"输出误差"<<endl;
  //  cout<<Ke<<" "<<Kde<<endl;
  // cout<<e<<"  "<<de<<endl;
  /* 将误差e模糊化*/
	int j=0;
	for(int i=0;i<N;i++)
	{
		if(mf_t_e=="trimf")
		  u_e[i]=trimf(e[index],e_mf_paras[i*3],e_mf_paras[i*3+1],e_mf_paras[i*3+2]);//e模糊化，计算它的隶属度
		else if(mf_t_e=="gaussmf")
		  u_e[i]=gaussmf(e[index],e_mf_paras[i*2],e_mf_paras[i*2+1]);//e模糊化，计算它的隶属度
		else if(mf_t_e=="trapmf")
		  u_e[i]=trapmf(e[index],e_mf_paras[i*4],e_mf_paras[i*4+1],e_mf_paras[i*4+2],e_mf_paras[i*4+3]);//e模糊化，计算它的隶属度

		if(u_e[i]!=0)
            u_e_index[j++]=i;                //存储被激活的模糊子集的下标，可以减小计算量
  	}
	for(;j<3;j++)u_e_index[j]=0;             //富余的空间填0

	/*将误差变化率de模糊化*/
	j=0;
	for(int i=0;i<N;i++)
	{
		if(mf_t_de=="trimf")
		   u_de[i]=trimf(de[index],de_mf_paras[i*3],de_mf_paras[i*3+1],de_mf_paras[i*3+2]);//de模糊化，计算它的隶属度
		else if(mf_t_de=="gaussmf")
		   u_de[i]=gaussmf(de[index],de_mf_paras[i*2],de_mf_paras[i*2+1]);//de模糊化，计算它的隶属度
		else if(mf_t_de=="trapmf")
		   u_de[i]=trapmf(de[index],de_mf_paras[i*4],de_mf_paras[i*4+1],de_mf_paras[i*4+2],de_mf_paras[i*4+3]);//de模糊化，计算它的隶属度

		if(u_de[i]!=0)
			u_de_index[j++]=i;            //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_de_index[j]=0;          //富余的空间填0

	float den=0,num=0;
	/*计算delta_Kp和Kp*/
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Kp_rule_matrix[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	delta_Kp=num/den;
	delta_Kp=Ku_p*delta_Kp;
  //  cout<<"Kp的变化"<<endl;
  //  cout<<num<<"  "<<den<<endl;
  //  cout<<Ku_p<<"  "<<num/den<<"  "<<delta_Kp<<endl;
	if(delta_Kp>=delta_Kp_max)   delta_Kp=delta_Kp_max;
	else if(delta_Kp<=-delta_Kp_max) delta_Kp=-delta_Kp_max;
	Kp[index]+=delta_Kp;
  // cout<<Kp<<endl;
	if(Kp[index]<0)Kp[index]=0;
	/*计算delta_Ki和Ki*/
	den=0;num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Ki_rule_matrix[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}

	delta_Ki=num/den;
	delta_Ki=Ku_i*delta_Ki;
	if(delta_Ki>=delta_Ki_max)   delta_Ki=delta_Ki_max;
	else if(delta_Ki<=-delta_Ki_max)  delta_Ki=-delta_Ki_max;
	Ki[index]+=delta_Ki;
	if(Ki[index]<0)Ki[index]=0;
	/*计算delta_Kd和Kd*/
	den=0;num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Kd_rule_matrix[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	delta_Kd=num/den;
	delta_Kd=Ku_d*delta_Kd;
  // cout<<"Kd的变化"<<endl;
  // cout<<Ku_d<<"  "<<num/den<<"  "<<delta_Kd<<endl;
	if(delta_Kd>=delta_Kd_max)   delta_Kd=delta_Kd_max;
	else if(delta_Kd<=-delta_Kd_max) delta_Kd=-delta_Kd_max;
	Kd[index]+=delta_Kd;
	if(Kd[index]<0)Kd[index]=0;
  e_pre_2=e_pre_1;
  e_pre_1=e;
  //cout<<e<<endl;
  //cout<<Kp<<"  "<<Kd<<endl;
}

 //模糊函数初始化
void FuzzyExampleController::Init_FuzzyExampleController(double e_max,double de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0)
{
   Ke=(N/2)/e_max;
   //cout<<Ke<<endl;
   //cout<<e_max<<"  "<<de_max<<"  "<<kp_max<<"  "<<"  "<<ki_max<<"  "<<kd_max<<"  "<<Kp0<<"  "<<Ki0<<endl;
   Kde=(N/2)/de_max;
   //cout<<Kde<<endl;
   delta_Kp_max=kp_max;
   delta_Ki_max=ki_max;
   delta_Kd_max=kd_max;
   Ku_p=delta_Kp_max/(N/2);
   Ku_i=delta_Ki_max/(N/2);
   Ku_d=delta_Kd_max/(N/2);
   mf_t_e="No type";
   mf_t_de="No type";
   mf_t_Kp="No type";
   mf_t_Ki="No type";
   mf_t_Kd="No type";
   e_mf_paras=NULL;
   de_mf_paras=NULL;
   Kp_mf_paras=NULL;
   Ki_mf_paras=NULL;
   Kd_mf_paras=NULL;
}

// //设置模糊隶属度函数的子函数
void FuzzyExampleController::setMf_sub(const string &type,float *paras,int n)
{
  int N_mf_e,N_mf_de,N_mf_Kp,N_mf_Ki,N_mf_Kd;
  switch(n)
  {
  case 0:
	  if(type=="trimf"||type=="gaussmf"||type=="trapmf")
	    mf_t_e=type;
	  else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_e=="trimf")
        N_mf_e=3;
	  else if(mf_t_e=="gaussmf")
		N_mf_e=2;
	  else if(mf_t_e=="trapmf")
		N_mf_e=4;
       
	  e_mf_paras=new float [N*N_mf_e];
	  for(int i=0;i<N*N_mf_e;i++)
		e_mf_paras[i]=paras[i];
	  break;

  case 1:
	  if(type=="trimf"||type=="gaussmf"||type=="trapmf")
	    mf_t_de=type;
	  else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_de=="trimf")
        N_mf_de=3;
	  else if(mf_t_de=="gaussmf")
		N_mf_de=2;
	  else if(mf_t_de=="trapmf")
		N_mf_de=4;
        de_mf_paras=new float [N*N_mf_de];
	  for(int i=0;i<N*N_mf_de;i++)
		de_mf_paras[i]=paras[i];
	  break;

   case 2:
	  if(type=="trimf"||type=="gaussmf"||type=="trapmf")
	    mf_t_Kp=type;
	  else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Kp=="trimf")
        N_mf_Kp=3;
	  else if(mf_t_Kp=="gaussmf")
		N_mf_Kp=2;
	  else if(mf_t_Kp=="trapmf")
		N_mf_Kp=4;
        Kp_mf_paras=new float [N*N_mf_Kp];
	  for(int i=0;i<N*N_mf_Kp;i++)
		Kp_mf_paras[i]=paras[i];
	  break;

   case 3:
	  if(type=="trimf"||type=="gaussmf"||type=="trapmf")
	    mf_t_Ki=type;
	  else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Ki=="trimf")
        N_mf_Ki=3;
	  else if(mf_t_Ki=="gaussmf")
		N_mf_Ki=2;
	  else if(mf_t_Ki=="trapmf")
		N_mf_Ki=4;
        Ki_mf_paras=new float [N*N_mf_Ki];
	  for(int i=0;i<N*N_mf_Ki;i++)
		Ki_mf_paras[i]=paras[i];
	  break;

   case 4:
	  if(type=="trimf"||type=="gaussmf"||type=="trapmf")
	    mf_t_Kd=type;
	  else
		cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Kd=="trimf")
        N_mf_Kd=3;
	  else if(mf_t_Kd=="gaussmf")
		N_mf_Kd=2;
	  else if(mf_t_Kd=="trapmf")
		N_mf_Kd=4;
        Kd_mf_paras=new float [N*N_mf_Kd];
	  for(int i=0;i<N*N_mf_Kd;i++)
		Kd_mf_paras[i]=paras[i];
	  break;

   default: break;
  }
}

// //设置模糊隶属度函数的类型和参数
void FuzzyExampleController::setMf(const string & mf_type_e,float *e_mf,
			   const string & mf_type_de,float *de_mf,
			   const string & mf_type_Kp,float *Kp_mf,
		     const string & mf_type_Ki,float *Ki_mf,
			   const string & mf_type_Kd,float *Kd_mf)
         {
          setMf_sub(mf_type_e,e_mf,0);
	        setMf_sub(mf_type_de,de_mf,1);
	        setMf_sub(mf_type_Kp,Kp_mf,2);
	        setMf_sub(mf_type_Ki,Ki_mf,3);
	        setMf_sub(mf_type_Kd,Kd_mf,4);
         }

// //设置模糊规则Matrix
void FuzzyExampleController::setRuleMatrix(int kp_m[N][N],int ki_m[N][N],int kd_m[N][N])
{
  for(int i=0;i<N;i++)
    for(int j=0;j<N;j++)
    {
      Kp_rule_matrix[i][j]=kp_m[i][j];
      Ki_rule_matrix[i][j]=ki_m[i][j];
      Kd_rule_matrix[i][j]=kd_m[i][j];
    }
}

// //三角隶属度函数
float FuzzyExampleController::trimf(float x,float a,float b,float c)
{
  float u;
  if(x>=a&&x<=b)
   u=(x-a)/(b-a);
   else if(x>b&&x<=c)
   u=(c-x)/(c-b);
   else 
   u=0;
   return u;
}

// //高斯隶属度函数
float FuzzyExampleController::gaussmf(float x,float ave,float sigma)
{
  float u;
  if(sigma<0)
  {
    cout<<"In gaussmf, sigma must larger than 0"<<endl;
  }
  u=exp(-pow(((x-ave)/sigma),2));
  return u;
}

float FuzzyExampleController::trapmf(float x,float a,float b,float c,float d)
{
  float u;
	if(x>=a&&x<b)
		u=(x-a)/(b-a);
	else if(x>=b&&x<c)
        u=1;
	else if(x>=c&&x<=d)
		u=(d-x)/(d-c);
	else
		u=0;
	return u;
}

std::array<double, 7> FuzzyExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  for(int i=0;i<4;i++)
  {
    if(tau_d_saturated[i]>10.0) tau_d_saturated[i]=10.0;
    if(tau_d_saturated[i]<-10.0)  tau_d_saturated[i]=-10.0;
  }
  for(int i=4;i<7;i++)
  {
     if(tau_d_saturated[i]>5.0) tau_d_saturated[i]=5.0;
    if(tau_d_saturated[i]<-5.0)  tau_d_saturated[i]=-5.0;
  }

  return tau_d_saturated;
}


FuzzyExampleController::~FuzzyExampleController()
{
  delete [] e_mf_paras;
  delete [] de_mf_paras;
  delete [] Kp_mf_paras;
  delete [] Ki_mf_paras;
  delete [] Kd_mf_paras;
}
}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::FuzzyExampleController,
                       controller_interface::ControllerBase)