#ifndef HPS_FT_SENSOR_HPS_SENSOR_H_
#define HPS_FT_SENSOR_HPS_SENSOR_H_

#include "hps_ft_sensor/hps_ftc_lib.h"

class hps_sensor
{
    public:

        hps_sensor();
        ~hps_sensor();
        //机器人现在的位置，通过机器人获取
        hps_robot_pose robot_pos_now;
        //传给机器人法兰盘的路点
        hps_robot_pose out_pos;


        hps_pose_quaternion_data temp_qua={0};
        double approach_joint[6]={0};

        const char* TOOL_IO_0_test = "T_DI/O_00";

        //设定重力补偿阈值
        double f_min = 0.3;
        double m_min = 0.005;
        //重力G以及重心x,y,z
        double G = 4.9527011155795826;
        double x = -1.6772825032937833;
        double y= 33.547618505588112;
        double z = 48.183012468727391;


        hps_ftc_error ftc_error;

        hps_ft_data ft;

        double ftc_joint[6] = { 45,0,-90,0,-90,-90 };

        double joint[6]={0};

        hps_robot_pose flange_sensor = { 0,0,45,1.5708,0,0 };
        //工具在法兰下偏移

        hps_robot_pose sensor_tool = { 0,0,45,0,0,0 };

        bool is_ftc_connection = false;


        int  ftc_open(short unsigned int host, const char *ip,
                      bool zero_on_connect = true);
        int  ftc_close();
        bool connect_sensor(short unsigned int host, const char *ip,
                            bool zero_on_connect = true);
        int zero_sensor();

        int ftc_config_Sensor();
        int ftc_config_LDD(bool isRunCalibration, double g, double lx, double ly, double lz);
        int ftc_config_TimeStep();
        int ftc_config_init();
        bool config_ftc_control();

        int ftc_configApproachMode();
        bool run_ftc_Approach();
        int get_ftData(double ft[6]);
};

#endif  // HPS_FT_SENSOR_HPS_SENSOR_H_
