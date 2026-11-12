#ifndef HPS_FTC_LIB_H
#define HPS_FTC_LIB_H
#define _LINUX_
#ifdef _LINUX_
    #ifdef HPS_FTC_LIB_LIBRARY
        #define HPS_FTC_API __attribute__((visibility("default")))
    #else
        #define HPS_FTC_API
    #endif
#else
    #ifdef HPS_FTC_LIB_LIBRARY
        #define HPS_FTC_API __declspec(dllexport)
    #else
        #define HPS_FTC_API __declspec(dllimport)
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define HPS_FTC_NULL -1
#define HPS_FTC_FAIL 0
#define HPS_FTC_SUCCESS 1
#define HPS_FTC_SIZEOF_6 6
#define HPS_FTC_SIZEOF_3 3
#define HPS_FTC_SIZEOF_7 7
#define HPS_FTC_PI 3.14159265358979323846
#define HPS_FTC_SDK_VERSION_ "2.0.12"       //230704
#define HPS_FTC_ERR_MSG_MAX_LEN 256

#define DOUBLE_SET_APPROACH_MINPATH 5001    //设置接触模式允许中断触发的最小修正距离 mm
#define DOUBLE_SET_APPROACH_ENDFORCE 5002    //设置接触模式允许中断触发的最小修正距离 mm
#define  IN
#define  OUT


    // 传感器量程信息,单位:(N,N,N,Nm,Nm,Nm)
    typedef double hps_ft_data[HPS_FTC_SIZEOF_6];

    typedef double hps_pose_quaternion_data[HPS_FTC_SIZEOF_7];

    // 机器人位置,单位:(mm,mm,mm,rad_RZ,rad_RY,rad_RX)
    typedef double hps_robot_pose[HPS_FTC_SIZEOF_6];

    // 修正范围,基于基座标系,单位:(mm,mm,mm)
    typedef double hps_correctionLimit[HPS_FTC_SIZEOF_3];

    // 柔顺系数
    typedef double hps_kr[HPS_FTC_SIZEOF_6];

    typedef enum {
        RCS_WORLD,			// 世界坐标系
        RCS_BASE,			// 基坐标系
        RCS_TOOL,			// 工具坐标系
        RCS_SENSOR,			// 传感器坐标系
    } IN HpsRcs;

    typedef enum {
        FT_FX,					// 控制主方向Fx(N)
        FT_FY,					// 控制主方向Fy(N)
        FT_FZ,					// 控制主方向Fz(N)
        FT_MX,					// 控制主方向Mx(Nm)
        FT_MY,					// 控制主方向MY(Nm)
        FT_MZ,					// 控制主方向MZ(Nm)
    }IN HpsDirection;

    enum HpsFTCCoord {
        FTC_ERR_SENSOR_TIMEOUT = 10001,			//传感器超时
        FTC_ERR_OVERLOAD = 10002,				//过载
        FTC_ERR_OVERMONITORING = 11000,			//超过监控力值
        FTC_ERR_NO_CONFIG_FTC = 11002,			//力控基本参数未设置
        FTC_ERR_FTCMOVEMOD = 11003,				//运动指令错乱
        FTC_ERR_NO_CONFIG_FTCMOVEMOD = 11004,	//力控运动参数未设置
        FTC_ERR_OVERPOSSCORRMON = 11005,		//超过总修正值
        FTC_ERR_TIMEOUT = 11006,				//修正超时
        FTC_END = 80001,						//修正结束

        //中断触发状态码 dec_code= bin_fx*64+bin_fy*32+bin_fz*8+bin_mx*4+bin_my*2+bin_mz
        //FTC_END_XX = 80002+dec_code;          //修正中断触发结束
        FTC_RUNING = 80000,						//运行中
    };

    enum HpsFTCoord {
        HPS_FT_RUN = 10000,							//传感器运行
        HPS_ETHERNET_NO_SENSOR_ERROR = 11000,		//转接盒未连接传感器
        HPS_FT_GET_MATRIX_ERROR = 11001,			//传感器获取校正数据失败
        HPS_FT_TEMP_COF_ERROR = 11002,				//传感器温度数据异常
        HPS_FT_ADC_GAIN_ERROR = 11003,				//传感器ADC值获取异常
        HPS_FT_ADC_NUM_ERROR = 11004,				//传感器ADC数值异常
        HPS_FT_ZERO_RESET_ERROR = 11005,			//传感器清零失败
        HPS_FT_SET_DAC_ERROR = 11006,				//传感器设置ADC失败
        HPS_FT_NULL_MATRIX_ERROR = 11007,			//传感器无校正记录
        HPS_FT_DATA_ERROR = 11008,					//传感器数据异常（过载导致损坏）
        HPS_FT_ATTITUDE_SENSOR_INIT_ERROR = 11009,	//姿态传感器初始化失败
        HPS_FT_ATTITUDE_SENSOR_DATA_ERROR = 11010,	//姿态传感器输出数据错误
        HPS_FT_OVERLOAD_ERROR = 11011,				//传感器当前过载
        HPS_FT_NO_REFERENCE_VOLTAGE_ERROR = 11012,	//传感器不存在无负载基准电压
        HPS_FT_NO_CROSSTALK_MATRIX_ERROR = 11013,	//传感器无串扰补偿
        //通讯状态
        HPS_COMM_RUN = 20000,						//运行
        HPS_COMM_CLOSE = 20001,						//关闭
        HPS_COMM_INIT = 20002,						//初始化
        HPS_COMM_IP_ERROR = 21000,					//IP错误
        HPS_COMM_PORT_ERROR = 21001,				//端口错误
        HPS_COMM_TIMEOUT_ERROR = 21002,				//连接超时
        HPS_COMM_CMDRETURN_ERROR = 21003,			//指令设置错误
        HPS_COMM_THREAD_OPEN_ERROR = 21004,			//线程打开失败
    };

    typedef struct hps_ftc_error_t {
        int ftc_code;
        int ft_code;
        int ft_ipoc;
    } hps_ftc_error;

    /**
    *开启力控功能,创建相关函数指针
    */
    HPS_FTC_API int  hps_ftc_open(bool isExternal);

    /**
    *关闭力控功能
    */
    HPS_FTC_API int  hps_ftc_close(void);



    /********************************力控部分********************************/
    /**
    *设置力接触模式下的参数,在未与物体接触时,通过该模式靠近.
    *@rcs					柔顺控制的参考坐标系
    *@mainDirection			柔顺控制的主要方向
    *@setExpectFT			柔顺控制的主要方向的期望力(N Nm)
    *@setKR					柔顺系数,力为(mm/s)/N 扭矩为('/s)/Nm 除力控的主要方向外其他方向默认为0,修改意味着该维度柔顺.
    *@translationalMotion	未接触期间的速度(mm/s)
    *@switchingForce		进入接触控制的判断条件F,越大越容易超调(N)
    *@switchingTorque		进入接触控制的判断条件M,越大越容易超调(Nm)
    *@breakConditionRange	中断触发条件,设定为期望力的偏差范围,达到后接触模式退出（标量）
    *@timeOut				接触运动最长时间,达到后退出(ms)
    */
    HPS_FTC_API int  hps_ftc_configApproachMode(
        IN HpsRcs rcs,
        IN HpsDirection mainDirection,
        IN double setExpectFT,
        IN hps_kr setKR,
        IN double translationalMotion,
        IN double switchingForce,
        IN double switchingTorque,
        IN hps_ft_data breakConditionRange,
        IN unsigned long timeOut);

    /**
    *设置力装配模式下的参数,在未与物体接触时,通过该模式靠近.
    *@rcs					柔顺控制的参考坐标系
    *@mainDirection			柔顺控制的主要方向
    *@setExpectFT			柔顺控制的主要方向的期望力(N Nm)
    *@setKR					柔顺系数,力为(mm/s)/N 扭矩为('/s)/Nm 除力控的主要方向外其他方向默认为0,修改意味着该维度柔顺.
    *@translationalMotion	未接触期间的速度(mm/s)
    *@switchingForce		进入接触控制的判断条件F,越大越容易超调(N)
    *@switchingTorque		进入接触控制的判断条件M,越大越容易超调(Nm)
    *@breakConditionRange	中断触发条件,设定为期望力的偏差范围,达到后接触模式退出（标量）
    *@timeOut				接触运动最长时间,达到后退出(ms)
    *@pathOut               装配运动最远距离（超过后退出）
    *@minPath               力控最小路径
    *@minPathForce          力控最小路径力阈值（超过后退出）
    */
   HPS_FTC_API int hps_ftc_configAssemblyMode(
        IN HpsRcs rcs,
        IN HpsDirection mainDirection,
        IN double setExpectFT,
        IN hps_kr setKR,
        IN double translationalMotion,
        IN double switchingForce,
        IN double switchingTorque,
        IN hps_ft_data breakConditionRange,
        IN unsigned long timeOut,
        IN double  pathOut,
        IN double minPath,
        IN double minPathForce);

    /**
    *配置力跟随模式的参数,在已经与物体接触时,通过该控制实时修正路径,达到柔顺效果.
    *@rcs                   柔顺控制的参考坐标系
    *@mainDirection         柔顺控制的主要方向
    *@eFT                   柔顺控制的期望力
    *@kr                    柔顺系数,若设置为零代表该方向不柔顺.力为(mm/s)/N 扭矩为('/s)/Nm
    *@breakConditionRange	中断触发条件,设定为期望力的偏差范围,力跟踪误差,达到后接触模式退出（矢量）
    *@desiredSpeed          期望的速度(mm/s)
    */
    HPS_FTC_API int  hps_ftc_configTrackingMotionMode(
        IN HpsRcs rcs,
        IN HpsDirection mainDirection,
        IN hps_ft_data expectFT,
        IN hps_kr kr,
        IN hps_ft_data breakConditionRange,
        IN double desiredSpeed);

    /**
    *设置传感器配置，传感器相对法兰盘偏移量和工具相对传感器偏移量，要使用力控功能必须先设置此函数
    *@ml				传感器量程
    *@flange_sensor		传感器受力点与机器人末端法兰偏移(mm,mm,mm,rad_RZ,rad_RY,rad_RX)
    *@sensor_tool       机器人工具端与传感器偏移(mm,mm,mm,rad_RZ,rad_RY,rad_RX)
    */
    HPS_FTC_API int  hps_ftc_setSensorConfig(IN hps_ft_data maxLoad, IN hps_robot_pose flange_sensor,IN hps_robot_pose sensor_tool);

    /**
    *设置负载(直接已知,或通过重力标定获取)
    *@g		重力N
    *@lx	重心位置,相对于默认工具x方向(mm) (Max 500mm)
    *@ly	重心位置,相对于默认工具y方向(mm) (Max 500mm)
    *@lz	重心位置,相对于默认工具z方向(mm) (Max 500mm)
    */
    HPS_FTC_API int  hps_ftc_setLoadData(IN double g, IN double lx,IN double ly,IN double lz);

    /**
    *设定修正限幅,超过后限幅
    *@max	基于基座标下位置的正修正范围(mm)
    *@min	基于基座标下位置的负修正范围(mm)(输入值为负值)
    *@ang	基于基座标下位置的角度修正范围(角度)
    */
    HPS_FTC_API int  hps_ftc_setCorrectionLimit(IN hps_correctionLimit max,IN hps_correctionLimit min,IN double ang);


    /*最小修正距离*/
    HPS_FTC_API int  hps_set_minValue(IN double value);
    /*各方向最小修正距离*/
    HPS_FTC_API int  hps_set_minValue_every(IN double value_group[HPS_FTC_SIZEOF_3]);

    /**
    *力控过程的力监控,超过后停止
    *@max		传感器各维度的范围(N,Nm)
    */
    HPS_FTC_API int  hps_ftc_setMonitoringFunctions(IN hps_ft_data _monitoringFunctions);

    /**
    *总修正量判断,超过后停止
    *@path		累计修正位置(mm)
    *@angle		累计修正角度(角度)
    */
    HPS_FTC_API int  hps_ftc_setCorrectionMonitoring(IN double path, IN double angle);

    /**
    *运行接触模式,超时或者触发中断后退出,修正过大或力过载退出,返回修正路径.(默认插补8ms)
    *@out_pos			路径插补点(mm,rad(ZYX))
    *@robot_pos_now		机器人实时位置(mm,rad(ZYX))(建议直接用上一插补周期的out_pos)
    *@error			错误信息
    *@return			是否继续运行
    */
    HPS_FTC_API int  hps_ftc_runApproach(
        OUT hps_robot_pose out_pos,
        IN hps_robot_pose robot_pos_now,
        OUT hps_ftc_error *err);

    /**
    *运行装配模式,超时或者触发中断后退出,修正过大或力过载退出,返回修正路径.(默认插补8ms)
    *@&out_pos			路径插补点(mm,rad(ZYX))
    *@robot_pos_now		机器人实时位置(mm,rad(ZYX))(建议直接用上一插补周期的out_pos)
    *@&error			错误信息
    *@return			是否继续运行
    */
    HPS_FTC_API int  hps_ftc_runAssembly(
        OUT hps_robot_pose out_pos,
        IN hps_robot_pose robot_pos_now,
        OUT hps_ftc_error *err);

    /**
    *运行路径修正模式,路径结束后退出,修正过大或力过载退出,返回修正路径.(默认插补8ms)
    *@out_pos			路径插补点(mm,rad(ZYX))
    *@robot_pos_start	直线运动起点位置(mm,rad(ZYX))(起始点应当为当前机器人实际位置)
    *@robot_pos_end		直线运动终点位置(mm,rad(ZYX))
    *@robot_pos_now		机器人实时位置(mm,rad(ZYX))(建议直接用上一插补周期的out_pos)
    *@error			错误信息
    *@return			是否继续运行
    */
    HPS_FTC_API int  hps_ftc_runTrackingMotionL(
        OUT hps_robot_pose out_pos,
        IN hps_robot_pose robot_pos_start,
        IN hps_robot_pose robot_pos_end,
        IN hps_robot_pose robot_pos_now,
        OUT hps_ftc_error *err);

    /**
    *运行路径修正模式,路径结束后退出,修正过大或力过载退出,返回修正路径.(默认插补8ms)
    *@out_pos				路径插补点(mm,rad(ZYX))
    *@robot_pos_start		圆弧运动起点位置(mm,rad(ZYX))(起始点应当为当前机器人实际位置)
    *@robot_pos_transition	圆弧运动中间点位置(mm,rad(ZYX))
    *@robot_pos_end			圆弧运动终点位置(mm,rad(ZYX))
    *@robot_pos_now			机器人实时位置(mm,rad(ZYX))(建议直接用上一插补周期的out_pos)
    *@error				错误信息
    *@return				是否继续运行
    */
    HPS_FTC_API int  hps_ftc_runTrackingMotionC(
        OUT hps_robot_pose out_pos,
        IN hps_robot_pose robot_pos_start,
        IN hps_robot_pose robot_pos_transition,
        IN hps_robot_pose robot_pos_end,
        IN hps_robot_pose robot_pos_now,
        hps_ftc_error *err);
    /********************************力控部分************************************************************/


    /********************************重力补偿相关********************************/
    /**
    *初始化传感器数据,机器人带负载时的零点，相当于清零(应当不与外界接触).
    *力控清零标志位,与传感器清零不冲突.
    */
    HPS_FTC_API int  hps_ftc_sensorReset(IN hps_robot_pose robot_pos);

    /**
    *记录重力标定数据(外部数据标定)
    *1.机器人可以运动3--16个标定位姿,其中3个标定位姿,必须是(传感器FX,FY,FZ正方向与重力方向相同)
    *2.其他标定位姿选取姿态变化大,或者重力方向在传感器负方向的位姿.
    *3.必须让机器人运动到标定位姿,待机器人停止后读取机器人当前工具和当前位姿.
    */
    HPS_FTC_API int  hps_ftc_recordCalibrationLoadData(IN hps_ft_data ft, IN hps_robot_pose robot_pos);

    /**
    *初始化传感器重力标定数据.
    */
    HPS_FTC_API int  hps_ftc_initCalibrationLoadData();

    /**
    *进行传感器重力标定.若成功会直接替换内部相关数据.负载不变的情况下只需要标定一次即可.
    */
    HPS_FTC_API int  hps_ftc_gravityCalibration();

    /**
    *获取当前设置或标定的负载数据.(在负载不变情况下,只需要一次标定,可以将负载数据记录下来)
    */
    HPS_FTC_API int  hps_ftc_getLoadData(OUT double *g, OUT double *lx,OUT double *ly, OUT double *lz);

    /**
    *获取当前位姿下,重力补偿后的(不同坐标系下的)传感器数据
    *@ft			返回值,获取的力分量(N,Nm)
    *@robot_pos		机器人实际姿态(mm,rad(ZYX))
    *@rcs			力矢量的参考坐标系(BASE,TOOL,SENSOR)
    *@return		获取数据是否成功
    */
    HPS_FTC_API int  hps_ftc_getSensorData(OUT hps_ft_data ft,IN hps_robot_pose robot_pos, IN HpsRcs rcs);

    /*获取工具坐标系下受力数据*/
    HPS_FTC_API int  hps_ftc_getRCS_FT_Data(OUT hps_ft_data ft);


    /**
    *设置传感器噪声阈值,绝对值低于设定值直接清零
    *@f_min		默认为Fmin=0.2N,设定值范围(0--100)
    *@m_min		默认为Mmin=0.002Nm,设定值范围(0--10)
    *@return	设定是否成功
    */
    HPS_FTC_API int  hps_ftc_setSensorNoiseThreshold(IN double f_min,IN double m_min);
    /********************************重力补偿部分end********************************/


    /**
    *清除当前运动参数
    */
    HPS_FTC_API int  hps_ftc_clear();


    /**
    *设置插补周期,4-20ms
    */
    HPS_FTC_API int  hps_ftc_setTimeStep(IN double msTime);

    /**
    *获取插补周期
    */
    HPS_FTC_API int  hps_ftc_getTimeStep(OUT double *msTime);



    /********************************传感器部分********************************/
    /**
    *读取原始六维力数据
    */
    HPS_FTC_API int  hps_ftc_obtainFTSensorData(OUT hps_ft_data m_ftData);

    /**
    *连接力传感器
    */
    HPS_FTC_API int  hps_ftc_initialFTSensor(IN short unsigned int host,IN const char *ip);

    /**
    *获取传感器量程
    */
    HPS_FTC_API int  hps_ftc_getFTSensorRange(OUT hps_ft_data range);

    /**
    *断开传感器连接
    */
    HPS_FTC_API int  hps_ftc_uninitialFTSensor();

    /**
    *传感器清零
    */
    HPS_FTC_API int  hps_ftc_zeroFTSensor();

    /**
    *设置传感器低通滤波器(0-7)
    */
    HPS_FTC_API int  hps_ftc_lowPassFilterFTSensor(IN int range);

    /**
    *设置传感器中值滤波器(0-64)
    */
    HPS_FTC_API int  hps_ftc_medianFilterDepthFTSensor(IN int range);
    /********************************传感器部分end********************************/

    /*欧拉角转四元数*/
    HPS_FTC_API int  hps_rpy2quaternion(OUT hps_pose_quaternion_data pose_qua,IN hps_robot_pose pose);
    /*四元数转欧拉角*/
    HPS_FTC_API int  hps_quaternion2rpy(OUT hps_robot_pose pose, IN hps_pose_quaternion_data pose_qua);

    HPS_FTC_API int  hps_pose2offset(OUT hps_robot_pose offset, IN hps_robot_pose p1, IN hps_robot_pose p2);




#ifdef __cplusplus
}
#endif
#endif // HPS_FTC_LIB_H
