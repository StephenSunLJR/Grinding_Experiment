#include "hps_ft_sensor/hps_sensor.h"

#include <cstdio>
#include <unistd.h>

#define Sleep(x) usleep((x) * 1000)

using namespace std;

hps_sensor::hps_sensor()
{
}

hps_sensor::~hps_sensor()
{
    if (is_ftc_connection)
    {
        ftc_close();
    }
}

int hps_sensor::ftc_open(short unsigned int host, const char *ip,
                         bool zero_on_connect)
{
    if (hps_ftc_open(false) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_open err : \n");
        return -1;
    }

    if (hps_ftc_initialFTSensor(host, ip) == HPS_FTC_SUCCESS)
    {

        // for (i = 0; i < 10; i++)
        // {
        //     if (hps_ftc_obtainFTSensorData(ft) == HPS_FTC_SUCCESS)
        //     {
        //         printf("func : %f %f %f %f %f %f\n", ft[0], ft[1], ft[2], ft[3], ft[4], ft[5]);
        //         Sleep(100);
        //     }
        // }

        if (zero_on_connect && hps_ftc_zeroFTSensor() != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_zeroFTSensor err : \n");
            hps_ftc_uninitialFTSensor();
            hps_ftc_close();
            return -1;
        }

        // for (i = 0; i < 10; i++)
        // {
        //     if (hps_ftc_obtainFTSensorData(ft) == HPS_FTC_SUCCESS)
        //     {
        //         printf("func : %f %f %f %f %f %f\n", ft[0], ft[1], ft[2], ft[3], ft[4], ft[5]);
        //         Sleep(100);
        //     }
        // }
    }
    else
    {
        printf("hps_ftc_initialFTSensor err : \n");
        hps_ftc_close();
        return -1;
    }
    return 1;
}

int hps_sensor::ftc_close()
{
    if (!is_ftc_connection)
    {
        return 1;
    }
    const int uninitialize_result = hps_ftc_uninitialFTSensor();
    const int close_result = hps_ftc_close();
    is_ftc_connection = false;
    if (uninitialize_result != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_uninitialFTSensor err : \n");
    }
    if (close_result != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_close err : \n");
    }
    return uninitialize_result == HPS_FTC_SUCCESS &&
                   close_result == HPS_FTC_SUCCESS
               ? 1
               : -1;
}

bool hps_sensor::connect_sensor(short unsigned int host, const char *ip,
                                bool zero_on_connect)
{
    if (is_ftc_connection)
    {
        return true;
    }
    if (ftc_open(host, ip, zero_on_connect) == 1)
    {
        is_ftc_connection = true;
    }
    else
    {
        is_ftc_connection = false;
    }
    return is_ftc_connection;
}

int hps_sensor::zero_sensor()
{
    if (!is_ftc_connection)
    {
        return -1;
    }
    return hps_ftc_zeroFTSensor() == HPS_FTC_SUCCESS ? 1 : -1;
}

int hps_sensor::ftc_config_Sensor()
{
    hps_ft_data maxLoad = {60, 60, 60, 5, 5, 5};

    if (hps_ftc_lowPassFilterFTSensor(3) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_lowPassFilterFTSensor err : \n");
        return -1;
    }
    if (hps_ftc_medianFilterDepthFTSensor(3) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_medianFilterDepthFTSensor err : \n");
        return -1;
    }

    if (hps_ftc_setSensorConfig(maxLoad, flange_sensor, sensor_tool) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_setSensorConfig err : \n");
        return -1;
    }

    return 1;
}

int hps_sensor::ftc_config_TimeStep()
{
    double tStep = 0;

    if (hps_ftc_setTimeStep(4) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_setTimeStep err : \n");
        return -1;
    }
    else
    {
        if (hps_ftc_getTimeStep(&tStep) == HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_getTimeStep %f : \n", tStep);
        }
    }
    return 1;
}

int hps_sensor::ftc_config_init()
{
    hps_correctionLimit maxLimit = {50, 50, 50};
    hps_correctionLimit minLimit = {-50, -50, -50};
    hps_ft_data mFunctions = {40, 40, 40, 5, 5, 5};

    if (hps_ftc_setCorrectionLimit(maxLimit, minLimit, 5) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_setCorrectionLimit err : \n");
        return -1;
    }

    if (hps_ftc_setMonitoringFunctions(mFunctions) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_setMonitoringFunctions err : \n");
        return -1;
    }
    return 1;
}

bool hps_sensor::config_ftc_control()
{
    if (!is_ftc_connection)
    {
        return false;
    }
    return ftc_config_Sensor() == 1 &&
           ftc_config_LDD(false, G, x, y, z) == 1 &&
           hps_ftc_setSensorNoiseThreshold(f_min, m_min) == HPS_FTC_SUCCESS &&
           ftc_config_TimeStep() == 1 && ftc_config_init() == 1;
}

int hps_sensor::ftc_configApproachMode()
{
    // 设定装配相关参数
    HpsRcs rcs = RCS_TOOL;
    HpsDirection mainDirection = FT_FZ;
    hps_kr kr = {0.5, 0.5, 0.2, 0, 0, 0};
    // 中断触发条件, 设定为期望力的偏差范围, 力跟踪误差, 达到后接触模式退出（矢量）
    hps_ft_data breakConditionRange = {0.1, 0.1, 0.5, 0.1, 0.1, 0.1};
    // 最小修正距离
    double min_offect[3] = {0, 0, 1};
    int a = 0;
    // hps_set_minValue_every(min_offect);
    // 总修正量
    if (hps_ftc_setCorrectionMonitoring(150, 6) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_setCorrectionMonitoring err : \n");
        return -1;
    }
    // 力接触模式下的参数
    hps_ftc_configApproachMode(
        rcs,
        mainDirection,
        20, kr, 3, 3, 0.2,
        breakConditionRange, 10000);
    return 1;
}

int hps_sensor::ftc_config_LDD(bool isRunCalibration, double g, double lx, double ly, double lz)
{
    double _g, _lx, _ly, _lz;
    if (isRunCalibration)
    {
        // 机器人六个点位坐标以及对应的传感器数据
        hps_robot_pose ps1 = {0};
        hps_ft_data ft1 = {0};
        hps_robot_pose ps2 = {0};
        hps_ft_data ft2 = {0};
        hps_robot_pose ps3 = {0};
        hps_ft_data ft3 = {0};
        hps_robot_pose ps4 = {0};
        hps_ft_data ft4 = {0};
        hps_robot_pose ps5 = {0};
        hps_ft_data ft5 = {0};
        hps_robot_pose ps6 = {0};
        hps_ft_data ft6 = {0};

        if (hps_ftc_initCalibrationLoadData() != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft1, ps1) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft2, ps2) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft3, ps3) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft4, ps4) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft5, ps5) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_recordCalibrationLoadData(ft6, ps6) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_recordCalibrationLoadData err : \n");
            return -1;
        }
        if (hps_ftc_gravityCalibration() != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_gravityCalibration err : \n");
            return -1;
        }
        if (hps_ftc_getLoadData(&_g, &_lx, &_ly, &_lz) == HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_getLoadData %f %f %f %f : \n", _g, _lx, _ly, _lz);
        }
    }
    else
    {
        if (hps_ftc_setLoadData(g, lx, ly, lz) != HPS_FTC_SUCCESS)
        {
            printf("hps_ftc_setLoadData err : \n");
            return -1;
        }
        else
        {
            if (hps_ftc_getLoadData(&_g, &_lx, &_ly, &_lz) == HPS_FTC_SUCCESS)
            {
                printf("hps_ftc_getLoadData %f %f %f %f : \n", _g, _lx, _ly, _lz);
            }
        }
    }
    return 1;
}

bool hps_sensor::run_ftc_Approach()
{
    printf("run_ftc_Approach\n");

    int ftc1 = HPS_FTC_SUCCESS;

    printf("robot_joint_now : %f %f %f %f %f %f\n",
           ftc_joint[0] * 180 / HPS_FTC_PI, ftc_joint[1] * 180 / HPS_FTC_PI, ftc_joint[2] * 180 / HPS_FTC_PI,
           ftc_joint[3] * 180 / HPS_FTC_PI, ftc_joint[4] * 180 / HPS_FTC_PI, ftc_joint[5] * 180 / HPS_FTC_PI);
    printf("robot_pos_now : %f %f %f %f %f %f\n",
           robot_pos_now[0], robot_pos_now[1], robot_pos_now[2],
           robot_pos_now[3], robot_pos_now[4], robot_pos_now[5]);

    Sleep(1000);

    // 初始化传感器数据,相当于清零
    if (hps_ftc_sensorReset(robot_pos_now) != HPS_FTC_SUCCESS)
    {
        printf("hps_ftc_sensorReset err : \n");
        return -1;
    }

    int step = 0;
    ftc1 = ftc_configApproachMode();
    hps_ft_data ft_c = {0};
    while (ftc1 == HPS_FTC_SUCCESS)
    {
        // 运行接触模式,超时或者触发中断后退出,修正过大或力过载退出,返回修正路径.(默认插补8ms)
        ftc1 = hps_ftc_runApproach(out_pos, robot_pos_now, &ftc_error);
        if (ftc1 != HPS_FTC_SUCCESS)
        {
            // faile
            printf("hps_ftc_runApproach : %i %i %i\n", ftc_error.ftc_code, ftc_error.ft_code, ftc_error.ft_ipoc);
            printf("robot_pos_now : %f %f %f %f %f %f\n",
                   robot_pos_now[0], robot_pos_now[1], robot_pos_now[2],
                   robot_pos_now[3], robot_pos_now[4], robot_pos_now[5]);
        }

        for (size_t i = 0; i < HPS_FTC_SIZEOF_6; i++)
        {
            robot_pos_now[i] = out_pos[i];
        }

        hps_ftc_obtainFTSensorData(ft);

        hps_ftc_getRCS_FT_Data(ft_c);

        Sleep(4);
        step += 1;
    }
    printf("step : %i\n", step);

    for (size_t i = 0; i < HPS_FTC_SIZEOF_6; i++)
    {
        robot_pos_now[i] = out_pos[i];
    }

    printf("step : %i\n", step);

    printf("run_ftc_Approach end\n");

    return true;
}

int hps_sensor::get_ftData(double *ft)
{
    if (!is_ftc_connection || ft == nullptr)
    {
        return -1;
    }
    return hps_ftc_obtainFTSensorData(ft);
}
