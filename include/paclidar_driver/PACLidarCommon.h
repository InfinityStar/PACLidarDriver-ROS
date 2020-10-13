/** ***************************************************************** 
* 版权所有：北京光勺科技有限公司
* 文件名：PACLidarCommon.h
* 文件功能描述：PAC雷达通用头文件
* 作者：Roser
* 维护者：Roser
* Email：roserxy@163.com
* 时间：2020-6
* 创建标识：Init 
* *******************************************************************/

#ifndef __PAC_LIDAR_COMMON_H__
#define __PAC_LIDAR_COMMON_H__

#include <iostream>
#include <string>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <stdlib.h>
#include <cerrno>
#include <vector>
#include <assert.h>
#include <cmath>
#include <sys/time.h>

#define PAC_NUM_OF_ONE_PKG  1020
#define PAC_NUM_OF_ONE_SCAN 1000

#define PAC_IDX_OF_TMP 1002
#define PAC_IDX_OF_ID  1003
#define PAC_IDX_OF_VER 1004
#define PAC_IDX_OF_SPD 1005

#define PAC_MAX_RANGE        60.0 //m
#define PAC_MIN_RANGE        0.05 //m
#define PAC_MAX_BEAMS        5760
#define PAC_ANGLE_RESOLUTION 0.0625

#ifndef DEGTORAD
#define DEGTORAD(x) (x * M_PI / 180.0)
#endif//DEGTORAD

/*Using for terminal text color*/
#define T_RESET   "\033[0m"
#define T_BLACK   "\033[30m"      /* Black */
#define T_RED     "\033[31m"      /* Red */
#define T_GREEN   "\033[32m"      /* Green */
#define T_YELLOW  "\033[33m"      /* Yellow */
#define T_BLUE    "\033[34m"      /* Blue */
#define T_MAGENTA "\033[35m"      /* Magenta */
#define T_CYAN    "\033[36m"      /* Cyan */
#define T_WHITE   "\033[37m"      /* White */
#define T_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define T_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define T_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define T_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define T_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define T_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define T_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define T_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*Using for terminal text color*/

namespace PacLidar
{
    enum lidarCMD
    {
        CTRL_START        = 0x7362,
        CTRL_STOP         = 0x6f70,
        CTRL_BOOT_LOAD    = 0xaabb,
        SET_SPEED_HZ_10   = 0x500a,
        SET_SPEED_HZ_15   = 0x500f,
        SET_SPEED_HZ_20   = 0x5014,
        SET_SPEED_HZ_25   = 0x5019,
        SET_SPEED_HZ_30   = 0x501e,
        NO_WAVE_FILTERING=  0x6474,
        WAVE_FILTERING_1 =  0x6475,
        WAVE_FILTERING_2 =  0x6476,
        WAVE_FILTERING_3 =  0x6477,
        WAVE_FILTERING_4 =  0x6478,
        WAVE_FILTERING_5 =  0x6479,
        WAVE_FILTERING_6 =  0x647a,
        HEART_BEAT        = 0x0101,
    };

    enum lidarDataDef
    {
        LIDAR_DATA_NULL = 0x0000,
        LIDAR_DATA_HEAD = 0x0010,
        LIDAR_DATA_END  = 0x8ca0,
    };

    typedef struct
    {
        uint32_t version    = 0;
        uint32_t id         = 0;
        int      temprature = 0;
        uint16_t speed      = 0;
    }  lidarState_t;

#pragma pack(1)
    typedef struct
    {
        uint16_t part1;
        uint16_t part2;
        uint16_t part3;
    } LidarData_t;
#pragma pack()

    static const LidarData_t LIDAR_DATA_HEADER_T = {LIDAR_DATA_HEAD, LIDAR_DATA_NULL, LIDAR_DATA_NULL};
    static const LidarData_t LIDAR_DATA_TAIL_T   = {LIDAR_DATA_END,  LIDAR_DATA_NULL, LIDAR_DATA_NULL};

} // namespace PacLidar

#endif //__PAC_LIDAR_COMMON_H__