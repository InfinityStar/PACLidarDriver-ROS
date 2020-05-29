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

#define DEGTORAD(x) (x * M_PI / 180.0)

namespace PacLidar
{
    enum lidarCMD
    {
        CTRL_START = 0x7362,
        CTRL_STOP = 0x6f70,
        SET_SPEED_HZ_10 = 0x500a,
        SET_SPEED_HZ_15 = 0x500f,
        SET_SPEED_HZ_20 = 0x5014,
        SET_DATA_ORIGINAL = 0x6474,
        SET_DATA_CHECKED = 0x6475,
    };

    enum lidarDataDef
    {
        LIDAR_DATA_NULL = 0x0000,
        LIDAR_DATA_HEAD = 0x0010,
        LIDAR_DATA_END = 0x8ca0,
    };

#pragma pack(1)
    typedef struct
    {
        uint16_t part1;
        uint16_t part2;
        uint16_t part3;
    } LidarData_t;
#pragma pack()

    static const LidarData_t LIDAR_DATA_HEADER_T = {LIDAR_DATA_HEAD, LIDAR_DATA_NULL, LIDAR_DATA_NULL};
    static const LidarData_t LIDAR_DATA_TAIL_T = {LIDAR_DATA_END, LIDAR_DATA_NULL, LIDAR_DATA_NULL};

} // namespace PacLidar

#endif //__PAC_LIDAR_COMMON_H__