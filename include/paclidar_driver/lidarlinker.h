/** ***************************************************************** 
* 版权所有：北京光勺科技有限公司
* 文件名：lidarlinker.h
* 文件功能描述：连接并驱动PAC雷达的C++ Class,适用于Linux
* 作者：Roser
* 维护者：Roser
* Email：roserxy@163.com
* 时间：2020-6
* 创建标识：Init 
* *******************************************************************/

#ifndef __LIDAR_MANAGER_H__
#define __LIDAR_MANAGER_H__

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <fcntl.h>
#include "PACLidarCommon.h"

// #define __DEBUG
#ifdef __DEBUG
#define DBG_INFO cout
#else
#define DBG_INFO 0 && cout
#endif
namespace PacLidar
{

class LidarLinker
{
public:
    LidarLinker() = default;
    LidarLinker(std::string ip = "192.168.1.199", uint16_t port = 5000, std::string name="Default");
    LidarLinker(LidarLinker&) = delete;
    ~LidarLinker();

public:
    /**
    * @brief 指定参数启动雷达电机以及数据通讯，且修改储存的参数
    * @param 雷达的转速以及雷达的数据类型
    * @return 成功 0 ，失败 -1
    */
    int connect2Lidar(uint32_t timeout_sec=0);

    /**
    * @brief 停止接收线程，关闭电机，关闭socket链接
    * @param Null
    * @return 成功 0 ，失败 -1
    */
    int disconnectFromLidar();

    /**
    * @brief 以储存的参数启动雷达电机以及数据通讯
    * @return 成功 0 ，失败 -1
    */
    int startupLidar();

    /** 
    * @brief 指定参数启动雷达电机以及数据通讯，且修改储存的参数
    * @param 雷达的转速以及雷达的数据类型
    * @return 成功 0 ，失败 -1
    */
    int startupLidar(PacLidar::lidarCMD speed,PacLidar::lidarCMD data_type);

    /**
    * @brief 停止接收线程，关闭电机
    * @param Null
    * @return 成功 0 ，失败 -1
    */
    int stopLidar();

    /**
    * @brief 设置参数，如果线程在运行，则向雷达发送修改参数指令，
    *       否则只修改结构体中的参数
    * @param 雷达转速、数据类型
    * @return：成功 0 ，失败 -1
    */
    int setupLidar(PacLidar::lidarCMD speed,PacLidar::lidarCMD data_type);

    /**
     * @brief 获取雷达的扫描数据
     * @param
     * 1和2 两个数组,rangs数组存储距离数据，intensities数组存储强度
     * 3和4 读取的起始光束和结束光束(包含始末两端)
     * @return 成功返回0,失败返回-1
     */
    int getLidarScanByBeam(float *ranges, float *intensities, unsigned start_beam = 0, unsigned stop_beam = PAC_MAX_BEAMS);

    /**
     * @brief 获取雷达的扫描数据
     * @param
     * 1和2 两个数组,rangs数组存储距离数据，intensities数组存储强度
     * 3和4 读取的起始角度和结束角度(包含起始角度，不包含结束角度，即:[start,stop))
     * @return 成功返回0,失败返回-1
     */
    int getLidarScanByAngle(float *ranges, float *intensities, float start_angle = 0, float stop_angle = 360);

    /** 
    * @brief 获取雷达状态，获取之前请启动雷达
    * @param state结构体
    * @return 成功返回0,失败返回-1
    */
    int getLidarState(PacLidar::lidarState_t &state);

    /**
    * @brief 作为子线程获取数据使用，请勿调用!
    */
    void capLidarData();

    enum{
        Connecting,
        Connected,
        Disconnected
    };

    /** 
    * @brief 注册雷达状态发生改变时的回调函数
    * @param 回调函数;回调函数的参数为雷达的状态
    * @return  null
    * @note 此类以设置断线重连，请勿在此函数中操作链接
    */
    void registerConnectionStateChangedCallback(void (*callback)(int));

private:
    /** 
 * @brief 直接给雷达发送命令,不论其是否在接受数据
 * @param 需要发送的命令，在Common.h中定义--lidarCMD
 * @return 如果socket句柄无效，则返回-1
 *         如果有效，则执行发送，返回值为send函数的返回值
 *         send：发送成功则返回发送出去的数据大小(byte)，失败则返回-1
 *              并且errno被设置为相应的错误代码
*/
    int send_cmd_to_lidar(uint16_t cmd);

    /** 
    * @brief 毫秒级休眠
    */
    void msleep(uint32_t msec);

    /** 
    * @brief：作为线程获取数据使用，请勿调用 
    */
    static void *dataRecvFunc(void *recver);

    /** 
    * @brief：断开重连 
    */
    int reconnect2Lidar();

    void pointsFilter(PacLidar::LidarData_t *points, size_t size);

private:
    int lidarSockFD;

    std::string lidarName;
    sockaddr_in *lidarSockAddr;

    pthread_t dataReceiver;

    PacLidar::LidarData_t sockDataPkg[PAC_NUM_OF_ONE_PKG];
    PacLidar::LidarData_t scanDataPkg[PAC_NUM_OF_ONE_SCAN];
    PacLidar::LidarData_t oneCircleData[PAC_MAX_BEAMS];
    
    PacLidar::lidarState_t lidarStatus;

    struct 
    {
        PacLidar::lidarCMD speed;
        PacLidar::lidarCMD dataType;
    } lidarParam;
    
    struct timeval rcvtimeout;

    volatile bool isCap;

    pthread_mutex_t mutex;
    pthread_mutexattr_t mutexattr;
    pthread_cond_t cond_CopyPkg;
private:
    void (*onConnectionStateChanged)(int);
};

}//namespace PacLidar

#endif //__LIDAR_MANAGER_H__