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

class lidarManager
{
public:
    lidarManager(std::string ip = "192.168.1.199", uint16_t port = 5000);
    ~lidarManager();

    int connect2Lidar(uint32_t timeout_sec=0);

    int disconnectFromLidar();

    /*
    * @desc:以储存的参数启动雷达电机以及数据通讯
    * @return：成功 0 ，失败 -1
    */
    int startupLidar();

    /*
    * @desc:指定参数启动雷达电机以及数据通讯，且修改储存的参数
    * @params：雷达的转速以及雷达的数据类型
    * @return：成功 0 ，失败 -1
    */
    int startupLidar(PacLidar::lidarCMD speed,PacLidar::lidarCMD data_type);

    int stopLidar();

    int setupLidar(PacLidar::lidarCMD speed,PacLidar::lidarCMD data_type);

    /* 
     * @desc:获取雷达的扫描数据
     * @params:
     * 1和2 两个数组,rangs数组存储距离数据，intensities数组存储强度
     * 3和4 读取的起始光束和结束光束(包含始末两端)
     * @return:成功返回0,失败返回-1
     */
    int getLidarScanByBeam(float *ranges, float *intensities, unsigned start_beam = 0, unsigned stop_beam = PAC_MAX_BEAMS);

    /* 
     * @desc:获取雷达的扫描数据
     * @params:
     * 1和2 两个数组,rangs数组存储距离数据，intensities数组存储强度
     * 3和4 读取的起始角度和结束角度(包含起始角度，不包含结束角度，即:[start,stop))
     * @return:成功返回0,失败返回-1
     */
    int getLidarScanByAngle(float *ranges, float *intensities, float start_angle = 0, float stop_angle = 360);

    /* 
    * @desc：作为线程获取数据使用，请勿调用 
    */
    void capLidarData();

private:
    /* 
 * @desc:直接给雷达发送命令,不论其是否在接受数据
 * @param:需要发送的命令，在Common.h中定义--lidarCMD
 * @return:如果socket句柄无效，则返回-1
 *         如果有效，则执行发送，返回值为send函数的返回值
 *         send：发送成功则返回发送出去的数据大小(byte)，失败则返回-1
 *              并且errno被设置为相应的错误代码
*/
    int send_cmd_to_lidar(uint16_t cmd);

    /* 
    * 毫秒级休眠
    */
    void msleep(uint32_t msec);

    /* 
     *@desc：作为线程获取数据使用，请勿调用 
    */
    static void *dataRecvFunc(void *recver);

    int reconnect2Lidar();

private:
    int lidarSockFD;

    uint16_t lidarcmd;

    sockaddr_in *lidarSockAddr;

    pthread_t dataReceiver;

    PacLidar::LidarData_t sockDataPkg[PAC_NUM_OF_ONE_PKG];
    PacLidar::LidarData_t scanDataPkg[PAC_NUM_OF_ONE_SCAN];
    PacLidar::LidarData_t oneCircleData[PAC_MAX_BEAMS];
    
    struct
    {
        uint32_t version    = 0;
        uint32_t id         = 0;
        int      temprature = 0;
        uint16_t speed      = 0;
    } lidarStatus;

    struct 
    {
        PacLidar::lidarCMD speed;
        PacLidar::lidarCMD dataType;
    } lidarParam;
    
    struct timeval rcvtimeout;

    volatile bool isCap;

    pthread_mutex_t mutex;
    pthread_mutexattr_t mutexattr;
};

#endif //__LIDAR_MANAGER_H__