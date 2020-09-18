/** ***************************************************************** 
* 版权所有：北京光勺科技有限公司
* 文件名：rosDriver.cpp
* 文件功能描述：PAC雷达的ROS驱动程序
* 作者：Roser
* 维护者：Roser
* Email：roserxy@163.com
* 时间：2020-6
* 创建标识：Init 
* *******************************************************************/

#include <iostream>
#include <sys/signal.h>
#include "lidarlinker.h"
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <paclidar_driver/PACLidarCtrl.h>
#include <paclidar_driver/LidarState.h>
#include <vector>

using namespace std;
using namespace PacLidar;

static string nodeFullName = "pac_lidar_node";
static string scanTpcName  = "scan";
static string stateTpcName = "pac_lidar_state";
static string ctrlSrvName  = "pac_lidar_ctrl";

static string lidarIP = "192.168.1.199";
static int lidarPort  = 5000;
static int lidarSpeed = 10;
static bool checkData = true;
static bool tearOptim = false;
static int dataProportion = 1;

static string frameID       = "world";
static float rangeMin       = PAC_MIN_RANGE;
static float rangeMax       = PAC_MAX_RANGE;
static float angleMin       = 0;
static float angleMax       = -M_PI+DEGTORAD(PAC_ANGLE_RESOLUTION);
static float angleIncrement = DEGTORAD(PAC_ANGLE_RESOLUTION);

LidarLinker *lm_ptr = nullptr;
pthread_t svr_thread_t;

void getAllParams(string path);
void on_sigint_recved(int signo);
void* ctrl_srv_advertise_func(void* node_handle);
void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg,double scanTm,std::vector<float>& ranges,std::vector<float>& intens);
void lidarConnectionChanged(int state);

int main(int argc, char **argv)
{

    signal(SIGINT,  on_sigint_recved);
    signal(SIGQUIT, on_sigint_recved);
    signal(SIGTERM, on_sigint_recved);

    ros::init(argc,argv,"paclidar");
    ros::NodeHandle ros_nh;
    nodeFullName = ros::this_node::getName()+"/";
    getAllParams(nodeFullName);

    LidarLinker lm(lidarIP,lidarPort,ros::this_node::getName());
    lm_ptr = &lm;
    lm.registerConnectionStateChangedCallback(&lidarConnectionChanged);

    ros::Publisher scanPub = ros_nh.advertise<sensor_msgs::LaserScan>(scanTpcName,1);
    ros::Publisher statePub = ros_nh.advertise<paclidar_driver::LidarState>(stateTpcName,1);

    PacLidar::lidarCMD dtType = checkData?PacLidar::SET_DATA_CHECKED:PacLidar::SET_DATA_ORIGINAL;
    PacLidar::lidarCMD spd;
    switch (lidarSpeed)
    {
    case 10:
        spd = PacLidar::SET_SPEED_HZ_10;
        break;
    case 15:
        spd = PacLidar::SET_SPEED_HZ_15;
        break;
    case 20:
        spd = PacLidar::SET_SPEED_HZ_20;
        break;
    default:
        spd = PacLidar::SET_SPEED_HZ_10;
        break;
    }

    lm.setTearOptimize(tearOptim);

    lm.connect2Lidar();
    lm.startupLidar(spd,dtType);
    std::vector<float> scanRans;
    std::vector<float> scanIntes;
    sensor_msgs::LaserScan scanMsg;

    paclidar_driver::LidarState stateMsg;
    PacLidar::lidarState_t dev_state;
    
    pthread_create(&svr_thread_t,NULL,ctrl_srv_advertise_func,&ros_nh);

    while (ros::ok())
    {
        double startTM = ros::Time::now().toSec();
        // lm.getLidarScanByAngle(scanRans, scanIntes,0,360);
        lm.getLidarScanData(scanRans,scanIntes);
        ROS_INFO("Size:%d",scanRans.size());
        publishLaserScanMsg(scanPub,scanMsg,ros::Time::now().toSec()-startTM,scanRans,scanIntes);
        if(scanMsg.scan_time>0.5) ROS_ERROR("Scan time over than 0.5s : %f",scanMsg.scan_time);

        lm.getLidarState(dev_state);
        stateMsg.firmware_version = dev_state.version;
        stateMsg.internal_temperature = dev_state.temprature;
        stateMsg.motor_speed = dev_state.speed;
        stateMsg.serial_number = dev_state.id;
        statePub.publish(stateMsg);

        // bzero(scanRans,PAC_MAX_BEAMS);
        // bzero(scanIntes,PAC_MAX_BEAMS);
        scanRans.clear();
        scanIntes.clear();
    }
    pthread_join(svr_thread_t,NULL);
    return 0;
}

void on_sigint_recved(int signo)
{
    ROS_INFO("Received signal:%s", strsignal(signo));
    ros::shutdown();
    if(lm_ptr!=nullptr) lm_ptr->disconnectFromLidar();
    exit(0);
}

void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg,double scanTm,std::vector<float>& ranges,std::vector<float>& intens)
{
    msg.header.frame_id = frameID;
    msg.header.stamp = ros::Time::now();

    msg.angle_min = angleMin;
    msg.angle_max = angleMax;
    msg.angle_increment = angleIncrement;

    msg.range_max = rangeMax;
    msg.range_min = rangeMin;

    msg.ranges = ranges;
    msg.intensities = intens;

    msg.scan_time = scanTm;
    msg.time_increment = msg.scan_time / (PAC_MAX_BEAMS - 1);
    pub.publish(msg);
}

void getAllParams(string path)
{
    bool ret = false;
    string key="ScanTopic";
    ret = ros::param::get(path + key, scanTpcName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),scanTpcName.c_str());
    else ROS_INFO("Can't get the paramter, using default %s : %s",key.c_str(),scanTpcName.c_str());

    key = "CtrlSrv";
    ret = ros::param::get(path + key, ctrlSrvName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),ctrlSrvName.c_str());
    else ROS_INFO("Can't get the paramter, using default %s : %s",key.c_str(),ctrlSrvName.c_str());

    key = "StateTopic";
    ret = ros::param::get(path + key, stateTpcName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),stateTpcName.c_str());
    else ROS_INFO("Can't get the paramter, using default %s : %s",key.c_str(),stateTpcName.c_str());

    key = "IP";
    ret = ros::param::get(path + key, lidarIP);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),lidarIP.c_str());
    else ROS_INFO("Can't get the paramter, using default %s : %s",key.c_str(),lidarIP.c_str());

    key = "Port";
    ret = ros::param::get(path + key, lidarPort);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),lidarPort);
    else ROS_INFO("Can't get the paramter, using default %s : %d",key.c_str(),lidarPort);

    key = "Speed";
    ret = ros::param::get(path + key, lidarSpeed);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),lidarSpeed);
    else ROS_INFO("Can't get the paramter, using default %s : %d",key.c_str(),lidarSpeed);

    key = "DataCheck";
    ret = ros::param::get(path + key, checkData);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),checkData);
    else ROS_INFO("Can't get the paramter, using default %s : %d",key.c_str(),checkData);

    key = "FrameID";
    ret = ros::param::get(path + key, frameID);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),frameID.c_str());
    else ROS_INFO("Can't get the paramter, using default %s : %s",key.c_str(),frameID.c_str());

    key = "RangeMin";
    ret = ros::param::get(path + key, rangeMin);
    if(ret) ROS_INFO("Got paramter %s : %0.2f",key.c_str(),rangeMin);
    else ROS_INFO("Can't get the paramter, using default %s : %0.2f",key.c_str(),rangeMin);

    key = "RangeMax";
    ret = ros::param::get(path + key, rangeMax);
    if(ret) ROS_INFO("Got paramter %s : %0.2f",key.c_str(),rangeMax);
    else ROS_INFO("Can't get the paramter, using default %s : %0.2f",key.c_str(),rangeMax);

    key = "TearOptimize";
    ret = ros::param::get(path + key, tearOptim);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),tearOptim);
    else ROS_INFO("Can't get the paramter, using default %s : %d",key.c_str(),tearOptim);
}

bool on_srv_called(paclidar_driver::PACLidarCtrl::Request &req,
                paclidar_driver::PACLidarCtrl::Response &res)
{
    PacLidar::lidarCMD dtType = req.dataCheck?PacLidar::SET_DATA_CHECKED:PacLidar::SET_DATA_ORIGINAL;
    PacLidar::lidarCMD spd;
    switch (req.lidarSpeed)
    {
    case 10:
        spd = PacLidar::SET_SPEED_HZ_10;
        break;
    case 15:
        spd = PacLidar::SET_SPEED_HZ_15;
        break;
    case 20:
        spd = PacLidar::SET_SPEED_HZ_20;
        break;
    default:
        req.lidarSpeed = 10;
        spd = PacLidar::SET_SPEED_HZ_10;
        break;
    }
    ROS_INFO("Received cmd to set LidarSpeed: %d Hz,DataChecked: %d.",req.lidarSpeed,req.dataCheck);
    res.result = lm_ptr->setupLidar(spd,dtType);
    return true;
}            

void* ctrl_srv_advertise_func(void* node_handle)
{
    ros::NodeHandle *nh = (ros::NodeHandle *)node_handle;
    ros::ServiceServer ctrlSvr = nh->advertiseService(ctrlSrvName,on_srv_called);
    ros::spin();
    ctrlSvr.shutdown();
}

void lidarConnectionChanged(int state)
{
    switch (state)
    {
    case LidarLinker::Connected:
        ROS_INFO("Lidar Connection is ready now.");
        break;
    case LidarLinker::Connecting:
        ROS_INFO("Trying to connect to lidar...");
        break;
    case LidarLinker::Disconnected:
        ROS_ERROR("Lidar Connetion is unavalible.");
        break;
    default:
        ROS_INFO("Lidar Connetion is unknown.");
        break;
    }
}