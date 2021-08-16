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

/** *****************************************************************
 * LaserScan data detail information:
 *  Header header
 * # stamp: The acquisition time of the first ray in the scan.
 * # frame_id: The laser is assumed to spin around the positive Z axis
 * # (counterclockwise, if Z is up) with the zero angle forward along the x axis
 * 
 * float32 angle_min # start angle of the scan [rad]
 * float32 angle_max # end angle of the scan [rad]
 * float32 angle_increment # angular distance between measurements [rad]
 * 
 * float32 time_increment # time between measurements [seconds] - if your scanner
 * # is moving, this will be used in interpolating position of 3d points
 * float32 scan_time # time between scans [seconds]
 * 
 * float32 range_min # minimum range value [m]
 * float32 range_max # maximum range value [m]
 * 
 * float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
 * float32[] intensities # intensity data [device-specific units]. If your
 * # device does not provide intensities, please leave the array empty.
 *  *****************************************************************/

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
static int linkTimeout = 10;
static bool tcpQuickAck = false;

static int lidarSpeed = 10;
static int filter_lev = 3;
static int dataProportion = 1;
static int perDegLaserCnt = 16;

static string frameID       = "world";
static float rangeMin       = PAC_MIN_RANGE;
static float rangeMax       = PAC_MAX_RANGE;
static float angleMin       = -M_PI+M_PI_4;
static float angleMax       = M_PI+M_PI_4;
static float angleIncrement = DEGTORAD(PAC_ANGLE_RESOLUTION);

static int start_angle = 0;
static int scan_angle = 360;

static int intensityMax = 8160;
static int intensityFactor = 32;

LidarLinker *lm_ptr = nullptr;
pthread_t svr_thread_t;

void getAllParams(string path);
void on_sigint_recved(int signo);
void* ctrl_srv_advertise_func(void* node_handle);
void processIntensities(sensor_msgs::LaserScan& msg);
void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg);
void lidarConnectionChanged(int state);

int main(int argc, char **argv)
{
    ros::init(argc,argv,"paclidar");
    ros::NodeHandle ros_nh;
    nodeFullName = ros::this_node::getName()+"/";
    getAllParams(nodeFullName);

    signal(SIGINT,  on_sigint_recved);
    signal(SIGQUIT, on_sigint_recved);
    signal(SIGTERM, on_sigint_recved);

    LidarLinker lm(lidarIP,lidarPort,ros::this_node::getName());
    lm_ptr = &lm;
    lm.registerConnectionStateChangedCallback(&lidarConnectionChanged);

    ros::Publisher scanPub = ros_nh.advertise<sensor_msgs::LaserScan>(scanTpcName,1);
    ros::Publisher statePub = ros_nh.advertise<paclidar_driver::LidarState>(stateTpcName,1);

    if(lm.connect2Lidar(linkTimeout)!=0) 
    {
        ROS_ERROR("Failed to connect lidar,please check your network configuration.");
        ros::shutdown();
        exit(0);
    }
    if(lm.setupLidar(LidarLinker::DATA_PROPORTION,dataProportion)<0)
        ROS_ERROR("Failed to set DATA_PROPORTION as :%d",dataProportion);

    lm.setupLidar(LidarLinker::TCP_QUICK_ACK,tcpQuickAck);

    lm.setupLidar(LidarLinker::SCAN_RATE,lidarSpeed);
    lm.setupLidar(LidarLinker::FILTER_LEVEL,filter_lev);
    lm.startupLidar();
    
    sensor_msgs::LaserScan scanMsg;

    paclidar_driver::LidarState stateMsg;
    PacLidar::lidarState_t dev_state;
    
    pthread_create(&svr_thread_t,NULL,ctrl_srv_advertise_func,&ros_nh);

    ros::Duration rator(0.025);
    while (ros::ok())
    {
        double startStamp,scanTime,incTime;
        lm.getLidarScanData(scanMsg.ranges,scanMsg.intensities,startStamp,scanTime,incTime);
        scanMsg.header.stamp = ros::Time().fromSec(startStamp);
        scanMsg.scan_time = scanTime;
        scanMsg.time_increment = incTime;

        /****Angle Clipping***/
        if (scan_angle < 360) {
            auto fixStartAngle = start_angle + 45;//原始数组下标0为 -45度数据，以此矫正
            decltype(scanMsg.ranges) splitRanges(scanMsg.ranges.size(),INFINITY);
            decltype(scanMsg.intensities) splitIntens(scanMsg.intensities.size(),float(0.0));

            auto beamsCnt = scan_angle*perDegLaserCnt;            
            for (auto i = 0,idx = fixStartAngle*perDegLaserCnt; i < beamsCnt; ++i,++idx)
            {
                if(idx >= splitRanges.size())
                    idx %= splitRanges.size();
                
                splitRanges[idx] = scanMsg.ranges[idx];
                splitIntens[idx] = scanMsg.intensities[idx];
            }

            scanMsg.ranges = splitRanges;
            scanMsg.intensities = splitIntens;
        }

        processIntensities(scanMsg);
        publishLaserScanMsg(scanPub,scanMsg);

        lm.getLidarState(dev_state);
        stateMsg.firmware_version = dev_state.version;
        stateMsg.internal_temperature = dev_state.temprature;
        stateMsg.motor_speed = dev_state.speed;
        stateMsg.serial_number = dev_state.id;
        statePub.publish(stateMsg);
        
        rator.sleep();
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

void processIntensities(sensor_msgs::LaserScan& msg)
{
    for(auto i = msg.intensities.begin(); i < msg.intensities.end(); ++i){
        if(*i > intensityMax)
            *i = intensityMax;
        *i = *i / intensityFactor;
    }
}

void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg)
{
    msg.header.frame_id = frameID;

    msg.angle_min = angleMin;
    msg.angle_max = angleMax;
    msg.angle_increment = angleIncrement*dataProportion;

    msg.range_max = rangeMax;
    msg.range_min = rangeMin;

    pub.publish(msg);
}

void getAllParams(string path)
{
    bool ret = false;
    string key="pac_lidar_scan_topic";
    ret = ros::param::get(path + key, scanTpcName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),scanTpcName.c_str());
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),scanTpcName.c_str());

    key = "pac_lidar_ctrl_srv";
    ret = ros::param::get(path + key, ctrlSrvName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),ctrlSrvName.c_str());
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),ctrlSrvName.c_str());

    key = "pac_lidar_state_topic";
    ret = ros::param::get(path + key, stateTpcName);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),stateTpcName.c_str());
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),stateTpcName.c_str());

    key = "pac_lidar_ip";
    ret = ros::param::get(path + key, lidarIP);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),lidarIP.c_str());
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),lidarIP.c_str());

    key = "pac_lidar_port";
    ret = ros::param::get(path + key, lidarPort);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),lidarPort);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),lidarPort);

    key = "link_timeout_sec";
    ret = ros::param::get(path + key, linkTimeout);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),linkTimeout);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),linkTimeout);

    key = "tcp_quick_ack";
    ret = ros::param::get(path + key, tcpQuickAck);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),tcpQuickAck?"True":"False");
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),tcpQuickAck?"True":"False");

    key = "pac_lidar_speed";
    ret = ros::param::get(path + key, lidarSpeed);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),lidarSpeed);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),lidarSpeed);

    key = "pac_lidar_filter_lev";
    ret = ros::param::get(path + key, filter_lev);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),filter_lev);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),filter_lev);

    key = "pac_lidar_frame_id";
    ret = ros::param::get(path + key, frameID);
    if(ret) ROS_INFO("Got paramter %s : %s",key.c_str(),frameID.c_str());
    else ROS_WARN("Can't get the paramter, using default %s : %s",key.c_str(),frameID.c_str());

    key = "pac_lidar_range_min";
    ret = ros::param::get(path + key, rangeMin);
    if(ret) ROS_INFO("Got paramter %s : %0.2f",key.c_str(),rangeMin);
    else ROS_WARN("Can't get the paramter, using default %s : %0.2f",key.c_str(),rangeMin);

    key = "pac_lidar_range_max";
    ret = ros::param::get(path + key, rangeMax);
    if(ret) ROS_INFO("Got paramter %s : %0.2f",key.c_str(),rangeMax);
    else ROS_WARN("Can't get the paramter, using default %s : %0.2f",key.c_str(),rangeMax);

    key = "pac_lidar_intensity_max";
    ret = ros::param::get(path + key, intensityMax);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),intensityMax);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),intensityMax);

    key = "pac_lidar_intensity_factor";
    ret = ros::param::get(path + key, intensityFactor);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),intensityFactor);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),intensityFactor);

    key = "pac_angular_resolution";
    ret = ros::param::get(path + key, dataProportion);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),dataProportion);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),dataProportion);

    key = "pac_data_start_angle";
    ret = ros::param::get(path + key, start_angle);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),start_angle);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),start_angle);
    if(!(start_angle>=0 && start_angle<360)){
        ROS_ERROR("Invaild error:Start angle:%d out of range.Default:0 will be used.",start_angle);
        start_angle = 0;
    }

    key = "pac_data_scan_angle";
    ret = ros::param::get(path + key, scan_angle);
    if(ret) ROS_INFO("Got paramter %s : %d",key.c_str(),scan_angle);
    else ROS_WARN("Can't get the paramter, using default %s : %d",key.c_str(),scan_angle);
    if(!(scan_angle>=0 && scan_angle<=360)){
        ROS_ERROR("Invaild error:Scan angle:%d out of range.Default:360 will be used.",scan_angle);
        scan_angle = 360;
    };

    perDegLaserCnt = PAC_MAX_BEAMS/360/dataProportion;
}

bool on_srv_called(paclidar_driver::PACLidarCtrl::Request &req,
                paclidar_driver::PACLidarCtrl::Response &res)
{
    ROS_INFO("Received cmd to set LidarSpeed: %d Hz,FilterLevel: %d.",req.lidarSpeed,req.filterLev);
    int ret1=0,ret2 = 0;
    if(req.lidarSpeed>0){
        ret1 = lm_ptr->setupLidar(LidarLinker::SCAN_RATE,req.lidarSpeed);
        if(ret1 < 0)
            res.result="Failed to set speed.";
        else
            res.result="Set speed completed.";
    }
    if(req.filterLev>=0){
        ret2 = lm_ptr->setupLidar(LidarLinker::FILTER_LEVEL,req.filterLev);
        if(ret2<0)
            res.result+="   Failed to set filter level.";
        else
            res.result+="   Set Filter level completed.";
    }

    if(ret1<0 && ret2<0)
        return false;

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