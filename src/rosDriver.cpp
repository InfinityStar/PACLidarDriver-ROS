#include <iostream>
#include <sys/signal.h>
#include "lidarManager.h"
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <paclidar_driver/PACLidarCtrl.h>
#include <paclidar_driver/LidarState.h>

using namespace std;

static string nodeFullName = "pac_lidar_node";
static string scanTpcName  = "scan";
static string stateTpcName = "pac_lidar_state";
static string ctrlSrvName  = "pac_lidar_ctrl";

static string lidarIP = "192.168.1.199";
static int lidarPort  = 5000;
static int lidarSpeed = 10;
static bool checkData = true;

static string frameID       = "world";
static float rangeMin       = PAC_MIN_RANGE;
static float rangeMax       = PAC_MAX_RANGE;
static float angleMin       = 0;
static float angleMax       = -M_PI+DEGTORAD(PAC_ANGLE_RESOLUTION);
static float angleIncrement = DEGTORAD(PAC_ANGLE_RESOLUTION);

lidarManager *lm_ptr;
pthread_t svr_thread_t;

void getAllParams(string path);
void on_sigint_recved(int signo);
void* ctrl_srv_advertise_func(void* node_handle);
void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg,double scanTm,float* ranges,float* intens);

int main(int argc, char **argv)
{
    signal(SIGINT,  on_sigint_recved);
    signal(SIGQUIT, on_sigint_recved);

    ros::init(argc,argv,"paclidar");
    ros::NodeHandle ros_nh;
    nodeFullName = ros::this_node::getName()+"/";
    getAllParams(nodeFullName);

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

    lidarManager lm(lidarIP,lidarPort);
    lm_ptr = &lm;
    lm.connect2Lidar();
    lm.startupLidar(spd,dtType);
    float scanRans[PAC_MAX_BEAMS];
    float scanIntes[PAC_MAX_BEAMS];
    sensor_msgs::LaserScan scanMsg;

    paclidar_driver::LidarState stateMsg;
    PacLidar::lidarState_t dev_state;
    
    pthread_create(&svr_thread_t,NULL,ctrl_srv_advertise_func,&ros_nh);

    // ros::Rate rt(10);
    while (ros::ok())
    {
        double startTM = ros::Time::now().toSec();
        lm.getLidarScanByAngle(scanRans, scanIntes,0,360);
        publishLaserScanMsg(scanPub,scanMsg,ros::Time::now().toSec()-startTM,scanRans,scanIntes);

        if(scanMsg.scan_time>0.15)
            ROS_INFO("Scan Spent %f.",scanMsg.scan_time);

        lm.getLidarState(dev_state);
        stateMsg.firmware_version = dev_state.version;
        stateMsg.internal_temperature = dev_state.temprature;
        stateMsg.motor_speed = dev_state.speed;
        stateMsg.serial_number = dev_state.id;
        statePub.publish(stateMsg);

        bzero(scanRans,PAC_MAX_BEAMS);
        bzero(scanIntes,PAC_MAX_BEAMS);
        // ROS_INFO("Published.Spent %f.",ros::Time::now().toSec()-startTM);
        // rt.sleep();
    }
    pthread_join(svr_thread_t,NULL);
    return 0;
}

void on_sigint_recved(int signo)
{
    std::cout << "Received signal:" << strsignal(signo) << std::endl;
    ros::shutdown();
    exit(0);
}

void publishLaserScanMsg(ros::Publisher &pub,sensor_msgs::LaserScan& msg,double scanTm,float* ranges,float* intens)
{
    msg.header.frame_id = frameID;
    // msg.header.stamp = ros::Time::now();

    msg.angle_min = angleMin;
    msg.angle_max = angleMax;
    msg.angle_increment = angleIncrement;

    msg.range_max = rangeMax;
    msg.range_min = rangeMin;

    msg.ranges.assign(ranges,ranges+PAC_MAX_BEAMS);
    msg.intensities.assign(intens, intens + PAC_MAX_BEAMS);

    msg.scan_time = scanTm;
    msg.time_increment = msg.scan_time / (PAC_MAX_BEAMS - 1);
    pub.publish(msg);
}

void getAllParams(string path)
{
    string key="ScanTopic";
    ros::param::get(path + key, scanTpcName);

    key = "CtrlSrv";
    ros::param::get(path + key, ctrlSrvName);

    key = "StateTopic";
    ros::param::get(path + key, stateTpcName);

    key = "IP";
    ros::param::get(path + key, lidarIP);

    key = "Port";
    ros::param::get(path + key, lidarPort);

    key = "Speed";
    ros::param::get(path + key, lidarSpeed);

    key = "DataCheck";
    ros::param::get(path + key, checkData);

    key = "FrameID";
    ros::param::get(path + key, frameID);

    key = "RangeMin";
    ros::param::get(path + key, rangeMin);

    key = "RangeMax";
    ros::param::get(path + key, rangeMax);
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
    return (!lm_ptr->setupLidar(spd,dtType));
}            

void* ctrl_srv_advertise_func(void* node_handle)
{
    ros::NodeHandle *nh = (ros::NodeHandle *)node_handle;
    ros::ServiceServer ctrlSvr = nh->advertiseService(ctrlSrvName,on_srv_called);
    ros::spin();
    ctrlSvr.shutdown();
}