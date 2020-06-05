#include <iostream>
#include <sys/signal.h>
#include "lidarManager.h"
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <paclidar_driver/PACLidarCtrl.h>

using namespace std;

static string nodeFullName = "pac_lidar_node";
static string scanTpcName  = "scan";
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
void initLaserScanMsg(sensor_msgs::LaserScan& msg);
void on_sigint_recved(int signo);
void* ctrl_srv_advertise_func(void* node_handle);


int main(int argc, char **argv)
{
    signal(SIGINT,  on_sigint_recved);
    signal(SIGQUIT, on_sigint_recved);

    ros::init(argc,argv,"paclidar");
    ros::NodeHandle ros_nh;
    nodeFullName = ros::this_node::getName()+"/";
    getAllParams(nodeFullName);

    ros::Publisher scanPub = ros_nh.advertise<sensor_msgs::LaserScan>(scanTpcName,1);

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

    lidarManager lm = lidarManager(lidarIP,lidarPort);
    lm_ptr = &lm;
    lm.connect2Lidar();
    lm.startupLidar(spd,dtType);
    float scanRans[PAC_MAX_BEAMS];
    float scanIntes[PAC_MAX_BEAMS];

    sensor_msgs::LaserScan scanData;
    initLaserScanMsg(scanData);
    
    pthread_create(&svr_thread_t,NULL,ctrl_srv_advertise_func,&ros_nh);

    while (ros::ok())
    {
        double startTM = ros::Time::now().toSec();
        lm.getLidarScanByAngle(scanRans, scanIntes,0,360);

        scanData.header.stamp = ros::Time::now();
        double endTM = scanData.header.stamp.toSec();
        
        scanData.ranges.assign(scanRans,scanRans+PAC_MAX_BEAMS);
        scanData.intensities.assign(scanIntes,scanIntes+PAC_MAX_BEAMS);

        scanData.scan_time = endTM - startTM;
        scanData.time_increment = scanData.scan_time / (PAC_MAX_BEAMS-1);
        
        scanPub.publish(scanData);
        // ROS_INFO("Published.Spent %f.",scanData.scan_time);
        bzero(scanRans,PAC_MAX_BEAMS);
        bzero(scanIntes,PAC_MAX_BEAMS);
    }
    return 0;
}

void on_sigint_recved(int signo)
{
    std::cout << "Received signal:" << strsignal(signo) << std::endl;
    ros::shutdown();
    exit(0);
}

void initLaserScanMsg(sensor_msgs::LaserScan& msg)
{
    msg.header.frame_id = frameID;

    msg.angle_min = angleMin;
    msg.angle_max = angleMax;
    msg.angle_increment = angleIncrement;

    msg.range_max = rangeMax;
    msg.range_min = rangeMin;
}

void getAllParams(string path)
{
    string key="ScanTopic";
    assert(ros::param::get(path+key,scanTpcName)==true);

    key = "CtrlSrv";
    assert(ros::param::get(path + key, ctrlSrvName) == true);

    key = "IP";
    assert(ros::param::get(path + key, lidarIP) == true);

    key = "Port";
    assert(ros::param::get(path + key, lidarPort) == true);

    key = "Speed";
    ros::param::get(path + key, lidarSpeed);

    key = "DataCheck";
    ros::param::get(path + key, checkData);

    key = "FrameID";
    assert(ros::param::get(path + key, frameID) == true);

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
        spd = PacLidar::SET_SPEED_HZ_10;
        break;
    }
    lm_ptr->setupLidar(spd,dtType);
    return true;
}            

void* ctrl_srv_advertise_func(void* node_handle)
{
    ros::NodeHandle *nh = (ros::NodeHandle *)node_handle;
    ros::ServiceServer ctrlSvr = nh->advertiseService(ctrlSrvName,on_srv_called);
    while(ros::ok())
    {
        ros::spin();
    }
    ctrlSvr.shutdown();
}