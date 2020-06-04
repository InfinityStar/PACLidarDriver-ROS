#include <iostream>
#include <sys/signal.h>
#include "lidarManager.h"
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


void initLaserScanMsg(sensor_msgs::LaserScan& msg);
void on_sigint_recved(int signo);


int main(int argc, char **argv)
{
    signal(SIGINT, on_sigint_recved);
    signal(SIGQUIT, on_sigint_recved);

    ros::init(argc,argv,"paclidar");
    ros::NodeHandle ros_nh;
    ros::Publisher scanPub = ros_nh.advertise<sensor_msgs::LaserScan>("scan",1);
    // ros::Rate rt(10);

    lidarManager lm = lidarManager();
    lm.connect2Lidar();
    lm.startupLidar(PacLidar::SET_SPEED_HZ_10,PacLidar::SET_DATA_CHECKED);
    float scanRans[PAC_MAX_BEAMS];
    float scanIntes[PAC_MAX_BEAMS];

    sensor_msgs::LaserScan scanData;
    initLaserScanMsg(scanData);

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
    msg.header.frame_id = "world";

    msg.angle_min = 0;
    msg.angle_max = -M_PI+DEGTORAD(PAC_ANGLE_RESOLUTION);
    msg.angle_increment = DEGTORAD(PAC_ANGLE_RESOLUTION);

    msg.range_max = PAC_MAX_RANGE;
    msg.range_min = PAC_MIN_RANGE;
}