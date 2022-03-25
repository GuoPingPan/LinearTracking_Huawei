#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <vector>


/**
 * 这个算法主要是实现雷达数据的过滤,并根据周围0.6米范围内的雷达点达到一定阈值后发出停止的信号
 *
 *
 * @param pub stop_pub["/stop",std_msgs::Int32]
 *            filter_pub["/filter_laser",sensor_msgs::PointCloud]
 * @param sub laser_sub["/scan",sensor_msgs::LaserScan] 
 * @param distThres 0.6 default
 * @param sizeThres 10 default
 * 
*/

using namespace std;

float distThres;
int sizeThres;
ros::Publisher stop_pub;
ros::Publisher filter_pub;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ros::Time t1;
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> obstacle;
    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "laser";
    pc.header.stamp = ros::Time::now();

    for(int i = 0;i<count;++i){
        if(scan->ranges[i]>distThres)    // 距离阈值
            continue;
        float angle = scan->angle_min+scan->angle_increment*i;
        cv::Point2f point;
        point.x = scan->ranges[i]*cos(angle);
        point.y = scan->ranges[i]*sin(angle);
        obstacle.push_back(point);

        geometry_msgs::Point32 point3d;
        point3d.x = point.x;
        point3d.y = point.y;
        point3d.z = 0.0;

        pc.points.push_back(point3d);
    }

    pc.points.resize(int(obstacle.size()));
    filter_pub.publish(pc);

    if(int(obstacle.size())>sizeThres){
        std_msgs::Int32 stop;
        stop.data = 1;
        stop_pub.publish(stop);
    }
}

int main(int argc,char**argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh("~");
    nh.param<float>("dist_thres",distThres,0.60);
    nh.param<int>("size_thres",sizeThres,10);

    ROS_INFO("dist_thres: %f",distThres);
    ROS_INFO("size_thres: %d",sizeThres);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    stop_pub = nh.advertise<std_msgs::Int32>("stop",1);
    filter_pub = nh.advertise<sensor_msgs::PointCloud>("filter_laser",1);

    ros::spin();

    return 0;
}