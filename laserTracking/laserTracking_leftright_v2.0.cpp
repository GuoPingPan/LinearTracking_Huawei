#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Header.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;


/**
 * 这个文件是实现雷达取中线路径的功能
 * 将左右边线进行单独显示
 *
 * @version:v2.0
 *
 * @tips: 相对于v1.0 使用roslaunch来配置初始参数
 *
 * @param sub laser_sub ["/scan",LaserScan]
 * @param pub laser_path["/laserpath",PointCloud]
 *            line_Left["/pcLeft",PointCloud]
 *            line_Right["/",PointCloud]
 *
*/

ros::Publisher laserpath;
ros::Publisher lineLeft;
ros::Publisher lineRight;
std_msgs::Header header;

float offset;
float disThres; 
float thetaX;
float thetaY;
float sizeThres;
/** 画出中间线 **/
/** bug: 当挡板宽度太大，还没进入的时候轨迹会发生偏差 **/
void pubMiddleline(vector<cv::Point2f>& xyLeft,vector<cv::Point2f>& xyRight){
    bool lbigthanr = xyLeft.size() > xyRight.size() ? 1:0;
    int ir = 0;
    int il = 0;
    sensor_msgs::PointCloud path;
    path.header = header;
    if(lbigthanr){
        while(il<xyLeft.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            il++;
        }
        path.points.resize(xyLeft.size());

    } else{
        while(il<xyLeft.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            ir++;
        }
        path.points.resize(xyRight.size());

    }
    int i=0;
    for(ir,il; il<xyLeft.size()-5&&ir<xyRight.size()-5;++ir,++il,++i){
        if(abs(xyLeft[il].x-xyRight[ir].x)>thetaX 
        && abs(xyLeft[il-1].y-xyLeft[il].y)>thetaY
        && abs(xyRight[ir-1].y-xyRight[ir].y)>thetaY)
            continue;
        path.points[i].x = (xyLeft[il].x+xyRight[ir].x)/2;
        path.points[i].y = (xyLeft[il].y+xyRight[ir].y)/2;
        path.points[i].z = 0.0;
    }

    laserpath.publish(path);
}
/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& xy){
    sensor_msgs::PointCloud point;
    point.points.resize(xy.size());
    point.header = header;
    int i=0;
    for(auto p:xy){
        if(p.y>0.1){
            point.points[i].x = p.x;
            point.points[i].y = p.y-offset;
            point.points[i].z = 0.0;
        }
        else if(p.y<-0.1){
            point.points[i].x = p.x;
            point.points[i].y = p.y+offset;
            point.points[i].z = 0.0;
        }
        else{
            continue;
        }
        ++i;
    }

    laserpath.publish(point);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> xyRight,xyLeft;
    header = scan->header;
    ROS_INFO("1");
    
    /** 将雷达点转成笛卡尔坐标系 **/
    for(int i=0;i<count;++i){
        if(scan->ranges[i]>disThres)    // 距离阈值
            continue;
        float angle = scan->angle_min+scan->angle_increment*i;
        cv::Point2f point;
        point.x = scan->ranges[i]*cos(angle);
        point.y = scan->ranges[i]*sin(angle);
        //  这里假设了车是垂直进入挡板的最佳状态
        if(point.y >0){
            xyRight.push_back(point);
        }
        else{
            xyLeft.push_back(point);
        }
    }
    
    /** 从大到小排序 **/
    sort(xyLeft.begin(),xyLeft.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
    sort(xyRight.begin(),xyRight.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
    ROS_INFO("Left laser point num: %d",xyLeft.size());
    ROS_INFO("Right laser point num: %d",xyRight.size());
    ROS_INFO("2");

    /** 投影到中线 **/
    if(xyLeft.size()>sizeThres&&xyRight.size()>sizeThres){
        pubMiddleline(xyLeft,xyRight);
    }
    else if(xyLeft.size()>sizeThres){
        pubMiddleline(xyLeft);
    }
    else if(xyRight.size()>sizeThres){
        pubMiddleline(xyRight);
    }
    else{
        return;
    }

    ROS_INFO("3");

    /** 可视化 **/
    sensor_msgs::PointCloud pcLeft;
    sensor_msgs::PointCloud pcRight;
    pcRight.header = header;
    pcLeft.header = header;
    pcLeft.points.resize(xyLeft.size());
    pcRight.points.resize(xyRight.size());

    for(int i = 0;i<xyLeft.size();++i){
        pcLeft.points[i].x = xyLeft[i].x;
        pcLeft.points[i].y = xyLeft[i].y;
        pcLeft.points[i].z = 0.0;
    }
    pcRight.points.resize(xyLeft.size());
    for(int i = 0;i<xyRight.size();++i){
        pcRight.points[i].x = xyRight[i].x;
        pcRight.points[i].y = xyRight[i].y;
        pcRight.points[i].z = 0.0;
    }

    lineLeft.publish(pcLeft);
    lineRight.publish(pcRight);
//    sensor_msgs::PointCloud pointcloud;
//    pointcloud.header = header;
//    pointcloud.points.resize(xyLeft.size()+xyRight.size());
//    pointcloud.channels.resize(2);
//    pointcloud.channels[0].name = "left";
//    pointcloud.channels[0].values.resize(xyLeft.size());
//    pointcloud.channels[1].name = "right";
//    pointcloud.channels[1].values.resize(xyRight.size());
//    int i =0;
//    for(i;i<xyRight.size();++i){
//        pointcloud.points[i].x = xyRight[i].x;
//        pointcloud.points[i].y = xyRight[i].y;
//        pointcloud.points[i].z = 0.0;
//        pointcloud.channels[0].values[i] = 47;
//        pointcloud.channels[1].values[i] = 0;
//    }
//    for(i;i<xyLeft.size();++i){
//        pointcloud.points[i].x = xyLeft[i].x;
//        pointcloud.points[i].y = xyLeft[i].y;
//        pointcloud.points[i].z = 0.0;
//        pointcloud.channels[0].values[i] = 47;
//        pointcloud.channels[1].values[i] = 0;
//
//    }
//    lineLeft.publish(pointcloud);
    ROS_INFO("4");

}

int main(int argc,char **argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh("~");
    nh.param<float>("dist_thres",disThres,0.60);
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<float>("size_thres",sizeThres,10);
    ros::Subscriber lasersub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    laserpath = nh.advertise<sensor_msgs::PointCloud>("pcRight",1);
    lineLeft = nh.advertise<sensor_msgs::PointCloud>("pcLeft",1);
//    lineRight = nh.advertise<sensor_msgs::PointCloud>("pcRight",1);
    ROS_INFO("1");
    ros::spin();
    return 0;
}
