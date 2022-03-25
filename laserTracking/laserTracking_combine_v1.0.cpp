#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

#include "std_msgs/Header.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;
/**
 * 这个文件将laser扫出来的左右边线进行融合
 *
 * @version：v1.0
 *
 *
 *
 * */
ros::Publisher laserpath;
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
    int i = 0;
    sensor_msgs::PointCloud path;
    path.points.resize((xyLeft.size() > xyRight.size() ? xyLeft.size():xyRight.size()));
    if(lbigthanr){
        while(il<xyLeft.size()&&ir<xyRight.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            il++;
        }
    } else{
        while(il<xyLeft.size()&&ir<xyRight.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            ir++;
        }
    }
    for(i,ir,il; il<xyLeft.size()-5&&ir<xyRight.size()-5;++ir,++il,++i){
        if(abs(xyLeft[il].x-xyRight[ir].x)>thetaX 
        && abs(xyLeft[il-1].y-xyLeft[il].y)>thetaY
        && abs(xyRight[ir-1].y-xyRight[ir].y)>thetaY)
            continue;
        path.points[i].x = (xyLeft[il].x+xyRight[ir].x)/2;
        path.points[i].y = (xyLeft[il].y+xyRight[ir].y)/2;
        path.points[i].z = 0.0;
    }
    path.header = header;
    laserpath.publish(path);
}
/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& xy){
    sensor_msgs::PointCloud path;
    int i=0;
    path.points.resize(xy.size());
    for(auto p:xy){
        if(p.y>0.1){
            path.points[i].x = p.x;
            path.points[i].y = p.y-offset;
            path.points[i].z = 0.0;
        }
        else if(p.y<-0.1){
            path.points[i].x = p.x;
            path.points[i].y = p.y+offset;
            path.points[i].z = 0.0;
        }
        ++i;
    }
    path.header = header;
    laserpath.publish(path);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ros::Time t1;
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> xyRight,xyLeft;
    header = scan->header;

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
    ROS_INFO("USE TIME: %f",(ros::Time::now()-t1).toSec());
}

int main(int argc,char **argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh;
    nh.param<float>("dist_thres",disThres,0.60);
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<float>("size_thres",sizeThres,10);
    ros::Subscriber lasersub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    laserpath = nh.advertise<sensor_msgs::PointCloud>("laserPath",1);
    ros::spin();
    return 0;
}
