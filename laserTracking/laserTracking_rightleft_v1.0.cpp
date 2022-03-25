#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Header.h"
#include <vector>
#include <string>
#include <algorithm>
using namespace std;


/**
 * 这个文件是实现雷达取中线路径的功能
 * 将左右边线进行单独显示
 *
 * @version:v1.0
 *
 * @param sub laser_sub ["/scan",LaserScan]
 * @param pub laser_path["/laserpath",PointCloud]
 *            line_Left["/pcLeft",PointCloud]
 *            line_Right["/",PointCloud]
*/

ros::Publisher laserpath;
ros::Publisher lineLeft;
ros::Publisher lineRight;
std_msgs::Header header;
const float offset = 0.45;

void pubMiddleline(vector<cv::Point2f>& xyLeft,vector<cv::Point2f>& xyRight){
    bool lbigthanr = xyLeft.size() > xyRight.size() ? 1:0;
    int ir = 0;
    int il = 0;
    sensor_msgs::PointCloud path;
    path.header = header;

    // 去除头部未对齐数据
    if(lbigthanr){
        while(abs(xyLeft[il].x-xyRight[ir].x)>20&&il<xyLeft.size()&&ir<xyRight.size()){
            il++;
        }
        path.points.resize(xyLeft.size());

    } else{
        while(abs(xyLeft[il].x-xyRight[ir].x)>20&&il<xyLeft.size()&&ir<xyRight.size()){
            ir++;
        }
        path.points.resize(xyRight.size());

    }
    int i=0;
    for(ir,il; il<xyLeft.size()-5&&ir<xyRight.size()-5;++ir,++il,++i){
        if(abs(xyLeft[il].x-xyRight[ir].x)>10 
        && abs(xyLeft[il-1].y-xyLeft[il].y)>10
        && abs(xyRight[ir-1].y-xyRight[ir].y)>10)
            continue;
        path.points[i].x = (xyLeft[il].x+xyRight[ir].x)/2;
        path.points[i].y = (xyLeft[il].y+xyRight[ir].y)/2;
        path.points[i].z = 0.0;
    }

    laser_path.publish(path);
}
/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& xy){
    sensor_msgs::PointCloud point;
    point.header = header;
    int i=0;
    for(auto p:xy){
        if(p.y>10){
            point.points[i].x = p.x;
            point.points[i].y = p.y-offset;
            point.points[i].z = 0.0;
        }
        else if(p.y<-10){
            point.points[i].x = p.x;
            point.points[i].y = p.y+offset;
            point.points[i].z = 0.0;
        }
        else{
            continue;
        }
        ++i;
    }

    laser_path.publish(point);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int count = scan->scan_time / scan->time_increment;
    vector<cv::Point2f> xyRight,xyLeft;
    header = scan->header;
    ROS_INFO("2");
    
    for(int i=0;i<count;++i){
        if(scan->ranges[i]>0.55)
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
    sort(xyLeft.begin(),xyLeft.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
    sort(xyRight.begin(),xyRight.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
    cout<<xyLeft<<endl;
    ROS_INFO("3");
    sensor_msgs::PointCloud pcLeft;
    pcLeft.header = header;
    sensor_msgs::PointCloud pcRight;
    pcRight.header = header;
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
    ROS_INFO("4");
    line_Left.publish(pcLeft);
    line_Right.publish(pcRight);
    if(xyLeft.size()>0&&xyRight.size()>0){
        pubMiddleline(xyLeft,xyRight);
    }
    else if(xyLeft.size()>0){
        pubMiddleline(xyLeft);
    }
    else if(xyRight.size()>0){
        pubMiddleline(xyRight);
    }
    ROS_INFO("5");

}

int main(int argc,char **argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    laser_path = nh.advertise<sensor_msgs::PointCloud>("laserpath",1);
    line_Left = nh.advertise<sensor_msgs::PointCloud>("pcLeft",1);
    line_Right = nh.advertise<sensor_msgs::PointCloud>("pcRight",1);
    ROS_INFO("1");
    ros::spin();
    return 0;
}
