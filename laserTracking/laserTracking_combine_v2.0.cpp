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
 * @version：v2.0
 *
 * @tips:相对于v1.0引入了 void pubMiddlelineCombine
 *
 *
 * */



ros::Publisher laserpath;
std_msgs::Header header;

float offset;
float disThres; 
float thetaX;
float thetaY;
int sizeThres;
int sizeSum;

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

/** 方法四：混合处理不分单双边线 **/
void pubMiddlelineCombine(vector<cv::Point2f>& xyLeft,vector<cv::Point2f>& xyRight){

    // 1.参数定义
    int r = 0;
    int l = 0;
    float max_distance = 0;
    sensor_msgs::PointCloud pc;
    pc.header = header;
    float lastparallel = 0; // 记录上一点的横向坐标
    bool init = 0;

    // 计算中线，只要有一边线的点都用完了就退出循环
    while (l<xyLeft.size() && r<xyRight.size())
    {
        // ROS_INFO("l: %d, r:%d",l,r);
        // 1.初始化
        if(!init){
            lastparallel =( xyLeft[l].y + xyRight[r].y )/2;
            init = 1;
            continue;
        }
        // 2.两边线纵向距离相等时
        if(xyLeft[l].x == xyRight[r].x){
            if(xyLeft[l].x > max_distance)
                max_distance = xyLeft[l].x;
            float tmp = ( xyLeft[l].y + xyRight[r].y )/2;
            if(abs(tmp - lastparallel)>0.1){// 与上一点横坐标的偏差
                l++;
                r++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyLeft[l].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            l++;
            r++;
            lastparallel = tmp;
        }
        // 3.左边线纵向距离大于右边线
        else if(xyLeft[l].x > xyRight[r].x){
            if(xyLeft[l].x > max_distance)
                max_distance = xyLeft[l].x;
            float tmp = xyLeft[l].y - offset;
            // cout<<"LEFT:"<<tmp<<endl;

            if(abs(tmp - lastparallel)>0.1){
                l++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyLeft[l].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            l++;
            lastparallel = tmp;
        }
        // 4.右边线纵向距离大于左边线
        else{
            if(xyRight[l].x > max_distance)
                max_distance = xyRight[l].x;
            // cout<<"RIGHT:"<<xyRight[r].y<<endl;
            float tmp = xyRight[r].y + offset;
            // cout<<"RIGHT:"<<tmp<<endl;
            if(abs(tmp - lastparallel)>0.1){
                r++;
                continue;
            }
            geometry_msgs::Point32 p;
            p.x = xyRight[r].x;
            p.y = tmp;
            p.z = 0.0;
            pc.points.push_back(p);
            r++;
            lastparallel = tmp;
        }

    }
    // ROS_INFO("OUT1");
    // 将另一边剩下的轨迹点加入
    // while (l<xyLeft.size())
    // {
    //     if(xyLeft[l].x > max_distance)
    //         max_distance = xyLeft[l].x;
    //     float tmp = xyLeft[l].y - offset;
        
    //     geometry_msgs::Point32 p;
    //     p.x = xyLeft[l].x;
    //     p.y = tmp;
    //     p.z = 0.0;
    //     pc.points.push_back(p);
    //     l++;
    // }
    
    // while (r<xyRight.size())
    // {
    //     if(xyRight[r].x > max_distance)
    //         max_distance = xyRight[r].x;
    //     // cout<<"RIGHT BEFORT:"<<xyRight[r].y<<endl;
    //     float tmp = xyRight[r].y + offset;
    //     // cout<<"RIGHT AFTER:"<<tmp<<endl;
    //     geometry_msgs::Point32 p;
    //     p.x = xyRight[r].x;
    //     p.y = tmp;
    //     p.z = 0.0;
    //     pc.points.push_back(p);
    //     r++;
    // }
    // ROS_INFO("OUT2");
    ROS_INFO("MAX DISTANCE %f",max_distance);

    laserpath.publish(pc);
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

    float x1(0),y1(0),x2(0),y2(0);
    ROS_INFO("X1 %f",x1);
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
            x1+=point.x;
            y1+=point.y;
            xyLeft.push_back(point);
        }
        else{
            x2+=point.x;
            y2+=point.y;
            xyRight.push_back(point);
        }
    }
    x1 /= xyLeft.size();
    y1 /= xyLeft.size();
    x2 /= xyRight.size();
    y2 /= xyRight.size();
    
    // 看情况：当只有一条边线的时候大致是什么情况
    //        如果有两条边线的时候该怎么判断是机器人还是挡板：通过算左右线均值坐标的距离来判断
    // 当接收到黄灯信息时发布 scan1 用于倒车定位

    if(xyRight.size()+xyLeft.size()<sizeSum){
        ROS_INFO("There is not obstacles.");
        return;
    }
    /** 投影到中线 **/
    if(xyLeft.size()>sizeThres&&xyRight.size()>sizeThres){
        /** 从大到小排序 **/
        sort(xyLeft.begin(),xyLeft.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
        sort(xyRight.begin(),xyRight.end(),[](const cv::Point2f p1,const cv::Point2f p2){return p1.x>p2.x;});
        ROS_INFO("Left laser point num: %d",xyLeft.size());
        ROS_INFO("Right laser point num: %d",xyRight.size());
        pubMiddlelineCombine(xyLeft,xyRight);
    }
    else if(xyLeft.size()>sizeThres){
        pubMiddleline(xyLeft);
    }
    else if(xyRight.size()>sizeThres){
        pubMiddleline(xyRight);
    }
    ROS_INFO("USE TIME: %f",(ros::Time::now()-t1).toSec());
}

/*
雷达的任务是：1.完成轨迹计算    采用两边线障碍点来进行处理
            2.完成机器人避障    
            3.完成倒车定位
*/

int main(int argc,char **argv){

    ros::init(argc,argv,"laserTracking");
    ros::NodeHandle nh;
    nh.param<float>("dist_thres",disThres,0.60);
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<int>("size_thres",sizeThres,10);
    nh.param<int>("size_sum",sizeSum,10);
    ros::Subscriber lasersub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,laserCallback);
    laserpath = nh.advertise<sensor_msgs::PointCloud>("laserPath",1);
    ros::spin();
    return 0;
}
