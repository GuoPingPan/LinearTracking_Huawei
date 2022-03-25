#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <chrono>

#include <vector>
using namespace std;
/**
 *  这个是nav_msgs::Path版本的,用的是最原始的策略:OpenCV从中间向两边进行扫描
 *
 *  @author:panguoping
 *  @version:v2.0
 *
 *  @functions:
 *      - transTo3D
 *      - pubMiddleline
 *      - pubMiddleline
 *
 * */

/** 参数定义 **/
const float fx = 760.4862674784594; // 相机内参
const float fy = 761.4971958529285;
const float cx = 631.6715834996345;
const float cy = 329.3054436037627;

const float y = 129.5585;   // 固定相机离地面高度，单位mm，通过计算得到，不一定准

float offset;
float thetaX;
float thetaY;
float scale;
int imgWidth;  // 图片大小
int imgHeight;

ros::Publisher pathpub;

/** 将图片中的 点uv 转化为 现实坐标中的 点xy **/
cv::Point3f getXYZ(cv::Point2f& point){
    cv::Point3f point3d;
    point3d.x = (fy * y/(point.y-cy))/scale;
    point3d.y = (-(point.x-cx)*point3d.x/fx); //注意这里千万不能再除一次scale,因为point3d.x是已经经过放缩后的了
    point3d.z = 0.0;
    return point3d;
}

/** 将二维边线转化为三维边线 **/
void transTo3D(vector<cv::Point2f>& uv,vector<cv::Point3f>& posearray){
    for(auto p:uv){
        cv::Point3f pose = getXYZ(p);
        posearray.push_back(pose);
    }
}

/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& uvLeft,vector<cv::Point2f>& uvRight){
    vector<cv::Point3f> xyLeft,xyRight;
    nav_msgs::Path path;
    transTo3D(uvLeft,xyLeft);
    transTo3D(uvRight,xyRight);
    ROS_INFO("int two middle");
    bool lbigthanr = xyLeft.size() > xyRight.size() ? true:false;
    int ir = 0;
    int il = 0;

    // 去除头部未对齐的点
    if(lbigthanr){
        while(il<xyLeft.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            ++il;
        }
    } else{
        while(ir<xyRight.size()&&abs(xyLeft[il].x-uvRight[ir].x)>thetaX){
            ++ir;
        }
    }
    ROS_INFO("sizer:%d,sizel:%d",xyRight.size(),xyLeft.size());
    ROS_INFO("ir:%d,il:%d",ir,il);
    ROS_INFO("sizer1:%d,sizel1:%d",xyRight.size()-5,xyLeft.size()-5);
    ROS_INFO("ir1:%d,il1:%d",ir,il);

    for(ir,il; il<int(xyLeft.size()-5)&&ir<int(uvRight.size()-5);++ir,++il){    //这里xyLeft.size()需要进行int强制转换,不然无法正确进行循环
        ROS_INFO("ir:%d,il:%d",ir,il);
        if(abs(xyLeft[il].x-uvRight[ir].x)>thetaX
           && abs(xyLeft[il+1].y-xyLeft[il].y)>thetaY
           && abs(xyRight[ir+1].y-xyRight[ir].y)>thetaY)
            continue;
        ROS_INFO("yes");

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = (xyLeft[il].x+xyRight[ir].x)/2;
        pose.pose.position.y = (xyLeft[il].y+xyRight[ir].y)/2;
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }
    ROS_INFO("pathpose:%d",path.poses.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";

    pathpub.publish(path);
    ROS_INFO("publish successfully");
}
/** 画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& uv){
    vector<cv::Point3f> xy;
    nav_msgs::Path path;
    transTo3D(uv,xy);
    for(auto p:xy){
        if(p.y>0.1){
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y-offset;
            pose.pose.position.z = 0.0;
            path.poses.push_back(pose);
        }
        else if(p.y<-0.1){
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y+offset;
            pose.pose.position.z = 0.0;
            path.poses.push_back(pose);
        }
        else{
            continue;
        }
    }
    ROS_INFO("pathpose:%d",path.poses.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";
    pathpub.publish(path);
    ROS_INFO("publish successfully");
}

int main(int argc ,char **argv){
    ros::init(argc,argv,"linktracking");
    ros::NodeHandle nh("~");
    ros::Time t1 = ros::Time::now();

    pathpub = nh.advertise<nav_msgs::Path>("path",0);
    string video;
    nh.param<string>("video",video,"../dataset/test.mp4");
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<float>("scale",scale,1000.0);
    nh.param<int>("imgWidth",imgWidth,1280);
    nh.param<int>("imgHeight",imgHeight,720);
    while (ros::ok())
    {
        /** 1.get the video **/
        cv::VideoCapture cap = cv::VideoCapture(video);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight);
        cap.set(cv::CAP_PROP_FPS,30);
        // LineState flag;
        ROS_INFO("get");
        ROS_INFO("%s",video.c_str());
        cv::Mat frame;

        while (cap.isOpened()) {
            /** 2.get frame and turn to binary **/
            cap.read(frame);
            cv::Mat gray;
//            ROS_INFO("frame.channel: %d",frame.channels());

            cv::imshow("frame",frame);
            cv::waitKey(33);
//            if(frame.channels()==1){
//                ROS_INFO("use time : %f",(ros::Time::now()-t1).toSec());
//                cv::imshow("frame",frame);
//                cv::waitKey(0);
//                continue;
//            }
            cv::cvtColor(frame, gray, CV_RGB2GRAY);
            cv::Mat binary;
            cv::threshold(gray, binary, 0, 255, CV_THRESH_OTSU);
            // cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
//            cv::imshow("show", binary);
//            cv::waitKey(0);

            /** 3.get point of two side **/
            int vline = binary.cols / 2;
            int pline = binary.rows * 2 / 3 - 20;
            vector<cv::Point2f> uvRight;
            vector<cv::Point2f> uvLeft;
            // right side
            for (int i = pline; i < binary.rows - 1; ++i) {
                uchar *data = binary.ptr<uchar>(i);
                for (int j = vline; j < binary.cols - 1; ++j) {
                    if (data[j] == 255 && data[j + 1] == 0) {
                        uvRight.push_back(cv::Point2f(j, i));
                        break;
                    }
                }
            }
            // left side
            for (int i = pline; i < binary.rows - 1; ++i) {
                uchar *data = binary.ptr<uchar>(i);
                for (int j = vline; j > 0; --j) {
                    if (data[j] == 255 && data[j - 1] == 0) {
                        uvLeft.push_back(cv::Point2f(j, i));
                        break;
                    }
                }
            }
            ROS_INFO("right.size = %d,left.size = %d",uvRight.size(),uvLeft.size());
            /** 4.transport **/
            if(uvLeft.size()>0&&uvRight.size()>0){
                pubMiddleline(uvLeft,uvRight);
            }
            else if(uvLeft.size()>0){
                pubMiddleline(uvLeft);
            }
            else if(uvRight.size()>0){
                pubMiddleline(uvRight);
            }
        }
        ROS_INFO("can't load the video!");
    }
    return 0;
}