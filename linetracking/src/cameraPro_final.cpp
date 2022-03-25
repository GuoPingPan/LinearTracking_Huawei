#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

/** 参数定义 **/
const float fx = 760.4862674784594; // 相机内参
const float fy = 761.4971958529285;
const float cx = 631.6715834996345;
const float cy = 329.3054436037627;

const float y = 129.5585;   // 固定相机离地面高度，单位mm，通过计算得到，不一定准

// 以下参数意思 main 函数有注释
float offset;
float thetaX;
float thetaY;
float scale;
int imgWidth;
int imgHeight;
float up_rate;
float down_rate;
int sizeThres;
ros::Publisher pathpub;
ros::Publisher leftpoint;
ros::Publisher rightpoint;

/** 将图片中的 点uv 转化为 现实坐标中的 点xy **/
cv::Point3f getXYZ(cv::Point2f& point){
    // 就是简单地通过相机内参来完成像素坐标到世界坐标的转换
    cv::Point3f point3d;
    point3d.x = (fy * y/(point.y-cy))/scale;
    point3d.y = (-(point.x-cx)*point3d.x/fx);
    point3d.z = 0.0;
    // std::cout<<point3d<<std::endl;
    return point3d;
}

/** 将二维边线转化为三维边线 **/
void transTo3D(vector<cv::Point2f>& uv,vector<cv::Point3f>& posearray){
    for(auto p:uv){
        cv::Point3f pose = getXYZ(p);
        posearray.push_back(pose);
    }
}

/** 配合方法一、二：处理单边情况 **/
void pubMiddleline(vector<cv::Point2f>& uv,bool is_left){
    /*
     * @param is_left 是为了区分单边线是左边线还是右边线
     * */
    vector<cv::Point3f> xy;
    sensor_msgs::PointCloud path;
    transTo3D(uv,xy);
    // path.points.resize(xy.size());
    int i = 0;
    for(auto p:xy){
        if(is_left){
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y - offset;
            point.z = 0.0;
            path.points.push_back(point);
        }
        else{
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y + offset;
            point.z = 0.0;
            path.points.push_back(point);
        }

    }
    ROS_INFO("pathpose:%d",path.points.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";
    pathpub.publish(path);
    ROS_INFO("publish successfully");
}

/** 方法四：混合处理不分单双边线 **/
void pubMiddlelineCombine(vector<cv::Point2f>& uvLeft,vector<cv::Point2f>& uvRight){
    
    // 1.将数据转化为三维坐标系下
    vector<cv::Point3f> xyLeft,xyRight;
    ROS_INFO("int two middle");
    transTo3D(uvLeft,xyLeft);
    transTo3D(uvRight,xyRight);

    // 2.参数定义
    int r = 0;
    int l = 0;
    float max_distance = 0;
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "camera";
    
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
            float tmp = ( xyLeft[l].y + xyRight[r].y )/2;
            if(abs(tmp - lastparallel)>0.1){    // 去除某些奇异不连续的点
                l++;r++;
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
            float tmp = xyLeft[l].y - offset; // 直接将左边点横坐标减去偏移量当作车道中线点
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
            float tmp = xyRight[r].y + offset; // 直接将右边点横坐标减去偏移量当作车道中线点
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
    pathpub.publish(pc);
}


int main(int argc ,char **argv){

    // 车道线跟踪节点初始化
    ros::init(argc,argv,"linktracking");
    ros::NodeHandle nh("~");

    // 信息发布者定义
    pathpub = nh.advertise<sensor_msgs::PointCloud>("path",1);         // 三维空间下的点云路径
    leftpoint = nh.advertise<sensor_msgs::PointCloud>("leftpath",1);   // 左车道线边界
    rightpoint = nh.advertise<sensor_msgs::PointCloud>("rightpath",1); // 右车道线边界

    // 参数定义
    string video;
    nh.param<string>("video",video,"/dev/video1");  // 相机入口
    nh.param<float>("offset",offset,0.45);          // 车道中线与边线距离
    nh.param<float>("thetaX",thetaX,0.2);           // 左右边线点x轴距离容忍度0.2m
    nh.param<float>("thetaY",thetaY,0.1);           // 左右边线点y轴距离容忍度0.1m
    nh.param<float>("scale",scale,1000.0);          // 尺度，相机坐标与世界坐标的尺度变换
    nh.param<int>("imgWidth",imgWidth,1280);        // 图像宽度
    nh.param<int>("imgHeight",imgHeight,720);       // 图像高度

    nh.param<float>("up_rate",up_rate,0.5);         // 截取图像高度 up_rate*imgHeight 到 down_rate*imgHeight 区域
    nh.param<float>("down_rate",down_rate,0.9);

    nh.param<int>("size_thres",sizeThres,10);       // 点数阈值，当边线点数少于该数时当作不存在该边线

    // 部分参数输出
    ROS_INFO("offset : %f",offset);
    ROS_INFO("thetaX : %f",thetaY);
    ROS_INFO("thetaY : %f",thetaX);
    ROS_INFO("scale : %f",scale);
    ROS_INFO("imgWidth : %d",imgWidth);
    ROS_INFO("imgHeight : %d",imgHeight);

    /** 1.获得视频数据 **/
    cv::VideoCapture cap = cv::VideoCapture(video);
    cap.set(3, imgWidth);//CV_CAP_PROP_FRAME_WIDTH
    cap.set(4, imgHeight);//CV_CAP_PROP_FRAME_HEIGHT
    cap.set(cv::CAP_PROP_FPS,30);

    ROS_INFO("%s",video.c_str());
    cv::Mat frame;
    cv::Mat gray;
    cv::Mat binary;
    
    if(!cap.isOpened()){
        ROS_INFO("Don't load the Image!");
        return 1;
    }

    int left_num=1,right_num=1;
    while (ros::ok()&&cap.isOpened()) {
        /** 2.获得帧并二值化 **/
        cap.read(frame);
        if(!frame.data){
            ROS_INFO("Don't get this the frame");
            break;
        }

        // 自适应二值化
        cv::cvtColor(frame, gray, 7);//CV_RGB2GRAY
        cv::threshold(gray, binary, 150, 255, 0);//CV_THRESH_BINARY
        cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(binary,binary,11);

        /** 3.获得两边轨迹点 **/
        // 动态中线，通过边线点数差值比来规划中线位置
        int base_vline = frame.cols / 2;
        int float_range = frame.cols / 4;
        float float_rate = ((long)left_num - right_num) / ((long)left_num + right_num + 0.1);
        int vline = (int)(base_vline + float_range * float_rate);

        vector<cv::Point2f> uvRight;
        vector<cv::Point2f> uvLeft;
        // 提取右边线点
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j < binary.cols - 1; ++j) {
                if (data[j] == 0 && data[j + 1] == 255) {   // 右边车道线 为图片像素由黑转白处
                    right_num++;
                    for (int tmp = 0; tmp < 10; tmp++) {
                        frame.at<Vec3b>(i, j - tmp)[0] = 0;
                        frame.at<Vec3b>(i, j - tmp)[1] = 0;
                        frame.at<Vec3b>(i, j - tmp)[2] = 255;
                        }
                    uvRight.push_back(cv::Point2f(j, i));
                    break;
                }
            }
       }
        // 提取左边线点
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j > 0; --j) {
                if (data[j] == 0 && data[j - 1] == 255) {   // 左边车道线 为图片像素由白转黑处
                    left_num++;
                    for (int tmp = 0; tmp < 10; tmp++) {
                        frame.at<Vec3b>(i, j + tmp)[0] = 0;
                        frame.at<Vec3b>(i, j + tmp)[1] = 255;
                        frame.at<Vec3b>(i, j + tmp)[2] = 0;
                    }
                    uvLeft.push_back(cv::Point2f(j, i));
                    break;
                }
            }
        }
        // 绘制中线
        for (int i = int(imgHeight * up_rate); i < int(imgHeight * down_rate) - 1; ++i) {
            for (int tmp = -1; tmp < 2; tmp++) {
                frame.at<Vec3b>(i, vline+tmp)[0] = 0;
                frame.at<Vec3b>(i, vline+tmp)[1] = 255;
                frame.at<Vec3b>(i, vline+tmp)[2] = 255;
            }

        }
        // 绘制基本中线
        for (int i = int(imgHeight * up_rate); i < int(imgHeight * down_rate) - 1; ++i) {
            frame.at<Vec3b>(i, base_vline)[0] = 128;
            frame.at<Vec3b>(i, base_vline)[1] = 128;
            frame.at<Vec3b>(i, base_vline)[2] = 128;
        }
        cv::imshow("frame",frame);
        cv::waitKey(33);
        

        /** 4.绘制两边线 **/
        ROS_INFO("right.size = %d,left.size = %d",uvRight.size(),uvLeft.size());

        // 两边线都存在
        if(uvRight.size()>sizeThres && uvLeft.size()>sizeThres){
            pubMiddlelineCombine(uvLeft,uvRight);
            ROS_INFO("combine");
        }
        // 只有右边线
        else if(uvRight.size()>=uvLeft.size()&&uvRight.size()>sizeThres){
            pubMiddleline(uvRight,0);
            ROS_INFO("right");
        }
        // 只有左边线
        else if(uvLeft.size()>uvRight.size()&&uvLeft.size()>sizeThres){
            pubMiddleline(uvLeft,1);
            ROS_INFO("left");
        }

    }
    cv::destroyAllWindows();
    cap.release();
    return 0;
}
