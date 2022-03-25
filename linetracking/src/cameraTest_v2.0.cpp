#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>
using namespace std;

/**
 *  该文件是没有删除注释的最终版 camera-line-tracking
 *
 *  @author: panguoping
 *  @version: v2.0
 *
 *  @tips: 和v1.0的区别在于 void pubMiddleline(vector<cv::Point2f>& uv)
 *          你可以直接搜索todo位置进行查看
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
float up_rate;
float down_rate;
int sizeThres;

ros::Publisher pathpub;
ros::Publisher leftpoint;
ros::Publisher rightpoint;

/** 将图片中的 点uv 转化为 现实坐标中的 点xy **/
cv::Point3f getXYZ(cv::Point2f& point){
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

/** 方法一：画出中间线 **/
void pubMiddleline(vector<cv::Point2f>& uvLeft,vector<cv::Point2f>& uvRight){
    vector<cv::Point3f> xyLeft,xyRight;
    sensor_msgs::PointCloud path;
    transTo3D(uvLeft,xyLeft);
    transTo3D(uvRight,xyRight);
//    std::cout<<xyLeft<<std::endl;
//    std::cout<<xyRight<<std::endl;

    // 选择最长size，将头部多出点去掉
    ROS_INFO("int two middle");
    bool lbigthanr = xyLeft.size() > xyRight.size() ? true:false;   
    int ir = 0;
    int il = 0;
    if(lbigthanr){
        path.points.resize(xyLeft.size());
        while(il<xyLeft.size()&&abs(xyLeft[il].x-xyRight[ir].x)>thetaX){
            ++il;
        }
    } else{
        path.points.resize(xyRight.size());
        while(ir<xyRight.size()&&abs(xyLeft[il].x-uvRight[ir].x)>thetaX){
            ++ir;
        }
    }
    // 生成左右边线和中线
    int k = 0;
    sensor_msgs::PointCloud pcLeft;
    pcLeft.header.stamp = ros::Time::now();
    pcLeft.header.frame_id = "camera";
    pcLeft.points.resize(uvLeft.size());

    sensor_msgs::PointCloud pcRight;
    pcRight.header.stamp = ros::Time::now();
    pcRight.header.frame_id = "camera";
    pcRight.points.resize(uvRight.size());
    int i = 0;
    //size需要强制转换
    for(i,k,ir,il; il<int(xyLeft.size()-5)&&ir<int(uvRight.size()-5);++ir,++il){    
        pcLeft.points[i].x = xyLeft[i].x;
        pcLeft.points[i].y = xyLeft[i].y;
        pcLeft.points[i].z = 0.0;
        pcRight.points[i].x = xyRight[i].x;
        pcRight.points[i].y = xyRight[i].y;
        pcRight.points[i].z = 0.0;
        ++i;
        if(abs(xyLeft[il].x-uvRight[ir].x)>thetaX   // 左右边线纵向距离不超过thetaX,否则舍弃
//           && abs(xyLeft[il+1].y-xyLeft[il].y)>thetaY
//           && abs(xyRight[ir+1].y-xyRight[ir].y)>thetaY
           )
            continue;
        ROS_INFO("yes");

        path.points[k].x = (xyLeft[il].x+xyRight[ir].x)/2;
        path.points[k].y = (xyLeft[il].y+xyRight[ir].y)/2;
        path.points[k].z = 0.0;
        ++k;

    }

    // 发布信息
    ROS_INFO("pathpose:%d",path.points.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";
    pathpub.publish(path);
    leftpoint.publish(pcLeft);
    rightpoint.publish(pcRight);
    ROS_INFO("publish successfully");


}

/** 方法二：不舍弃纵向距离点，直接+、-偏移量移到中央位置**/
void pubMiddlelinePro(vector<cv::Point2f>& uvLeft,vector<cv::Point2f>& uvRight){
    // 1.将数据转化为三维坐标系下
    vector<cv::Point3f> xyLeft,xyRight;
    ROS_INFO("int two middle");
    transTo3D(uvLeft,xyLeft);
    transTo3D(uvRight,xyRight);

    // 2.建立左右点云

    sensor_msgs::PointCloud pcLeft;
    pcLeft.header.stamp = ros::Time::now();
    pcLeft.header.frame_id = "camera";
    pcLeft.points.resize(uvLeft.size());

    sensor_msgs::PointCloud pcRight;
    pcRight.header.stamp = ros::Time::now();
    pcRight.header.frame_id = "camera";
    pcRight.points.resize(uvRight.size());

    int size = uvLeft.size()>=uvRight.size() ? int(uvLeft.size()):int(uvRight.size());
    sensor_msgs::PointCloud path;
    path.points.resize(uvLeft.size()+uvRight.size());

    // 3.左中右往下走
    int r = 0;
    int l = 0;
    int m = 0;
    int pline = imgHeight * 2 / 3 - 10;
    for(int i = pline;i<imgHeight;++i){
        // ROS_INFO("%d",i);
        // float z = ((fy * y)/(i - cy))/scale;
        ROS_INFO("lz:%f rz:%f",xyLeft[l].x,xyRight[r].x);
        if(l<xyLeft.size()&&r<xyRight.size()&&xyLeft[l].x==xyRight[r].x){   //当两者相等时
            ROS_INFO("Equal!");
            path.points[m].x = xyRight[r].x;
            path.points[m].y = (xyLeft[l].y+xyRight[r].y)/2;
            path.points[m].z = 0.0;
            pcLeft.points[l].x = xyLeft[l].x;
            pcLeft.points[l].y = xyLeft[l].y;
            pcLeft.points[l].z = 0.0;
            pcRight.points[r].x = xyRight[r].x;
            pcRight.points[r].y = xyRight[r].y;
            pcRight.points[r].z = 0.0;
            ++l;
            ++r;
            ++m;
        }
        else if(xyRight[r].x > xyLeft[l].x){    //若右距离大于左
            ROS_INFO("r bigthan l");
            if(r<xyRight.size()){
                path.points[m].x = xyRight[r].x;
                path.points[m].y = xyRight[r].y - offset;
                path.points[m].z = 0.0;
                pcRight.points[r].x = xyRight[r].x;
                pcRight.points[r].y = xyRight[r].y;
                pcRight.points[r].z = 0.0;
                ++r;
                ++m;
            }
        }
        else if(xyLeft[l].x>xyRight[r].x){  //若左距离大于右
                ROS_INFO("l bigthan r");
                if(l<xyLeft.size()){
                path.points[m].x = xyLeft[l].x;
                path.points[m].y = xyLeft[l].y + offset;
                path.points[m].z = 0.0;
                pcLeft.points[l].x = xyLeft[l].x;
                pcLeft.points[l].y = xyLeft[l].y;
                pcLeft.points[l].z = 0.0;
                ++l;
                ++m;
            }
        }

    }

    ROS_INFO("pathpose:%d",path.points.size());
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera";
    pathpub.publish(path);
    leftpoint.publish(pcLeft);
    rightpoint.publish(pcRight);
    ROS_INFO("publish successfully");


}

//@todo 相对于v1.0的版本，这里是使用append的方式来添加点，
// 防止预留空间的方法浪费空间，同时可能会出现一些奇怪的初始化现象

/** 配合方法一、二：处理单边情况 **/
void pubMiddleline(vector<cv::Point2f>& uv){
    vector<cv::Point3f> xy;
    sensor_msgs::PointCloud path;
    transTo3D(uv,xy);
    // path.points.resize(xy.size());
    int i = 0;
    for(auto p:xy){
        ROS_INFO("P.Y %f",p.y);
        if(p.y>0){
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y - offset;
            point.z = 0.0;
            path.points.push_back(point);
        }
        else if(p.y<0){
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

    pathpub.publish(pc);
}

/** 转换到lab色域 **/
void binaryFromLab(cv::Mat & frame,cv::Mat & binary){
    cv::Mat lab;
    cv::cvtColor(frame,lab,CV_RGB2Lab);
    cv::Mat lab_1;
    vector<cv::Mat> channels;
    cv::split(lab,channels);
    lab_1 = channels.at(1);
    double max;
    double min;
    cv::minMaxLoc(lab_1,&min,&max);
    float thres = max - min;
    for(int i = 0;i<frame.rows;++i){
        uchar* datalab = lab_1.ptr<uchar>(i);
        uchar* databinary = binary.ptr<uchar>(i);
        for(int j = 0;j<frame.cols;++j){
            if((datalab[j] - min)/thres<0.2){
                databinary[j] = 255;
            }
            else{
                databinary[j] = 0;
            }
        }
    }

}


/** cmyk色域 **/
void binaryFromCMYK(cv::Mat & frame,cv::Mat & binary){
    for(int i = 0;i<frame.rows;++i){
        uchar* dataframe = frame.ptr<uchar>(i);
        uchar* datagray = binary.ptr<uchar>(i);
        for(int j = 0;j<frame.cols;++j){
            uchar c = 255 - dataframe[j*3+2];
            uchar m = 255 - dataframe[j*3+2];
            uchar y = 255 - dataframe[j*3+2];
            uchar K = min(min(c,m),y);
            if(K == 255){
                datagray[j] = 0;
                continue;
            }
            uchar C = (uchar)((c - K)*255.0 / (255 - K));
            uchar M = (uchar)((m - K)*255.0 / (255 - K));
            uchar Y = (uchar)((y - K)*255.0 / (255 - K));
            if(Y>150){
                datagray[j] = 255;
            }
            else{
                datagray[j] = 0;
            }
        }
    }
}



int main(int argc ,char **argv){
    ros::init(argc,argv,"linktracking");
    ros::NodeHandle nh("~");
    pathpub = nh.advertise<sensor_msgs::PointCloud>("path",1);
    leftpoint = nh.advertise<sensor_msgs::PointCloud>("leftpath",1);
    rightpoint = nh.advertise<sensor_msgs::PointCloud>("rightpath",1);
    string video;
    nh.param<string>("video",video,"../dataset/test.mp4");
    nh.param<float>("offset",offset,0.45);
    nh.param<float>("thetaX",thetaX,0.2);
    nh.param<float>("thetaY",thetaY,0.1);
    nh.param<float>("scale",scale,1000.0);
    nh.param<int>("imgWidth",imgWidth,1280);
    nh.param<int>("imgHeight",imgHeight,720);
    nh.param<float>("up_rate",up_rate,0.5);
    nh.param<float>("down_rate",down_rate,0.9);
    nh.param<int>("size_thres",sizeThres,10);

    ROS_INFO("offset : %f",offset);
    ROS_INFO("thetaX : %f",thetaY);
    ROS_INFO("thetaY : %f",thetaX);
    ROS_INFO("scale : %f",scale);
    ROS_INFO("imgWidth : %d",imgWidth);
    ROS_INFO("imgHeight : %d",imgHeight);

    /** 1.获得视频数据 **/
    cv::VideoCapture cap = cv::VideoCapture(video);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgHeight);
    cap.set(cv::CAP_PROP_FPS,30);

    ROS_INFO("%s",video.c_str());
    cv::Mat frame;
    cv::Mat gray;
    cv::Mat binary;
    
    if(!cap.isOpened()){
        ROS_INFO("Don't load the Image!");
        return 1;
    }


    while (ros::ok()&&cap.isOpened()) {
        /** 2.获得帧并二值化 **/
        cap.read(frame);
        if(!frame.data){
            ROS_INFO("Don't get this the frame");
            break;
        }
        
        // method 1:转到lab色域
        // cv::Mat binary(frame.rows,frame.cols, CV_8UC1);
        // binaryFromLab(frame,binary);
        // cv::dilate(binary,binary,11);
        
        // method 2:自适应二值化
        cv::cvtColor(frame, gray, CV_RGB2GRAY);
        cv::threshold(gray, binary, 150, 255, CV_THRESH_BINARY);
        cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(binary,binary,11);
        
        // cv::Sobel(gray,binary,-1,0,1);
        //cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
        
        // method 3:适应性二值化
        // cv::adaptiveThreshold(frame,binary,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,11,2);
        // cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
        // cv::imshow("show", binary);
        // cv::waitKey(0);


        /** 3.获得两边轨迹点 **/
        int vline = binary.cols / 2;
        int pline = binary.rows * 2 / 3 - 10;
        vector<cv::Point2f> uvRight;
        vector<cv::Point2f> uvLeft;
        // right side
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j < binary.cols - 1; ++j) {
                if (data[j] == 0 && data[j + 1] == 255) {
                    data[j] = 150;
                    data[j+1] = 150;
                    data[j+2] = 150;
                    data[j+3] = 150;
                    data[j+4] = 150;
                    data[j+5] = 150;
                    data[j+6] = 150;
                    data[j+7] = 150;
                    data[j+8] = 150;
                    data[j+9] = 150;
                    data[j+10] = 150;
                    uvRight.push_back(cv::Point2f(j, i));
                    break;
                }
            }
        }
        // left side
        for (int i = int(imgHeight*up_rate); i < int(imgHeight*down_rate)-1; ++i) {
            uchar *data = binary.ptr<uchar>(i);
            for (int j = vline; j > 0; --j) {
                if (data[j] == 0 && data[j - 1] == 255) {
                    data[j] = 150;
                    data[j-1] = 150;
                    data[j-2] = 150;
                    data[j-3] = 150;
                    data[j-4] = 150;
                    data[j-5] = 150;
                    data[j-6] = 150;
                    data[j-7] = 150;
                    data[j-8] = 150;
                    data[j-9] = 150;
                    data[j-10] = 150;
                    uvLeft.push_back(cv::Point2f(j, i));
                    break;
                }
            }
        }
        // for (int i = pline; i < imgHeight; ++i) {
        //     uchar *data = binary.ptr<uchar>(i);
        //     for (int j = 0; j<imgWidth; ++j) {
        //         if (data[j] == 0 && data[j + 1] == 255) {
        //             data[j] = 150;
        //             data[j+1] = 150;
        //             data[j+2] = 150;
        //             data[j+3] = 150;
        //             data[j+4] = 150;
        //             data[j+5] = 150;
        //             data[j+6] = 150;
        //             data[j+7] = 150;
        //             data[j+8] = 150;
        //             data[j+9] = 150;
        //             data[j+10] = 150;
        //             uvLeft.push_back(cv::Point2f(j, i));
        //             break;
        //         }
        //     }
        // }
        cv::imshow("frame",binary);
        cv::waitKey(0);
        

        /** 4.绘制两边线 **/
        ROS_INFO("right.size = %d,left.size = %d",uvRight.size(),uvLeft.size());
    
        // if(uvLeft.size()>0&&uvRight.size()>0){
        //     pubMiddlelinePro(uvLeft,uvRight);
        // }
        // else if(uvLeft.size()>0){
        //     pubMiddleline(uvLeft);
        // }
        // else if(uvRight.size()>0){
        //     pubMiddleline(uvRight);
        // }
        if(uvRight.size()>sizeThres && uvLeft.size()>sizeThres){
            pubMiddlelineCombine(uvLeft,uvRight);
            ROS_INFO("combine");
        }
        else if(uvRight.size()>=uvLeft.size()&&uvRight.size()>sizeThres){
            pubMiddleline(uvRight);
            ROS_INFO("right");
        }
        else if(uvLeft.size()>uvRight.size()&&uvLeft.size()>sizeThres){
            pubMiddleline(uvLeft);
            ROS_INFO("left");
        }
            

    }
    cv::destroyAllWindows();
    cap.release();
    return 0;
}