#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>

/**
 *  最初的lineTracking
 * 
 * 
*/

using namespace std;

const float fx = 760.4862674784594;
const float fy = 761.4971958529285;
const float cx = 631.6715834996345;
const float cy = 329.3054436037627;
const int imgWidth = 1280;
const int imgHeight = 720;
const float y = 129.5585;


inline void getXY(int u,int v,cv::Point2f& xy){
    xy.x = fy * y/(v-cy);
    xy.y = -(u-cx)*xy.x/fx;
}

void getLineworld(vector<cv::Point2f>& vIn,vector<cv::Point2f>& vOut){
    for(vector<cv::Point2f>::iterator in = vIn.begin();in!=vIn.end();++in){
        cv::Point2f out;
//        out.x = fy*y/(in->y-cy);
//        out.y = -(in->x-cx)*out.x/fx;
//        vOut.push_back(out);
        getXY(in->x,in->y,out);
        vOut.push_back(out);

    }
}

void image2PointCloud(vector<cv::Point2f> uv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    int lastX =  fy * y/(uv[0].y-cy);
    int lastY = -(uv[0].x-cx)*lastX/fx;;
    for(auto p2:uv){
        pcl::PointXYZ point;
        // in camera frame  y=const z=fy * y/(v-cy) x=(u-cx)*z/fx
        // but in world frame z = y x = z y = -x
        point.x = fy * y/(p2.y-cy);
        point.y = -(p2.x-cx)*point.x/fx;
        point.z = 2;
        if(abs(point.y - lastY)>10)
            continue;
        cloud->push_back(point);
        lastY = point.y;
    }
}

enum LineState{Right=0,Left,None,Both};

Eigen::Vector3f fixCurve(pcl::PointCloud<pcl::PointXYZ>::Ptr xy){
    int size = xy->size();
    Eigen::Matrix<float,Eigen::Dynamic,3> Y;
    Eigen::Matrix<float,Eigen::Dynamic,1> X;
    for(int i = 0;i<xy->points.size();++i){
        Y(i,0) = std::pow(xy->points[i].y,2);Y(i,1) = xy->points[i].y;Y(i,2) = 1;
        X(i,0) = xy->points[i].x;
    }
    return (X.transpose()*X).inverse()*X.transpose()*Y;
}
//void calculateCurve(pcl::PointCloud<pcl::PointXYZ>::Ptr xyLeft,pcl::PointCloud<pcl::PointXYZ>::Ptr xyRight,LineState flag){
//    Eigen::Vector3f params;
//    if()
//}

float getOffset(int vline,int begin,int end){
    float offset=0;
    for(int i = begin;i<end;++i){
        cv::Point2f point;
        getXY(vline,i,point);
        offset += point.y;
    }
    return offset/(end-begin);
}

void getMiddleLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr left,const pcl::PointCloud<pcl::PointXYZ>::Ptr right,pcl::PointCloud<pcl::PointXYZ>::Ptr middle){

//    int size = left->size()>right->size() ? right->size():left->size();
    bool lbigthanr = left->size()>right->size() ? 1:0;
    int ir = 0;
    int il = 0;
    if(lbigthanr){
        while(abs(left->points[il].x-right->points[ir].x)>20){
            il++;
        }
    } else{
        while(abs(left->points[il].x-right->points[ir].x)>20){
            ir++;
        }
    }
    for(ir,il; il<left->size()&&ir<right->size();++ir,++il){
        if(abs(left->points[il].x-right->points[ir].x)>10
            && abs(left->points[il].y -left->points[il-1].y)>5
               && abs(right->points[ir].y -right->points[ir-1].y)>5)
            continue;
        pcl::PointXYZ point;
        point.x = (left->points[il].x+right->points[ir].x)/2;
        point.y = (left->points[il].y+right->points[ir].y)/2;
        point.z = (left->points[il].z+right->points[ir].z)/2;
        middle->push_back(point);
    }
}


void test(){
    cv::Mat frame = cv::imread("./calib/image/distort.png");
    cv::Mat gray;
    cv::cvtColor(frame,gray,CV_RGBA2GRAY);
    cv::Mat binary;
    cv::threshold(gray,binary,0,255,CV_THRESH_OTSU);
    cv::erode(binary,binary,cv::Mat(),cv::Point(-1,-1),2);
    cv::imshow("show",binary);
    cv::waitKey(0);
}
void test1(){
    cv::Mat mat = cv::imread("./calib/image/42.png");
    if(mat.data){
        cv::imshow("test",mat);
        cv::waitKey(0);
        cv::Point2f uv1(656,674);
        cv::Point2f uv2(656,533);
        cv::Point2f uv3(1098,533);
        vector<cv::Point2f> uv{uv1,uv2,uv3};
        vector<cv::Point2f> xy;
        getLineworld(uv,xy);
        for(auto p:xy){
            cout<<p<<endl;
        }
    }
}
void test2(){
    /** 1.get the picture **/
    cv::Mat frame = cv::imread("./calib/image/51.png");
    if(!frame.data){
        cerr<<"the frame doesn't exist"<<endl;
        return;
    }
    /** 2.turn the picture to binary **/
    cv::Mat gray;
    cv::cvtColor(frame,gray,CV_RGB2GRAY);
    cv::Mat binary;
    cv::threshold(gray,binary,0,255,CV_THRESH_OTSU);
//    cv::erode(binary,binary,cv::Mat(),cv::Point(-1,-1),2);
    cv::imshow("show",binary);
    cv::waitKey(0);

    /** 3.calculate the two side point **/
    int vline = binary.cols/2;  // vertical line
    int pline = binary.rows*2/3-20; // parallel line
    vector<cv::Point2f> uvRight;
    vector<cv::Point2f> uvLeft;
    LineState flag;
    // right side
    for(int i = pline;i<binary.rows-1;++i){
        uchar* data = binary.ptr<uchar>(i);
        for(int j = vline;j<binary.cols-1;++j){
            if(data[j]==255&&data[j+1]==0){ // white to black
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
                uvRight.push_back(cv::Point2f(j,i));
                break;
            }
        }
    }
    // left side
    for(int i = pline;i<binary.rows-1;++i){
        uchar* data = binary.ptr<uchar>(i);
        for(int j = vline;j>0;--j){
            if(data[j]==255&&data[j-1]==0){ // white to black
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
                uvLeft.push_back(cv::Point2f(j,i));
                break;
            }
        }
    }
    if(uvLeft.size()>0&&uvRight.size()>0){
        flag = Both;
    }
    else if(uvLeft.size()>0){
        flag = Left;
    }
    else if(uvRight.size()>0){
        flag = Right;
    }
    else{
        flag = Both;
    }
//    cout<<cv::mean(uvLeft)<<endl;
//    cout<<uvLeft<<endl;
//    cout<<"******************"<<endl;
//    std::sort(uvLeft.begin(),uvLeft.end(),[](const cv::Point2f& p1,const cv::Point2f& p2){return p1.x>p2.x;});
    cout<<uvLeft<<endl;
    /** show the gray boundary **/
    cv::imshow("show",binary);
    cv::waitKey(0);
    cv::destroyAllWindows();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr middle(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr average(new pcl::PointCloud<pcl::PointXYZ>());
    image2PointCloud(uvRight,cloud1);
    image2PointCloud(uvLeft,cloud2);
    pcl::copyPointCloud(*cloud1,*middle);
//    for(int i =0 ;i<middle->size();++i){
//        middle->points[i].y = 0;
//    }
    getMiddleLine(cloud1,cloud2,average);
//    float offset = getOffset(vline,pline,binary.rows-10);
//    cout<<offset<<endl;


    pcl::visualization::PCLVisualizer viewer("line");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source1(cloud1,255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source2(cloud2,0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source3(middle,255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source4(average,0,0,255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud1,source1,"cloud1");
    viewer.addPointCloud<pcl::PointXYZ>(cloud2,source2,"cloud2");
    viewer.addPointCloud<pcl::PointXYZ>(middle,source3,"cloud3");
    viewer.addPointCloud<pcl::PointXYZ>(average,source4,"cloud4");
    viewer.addCoordinateSystem(300,0,0,0);
//    for(auto p:*middle){
//        cout<<p<<endl;
//    }
    viewer.spin();
}

void test3(){
    /** 1.get the picture **/
    cv::VideoCapture cap  = cv::VideoCapture("");
    if(cap.isOpened()){
        cv::Mat frame  = cap.read();
 
        /** 2.turn the picture to binary **/
        cv::Mat gray;
        cv::cvtColor(frame,gray,CV_RGB2GRAY);
        cv::Mat binary;
        cv::threshold(gray,binary,0,255,CV_THRESH_OTSU);
    //    cv::erode(binary,binary,cv::Mat(),cv::Point(-1,-1),2);
        // cv::imshow("show",binary);
        // cv::waitKey(0);

        /** 3.calculate the two side point **/
        int vline = binary.cols/2;  // vertical line
        int pline = binary.rows*2/3-20; // parallel line
        vector<cv::Point2f> uvRight;
        vector<cv::Point2f> uvLeft;
        LineState flag;
        // right side
        for(int i = pline;i<binary.rows-1;++i){
            uchar* data = binary.ptr<uchar>(i);
            for(int j = vline;j<binary.cols-1;++j){
                if(data[j]==255&&data[j+1]==0){ // white to black
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
                    uvRight.push_back(cv::Point2f(j,i));
                    break;
                }
            }
        }
        // left side
        for(int i = pline;i<binary.rows-1;++i){
            uchar* data = binary.ptr<uchar>(i);
            for(int j = vline;j>0;--j){
                if(data[j]==255&&data[j-1]==0){ // white to black
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
                    uvLeft.push_back(cv::Point2f(j,i));
                    break;
                }
            }
        }
        if(uvLeft.size()>0&&uvRight.size()>0){
            flag = Both;
        }
        else if(uvLeft.size()>0){
            flag = Left;
        }
        else if(uvRight.size()>0){
            flag = Right;
        }
        else{
            flag = Both;
        }
    //    cout<<cv::mean(uvLeft)<<endl;
    //    cout<<uvLeft<<endl;
    //    cout<<"******************"<<endl;
    //    std::sort(uvLeft.begin(),uvLeft.end(),[](const cv::Point2f& p1,const cv::Point2f& p2){return p1.x>p2.x;});
        cout<<uvLeft<<endl;
        /** show the gray boundary **/
        cv::imshow("show",binary);
        cv::waitKey(0);
        cv::destroyAllWindows();


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr middle(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr average(new pcl::PointCloud<pcl::PointXYZ>());
        image2PointCloud(uvRight,cloud1);
        image2PointCloud(uvLeft,cloud2);
        pcl::copyPointCloud(*cloud1,*middle);
    //    for(int i =0 ;i<middle->size();++i){
    //        middle->points[i].y = 0;
    //    }
        getMiddleLine(cloud1,cloud2,average);
    //    float offset = getOffset(vline,pline,binary.rows-10);
    //    cout<<offset<<endl;


        pcl::visualization::PCLVisualizer viewer("line");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source1(cloud1,255,0,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source2(cloud2,0,255,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source3(middle,255,255,255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source4(average,0,0,255);
        viewer.addPointCloud<pcl::PointXYZ>(cloud1,source1,"cloud1");
        viewer.addPointCloud<pcl::PointXYZ>(cloud2,source2,"cloud2");
        viewer.addPointCloud<pcl::PointXYZ>(middle,source3,"cloud3");
        viewer.addPointCloud<pcl::PointXYZ>(average,source4,"cloud4");
        viewer.addCoordinateSystem(300,0,0,0);
    //    for(auto p:*middle){
    //        cout<<p<<endl;
    //    }
        viewer.spin();
    }
    
}



int main(){

    test3();

    return 0;
}


//int main() {
//
//    /** get the video **/
//    cv::VideoCapture cap = cv::VideoCapture("");
//    cv::Mat frame = cv::imread("./calib/image/50.png");
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, imgWidth);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgWidth);
//    LineState flag;
//    pcl::visualization::PCLVisualizer viewer("line");
//    while (cap.isOpened()) {
//        /** get frame and turn to binary **/
//        cv::Mat frame;
//        cap.read(frame);
//        cv::Mat gray;
//        cv::cvtColor(frame, gray, CV_RGB2GRAY);
//        cv::Mat binary;
//        cv::threshold(gray, binary, 0, 255, CV_THRESH_OTSU);
//        cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
//        cv::imshow("show", binary);
//        cv::waitKey(0);
//
//        /** get point of two side **/
//        int vline = binary.cols / 2;
//        int pline = binary.rows * 2 / 3 - 20;
//        vector<cv::Point2f> uvRight;
//        vector<cv::Point2f> uvLeft;
//        // right side
//        for (int i = pline; i < binary.rows - 1; ++i) {
//            uchar *data = binary.ptr<uchar>(i);
//            for (int j = vline; j < binary.cols - 1; ++j) {
//                if (data[j] == 255 && data[j + 1] == 0) {
//                    data[j] = 150;
//                    data[j + 1] = 150;
//                    data[j + 2] = 150;
//                    data[j + 3] = 150;
//                    data[j + 4] = 150;
//                    data[j + 5] = 150;
//                    data[j + 6] = 150;
//                    data[j + 7] = 150;
//                    data[j + 8] = 150;
//                    data[j + 9] = 150;
//                    data[j + 10] = 150;
//                    uvRight.push_back(cv::Point2f(j, i));
//                    break;
//
//                }
//            }
//        }
//        // left side
//        for (int i = pline; i < binary.rows - 1; ++i) {
//            uchar *data = binary.ptr<uchar>(i);
//            for (int j = vline; j > 0; --j) {
//                if (data[j] == 255 && data[j - 1] == 0) {
//                    data[j] = 150;
//                    data[j - 1] = 150;
//                    data[j - 2] = 150;
//                    data[j - 3] = 150;
//                    data[j - 4] = 150;
//                    data[j - 5] = 150;
//                    data[j - 6] = 150;
//                    data[j - 7] = 150;
//                    data[j - 8] = 150;
//                    data[j - 9] = 150;
//                    data[j - 10] = 150;
//                    uvLeft.push_back(cv::Point2f(j, i));
//                    break;
//                }
//            }
//        }
//        if(uvLeft.size()>0&&uvRight.size()>0){
//            flag = Both;
//        }
//        else if(uvLeft.size()>0){
//            flag = Left;
//        }
//        else if(uvRight.size()>0){
//            flag = Right;
//        }
//        else{
//            flag = Both;
//        }
//
//        /** show the boundary **/
//        cv::imshow("show", binary);
//        cv::waitKey(0);
//
//        /** visualization **/
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
//        image2PointCloud(uvRight, cloud1);
//        image2PointCloud(uvLeft, cloud2);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source1(cloud1, 255, 0, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source2(cloud2, 255, 255, 0);
//        viewer.addPointCloud<pcl::PointXYZ>(cloud1, source1, "cloud1");
//        viewer.addPointCloud<pcl::PointXYZ>(cloud2, source2, "cloud2");
//        viewer.addCoordinateSystem(1000, 0, 0, 0);
//        viewer.spin();
//
//    }
//    return 0;
//}