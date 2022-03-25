#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
using namespace std;

int main() {
    string video = "../dataset/test14.mp4";
    cv::VideoCapture cap = cv::VideoCapture(video);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 30);
    cv::Mat frame;

    while (cap.isOpened()) {
        /** 2.get frame and turn to binary **/
        cap.read(frame);
        cv::Mat gray(frame.rows,frame.cols,CV_8UC1);
        cv::Mat lab;
        cv::cvtColor(frame,lab,CV_RGB2Lab);
        cout<<"yes"<<endl;
        cv::imshow("frame", lab);
        cv::waitKey(0);
        for (int i = 0; i < frame.rows; ++i) {
            uchar *datagray = gray.ptr<uchar>(i);
            uchar *datalab = lab.ptr<uchar>(i);
            for (int j = 0; j < frame.cols; ++j) {
               if(datalab[j*3+2]>100){
                   datagray[j] = 255;
               }
               else{
                   datagray = 0;
               }
            }

        }

        cv::imshow("frame", gray);
        cv::waitKey(33);
        cv::imshow("frame", lab);
        cv::waitKey(0);
    }
}