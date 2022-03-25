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
        cv::Mat cmyk(frame.rows,frame.cols,CV_8UC4);
        cout<<"yes"<<endl;
//        cv::cvtColor(frame, gray, CV_RGB2GRAY);
        for (int i = 0; i < frame.rows; ++i) {
            uchar *dataframe = frame.ptr<uchar>(i);
            uchar *datagray = gray.ptr<uchar>(i);
            uchar *datacmyk = cmyk.ptr<uchar>(i);
            for (int j = 0; j < frame.cols; ++j) {
                uchar c = 255 - dataframe[j * 3 + 0];
                uchar m = 255 - dataframe[j * 3 + 1];
                uchar y = 255 - dataframe[j * 3 + 2];
                uchar K = min(min(c, m), y);
                if (K == 255) {
                    datagray[j] = 0;
                    datacmyk[j*4+0] = 0;
                    datacmyk[j*4+1] = 0;
                    datacmyk[j*4+2] = 0;
                    datacmyk[j*4+3] = 255;
                    continue;
                }
                uchar C = (uchar)((c - K) * 255.0 / (255 - K));
                uchar M = (uchar)((m - K) * 255.0 / (255 - K));
                uchar Y = (uchar)((y - K) * 255.0 / (255 - K));
                datacmyk[j*4+0] = C;
                datacmyk[j*4+1] = M;
                datacmyk[j*4+2] = Y;
                datacmyk[j*4+3] = K;
                if (Y > 200) {
                    datagray[j] = 255;
                }
                else {
                    datagray[j] = 0;
                }
            }

        }
        cv::imshow("frame", gray);
        cv::waitKey(33);
        cv::imshow("frame", cmyk);
        cv::waitKey(0);
    }
}