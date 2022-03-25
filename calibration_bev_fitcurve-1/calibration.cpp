#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
using namespace std;

/**
 * 这个文件是cpp的相机标定程序，并且将参数写出到文件
 *
 *
 * */


void undistroied(cv::Mat& M, cv::Mat& distCoeffs);

int main(){
    ifstream fin("./calib/calibdata.txt");
//    std::string path = "./images/*.jpg";
//
//    // 使用glob函数读取所有图像的路径
//    cv::glob(path, images);
//    frame = cv::imread(images[i]);
    cv::Size board_size(9,6);
    string imgPath;
    vector<cv::Point2f> image_points;
    vector<vector<cv::Point2f>> image_points_seq;
    cv::Size square_size(4,4);
    vector<vector<cv::Point3f>> obj_points_seq;
    cv::Mat M = cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    cv::Mat distCoeffs = cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0));
    cv::Mat tvecs;
    cv::Mat rvecs;
    while (getline(fin,imgPath)){
        cv::Mat image = cv::imread(imgPath);
        if(image.data){
//            cv::imshow(imgPath,image);
//            cv::waitKey(0);
//            cv::destroyWindow(imgPath);
        } else{
            cerr<<"Error,this image: "<<imgPath<<" doesn't exist!"<<endl;
            continue;
        }
        if(cv::findChessboardCorners(image,board_size,image_points)==0){
            cerr<<"Error,can't find the corners"<<endl;
            continue;
        } else{
            cv::Mat gray;
            cv::cvtColor(image,gray,CV_RGBA2GRAY);
            cv::cornerSubPix(gray,image_points,
                             cv::Size(11,11),
                             cv::Size(-1,-1),
                             cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01)
            );
            image_points_seq.push_back(image_points);
        }
        cv::Mat imgClone = image.clone();
        cv::drawChessboardCorners(imgClone,board_size,image_points, true);
//        cv::imshow("draw",imgClone);
//        cv::waitKey(0);
//        cv::destroyWindow("draw");

        vector<cv::Point3f> obj_points;
        for(int i = 0;i<board_size.height;++i){
            for(int j = 0;j<board_size.width;++j){
                cv::Point3f point;
                point.x = j*square_size.width;
                point.y = i*square_size.height;
                point.z = 0.0;
                obj_points.push_back(point);
            }
        }
        obj_points_seq.push_back(obj_points);
    }
    cv::calibrateCamera(obj_points_seq,
                        image_points_seq,
                        cv::Size(954,718),
                        M,distCoeffs,
                        rvecs,tvecs,CV_CALIB_FIX_K3);
    cout<<"The K:\n"<<M<<endl;
    cout<<"The distCoeffs:\n"<<distCoeffs<<endl;
    cout<<"The rvec:\n"<<rvecs<<endl;
    cout<<"The tvecs:\n"<<tvecs<<endl;

    ofstream out("./calib/cameraParams1.txt");
    if(out.is_open()){
//        out<<"The K:\n"<<"[ "<<M.at<float>(0,0)<<" , "<<M.at<float>(0,1)<<" , "<<M.at<float>(0,2)<<" ]\n"
//                              <<M.at<float>(1,0)<<" , "<<M.at<float>(1,1)<<" , "<<M.at<float>(1,2)<<" ]\n"
//                              <<M.at<float>(2,0)<<" , "<<M.at<float>(2,1)<<" , "<<M.at<float>(2,2)<<" ] ]\n"<<endl;
        out<<"The K:\n"<<M<<endl<<endl;
        out<<"The distCoeffs: \n"<<distCoeffs<<endl<<endl;
        out<<"The rvec: \n"<<rvecs<<endl<<endl;
        out<<"The tvec: \n"<<tvecs<<endl<<endl;

    }

    undistroied(M,distCoeffs);


    return 0;
}

void undistroied(cv::Mat& M, cv::Mat& distCoeffs){
    cv::Mat m = cv::imread("./calib/image/26.png");
    cv::imshow("distort",m);
    cv::Mat mo;
    undistort(m, mo, M, distCoeffs);
    cv::imshow("undistort",mo);
    cv::waitKey(0);
}