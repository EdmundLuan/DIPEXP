#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <complex>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#define LINEAR_X 0

using namespace std;
using namespace cv;

static void onChange(int, void*) {
    ;
}

void clrSeg(const Mat& src, Mat& des) {
    int minH = 10, minS = 0, minV = 221;
    int maxH = 180, maxS = 50, maxV = 255;
    //createTrackbar("minH", "src", &minH, 179, onChange, 0);
    inRange(src, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), des);
}

void morphOpt(const Mat& src, Mat& des) {
    const Mat SE = getStructuringElement(MORPH_RECT, Size(5, 5));	// Structuring element
// Open operation
    morphologyEx(src, des, MORPH_OPEN, SE);
// Close operation
    morphologyEx(des, des, MORPH_CLOSE, SE);
}

void dtmPrkMrk(const Mat& src, Mat& mrk) {
    Mat edges;
//	Canny Edge Detection
    Canny(src, edges, 100, 200, 5, false);
//	imshow("edges", edges);

// Find contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat cntrPic = Mat::zeros(edges.rows, edges.cols, CV_8UC3);

    findContours(edges, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    for(int i = 0; i < contours.size(); i++)
        drawContours(cntrPic, contours, i, Scalar(255, 255, 255), 1);
//	imshow("contours", cntrPic);

//	Find Rectangles
    vector<Rect> polyCntrs(contours.size());
    for(int i = 0; i < contours.size(); i++) {
        polyCntrs[i] = boundingRect(contours[i] );
    }
    for(int i = 0; i < contours.size(); i++)
        rectangle(cntrPic, polyCntrs[i], Scalar(0, 0, 255));
    imshow("contours", cntrPic);
}


/****************************** MAIN **************************************/

int main(int argc, char **argv) {
    VideoCapture capture;
    capture.open(0);//1打开 zed 相机, 0打开笔记本摄像头
    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化 ROS 节点
    ros::NodeHandle n;
// ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器
    if (!capture.isOpened()) {
        printf("摄像头没有正常打开,重新插拔工控机上的摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高

    while (ros::ok()) {
        capture.read(frame);
        if (frame.empty()) {
            break;
        }
// Mat frIn = frame();//使用笔记本摄像头
//        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        Mat frIn = frame.clone();

// Gaussian Filter
        GaussianBlur(frIn, frIn, Size(3, 3), 1);
        imshow("RGB", frIn);

// Convert to HSV
        Mat frHSV;
        cvtColor(frIn, frHSV, COLOR_BGR2HSV);
        imshow("HSV", frHSV);

// Color Segmentation
        Mat prkMrk;
        clrSeg(frHSV, prkMrk);
        imshow("White", prkMrk);

// Morphological operations
        morphOpt(prkMrk, prkMrk);

// Parking mark detection, determination and  fining
        dtmPrkMrk(prkMrk, prkMrk);


        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
