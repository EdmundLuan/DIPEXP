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

using namespace cv;




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
		imshow("Original", frIn);
		Mat frHSV;
		cvtColor(frIn, frHSV, COLOR_BGR2HSV);	// Convert to HSV
		imshow("HSV", frHSV);



        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}

