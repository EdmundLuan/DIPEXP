#include <cstdio>
#include <cstdlib>
#include <cmath>
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

const double e = 2.718281828f;
const double pi = 3.141592654f;

//////////////////////滤波//////////////////

// 空域高斯函数
inline Mat GssMsk(int size, int sigma) {
    Mat ret(size, size, CV_64F);
    size >>= 1;
    sigma *= sigma;
    double sum = 0.0;
    for (int i = 0; i < ret.rows; i++)
        for (int j = 0; j < ret.cols; j++) {
            // G(x,y)=\dfrac{1}{2\pi \sigma^2} e^{-\frac{x^2+y^2}{2\sigma^2}}
            double d2 = (i - size) * (i - size) + (j - size) * (j - size);
            ret.at<double>(i, j) = 1.0 / (2 * pi * sigma) * pow(e, -d2 / sigma / 2);
            sum += ret.at<double>(i, j);
        }
    // Equalize
    for (int i = 0; i < ret.rows; i++)
        for (int j = 0; j < ret.cols; j++) {
            ret.at<double>(i, j) = ret.at<double>(i, j) / sum;
        }
    return ret;
}

// 空域高斯滤波器
void Gaussian(Mat input, Mat &output, double sigma, int mskSize) {
    output = input.clone();
    if (mskSize < 0 || mskSize % 2 == 0) {
        printf("Invalid size of Gaussian mask!\nProcessing abort!\n\n");
        return;
    }
    Mat msk = GssMsk(mskSize, sigma);
    mskSize >>= 1;
    // Convolution
    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            double res = 0;
            for (int s = i - mskSize; s <= i + mskSize; s++) {
                if (s < 0 || s >= input.rows) continue;
                for (int t = j - mskSize; t <= j + mskSize; t++) {
                    // Oversize, tapped to ZERO.
                    if (t < 0 || t >= input.cols) continue;
                    res += msk.at<double>(s - i + mskSize, t - j + mskSize) * input.at<uchar>(s, t);
                }
            }
            output.at<uchar>(i, j) = (uchar)(res + 0.5);
        }

    return;
}

// 图像二值化
void binaryzation(Mat input, Mat&output) {
    output = input.clone();
    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            output.at<uchar>(i, j) = input.at<uchar>(i, j) < 128 ? 0 : 255;
        }
}

// 快速傅里叶变换
void fastFuriorTransform(Mat image) {
}

// 理想低通滤波器函数
Mat ideal_lbrf_kernel(Mat scr, float sigma) {
    //return scr.clone();
}

// 频率域滤波函数
// src:原图像
// blur:滤波器函数
Mat freqfilt(Mat scr, Mat blur) {
}

//////////////////////形态学//////////////////

// 膨胀函数
void Dilate(Mat src, const Mat &tem, Mat &dst) {
    // The origin of template is always (tem.rows/2, tem.cols/2)
    dst = src.clone();
    for (int i = 0; i < src.rows; i++)
        for (int j = 0; j < src.cols; j++) {
            int hit = 0;
            for (int s = i - tem.rows / 2; s - i + tem.rows / 2 < tem.rows && !hit; s++) {
                if (s < 0 || s >= src.rows) continue;
                for (int t = j - tem.cols / 2; t - j + tem.cols / 2 < tem.cols && !hit; t++) {
                    if (t < 0 || t >= src.cols) continue;
                    const uchar& temEle = tem.at<uchar>(s - i + tem.rows / 2, t - j + tem.cols / 2);
                    if(temEle > 1) continue;
                    hit |= temEle & src.at<uchar>(s, t);
//                    printf("[%d, %d] %d", i,j,hit);
                }
            }
            dst.at<uchar>(i, j) = hit ? 255 : 0;
        }
    return;
}

// 腐蚀函数
void Erode(Mat src, const Mat &tem, Mat &dst) {
    dst = src.clone();
    for (int i = 0; i < src.rows; i++)
        for (int j = 0; j < src.cols; j++) {
            int fit = 1;
            for (int s = i - tem.rows / 2; s - i + tem.rows / 2 < tem.rows && fit; s++) {
                if (s < 0 || s >= src.rows) continue;
                for (int t = j - tem.cols / 2; t - j + tem.cols / 2 < tem.cols && fit; t++) {
                    if (t < 0 || t >= src.cols) continue;
                    const uchar& temEle = tem.at<uchar>(s - i + tem.rows / 2, t - j + tem.cols / 2);
                    if(temEle > 1) continue;	// "Don't care"s
                    fit &= temEle & src.at<uchar>(s, t);
//                    printf("[%d, %d] %d", i,j,hit);
                }
            }
            dst.at<uchar>(i, j) = fit ? 255 : 0;
        }
    return;
}


// MAIN

int main(int argc, char **argv) {
    VideoCapture capture;
    capture.open(0);//打开 zed 相机
    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化 ROS 节点
    ros::NodeHandle n;
// ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器
    if (!capture.isOpened()) {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
    // Structural Element
    Mat SE(5, 5, CV_8U, cv::Scalar::all(1));

    while (ros::ok()) {
        capture.read(frame);
        if (frame.empty()) {
            break;
        }
// Mat frIn = frame();//使用笔记本摄像头
        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        imshow("Original", frIn);
        cvtColor(frIn, frIn, CV_BGR2GRAY);
        imshow("Convert to Gray", frIn);
// 空域滤波函数
        Mat dst(frIn.rows, frIn.cols, CV_8U);
//        Gaussian(frIn, dst, 2, 9);
//        imshow("Spatial Gaussian", dst);
// 频域滤波函数
//      freqfilt();
// 将图像二值化
        binaryzation(frIn, frIn);
        imshow("Binarized",  frIn);
// 膨胀函数
//        Dilate(frIn, SE, dst);
//        imshow("Dilated", dst);
// 腐蚀函数
        Erode(frIn, SE, dst);
        imshow("Eroded", dst);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
