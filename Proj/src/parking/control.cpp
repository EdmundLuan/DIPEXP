#include<opencv2/opencv.hpp>
#include<stdlib.h>
#include<vector>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <numeric>
using namespace std;
using namespace cv;

#define kp1 (1.2)
#define kd1 (0.27)
#define kp2 (0.008)
#define MAX_TURN (3)
#define NORM2(a,b) ((a)*(a)+(b)*(b))
#define EPS 0.000001
float last_slope = 0;
double slope_mean = 0;
double bs_mean  = 0;
double point_ground = 0;
double a_z = 0;
int fast_time = 0;
int line_state = -1; //-1->left  1->right
bool ISOBS = false;
bool flag = 0; //是否到达直线
void obsCallback(const std_msgs::Bool _isObs) {
    ISOBS = _isObs.data;

}


void perspectiveTransform(const Mat& src, Mat& img_trans) {
    vector<Point2f> corners(4);
    corners[0] = Point2f(275, 229);
    corners[1] = Point2f(404, 229);
    corners[2] = Point2f(236, 363);
    corners[3] = Point2f(466, 363);

    vector<Point2f> corners_trans(4);
    corners_trans[0] = Point2f(300, 274);
    corners_trans[1] = Point2f(380, 274);
    corners_trans[2] = Point2f(300, 374);
    corners_trans[3] = Point2f(380, 374);

    Mat transform = getPerspectiveTransform(corners, corners_trans);
    warpPerspective(src, img_trans, transform, Size(src.cols, src.rows), cv::INTER_AREA);
}

void morphOpt(const Mat& src, Mat& des) {
    const Mat SE = getStructuringElement(MORPH_RECT, Size(3, 3));	// Structuring element
// Close operation
    morphologyEx(des, des, MORPH_CLOSE, SE);
// Open operation
    morphologyEx(src, des, MORPH_OPEN, SE);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Control");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg;
    ros::Rate loop_rate(20);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub_obs = nh.subscribe("/isObs", 2, obsCallback);
    VideoCapture capture;
    capture.open(1);
    if (!capture.isOpened()) {
        printf("摄像头没有正常打开，重新插拔工控机上的摄像头\n");
        return 0;
    }
    waitKey(2000);

    Mat frame, color, edges, img_trans, dstImg;
    while(ros::ok()) {
//=========================图像处理========================================
        capture >> frame;
        frame = frame(Rect(0, 0, frame.size().width / 2, frame.size().height));
        frame.copyTo(color);

        GaussianBlur(color, color, Size(3, 3), 0, 0);

        cvtColor(color, color, COLOR_BGR2HSV);

        // 颜色分割
        inRange(color, Scalar(35, 43, 46), Scalar(99, 255, 255), color);
        imshow("color", color);
//===============================透视变换、边缘二值图=========================================

        perspectiveTransform(color, img_trans);
        dstImg = Mat::zeros(img_trans.rows, img_trans.cols, CV_8U);
        //morphOpt(img_trans, dstImg);
        Canny(img_trans, img_trans, 100, 300, 3);
        //dstImg=img_trans.clone();


        // Find contours
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        int areaThreshold = 50;
        Mat cntrPic = Mat::zeros(dstImg.rows, dstImg.cols, CV_8UC3);
        findContours(img_trans, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
        for(int i = 0; i < contours.size(); i++)
            drawContours(cntrPic, contours, i, Scalar(255, 255, 255), 1);
        vector<vector<Point> > contours_poly(contours.size());//用于存放折线点集
        for (int i = 0; i < contours.size(); i++) {
            approxPolyDP(Mat(contours[i]), contours_poly[i], 5, true);
            drawContours(dstImg, contours_poly, i, Scalar(255, 255, 255), 1, 8);
            //drawContours(dstImg, contours, i, Scalar(255, 255, 255), 1, 8);
        }
        imshow("img_trans", dstImg);

//================================霍夫变换================================
        vector<Vec4i> lines; //包含4个int类型的结构体
        int rho = 1;
        double theta = CV_PI / 180;
        int threshold = 20;
        int min_line_len = 50 ;
        int max_line_gap = 0;
        //HoughLinesP(dstImg,lines,rho,theta,threshold,min_line_len,max_line_gap);
        HoughLinesP(dstImg, lines, rho, theta, threshold);
        Mat image_draw = Mat::zeros(color.size(), CV_8UC3);
        for(size_t i = 0; i < lines.size(); i++) {
            Vec4i L = lines[i];
            line(image_draw, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255, 255, 255), 3, LINE_AA);
        }
        imshow("Hough", image_draw);
//=============================直线分组=============================
        // fine * step = 180
        const int fine = 36;
        int bucket[fine] = {0};
        int step = 180 / fine;
        double angle;
        // Slope == dx/dy
        for (int i = 0; i < lines.size(); i++) {
            Vec4i L;
            double slope;
            L = lines[i];
            if(abs(L[3] - L[1]) < EPS) continue;
            slope = (L[2] - L[0]) * 1.0 / (L[3] - L[1]);
            angle = atan(slope) / CV_PI * 180 + 90;
            bucket[(int)(angle / step + 0.5)]++;
        }
        int maxCnt = 0, angIdx = 0;
        for(int i = 0; i < fine; i++) {
            if(maxCnt < bucket[i]) {
                maxCnt = bucket[i];
                angIdx = i;
            }
        }
        double sum0 = 0, sum1 = 0, sum2 = 0, sum3 = 0;
//		for (int i=0;i<lines.size();i++)
//		{
//			Vec4i L;
//			double slope;
//			L = lines[i];
//			if(abs(L[3]-L[1])<EPS) continue;
//			slope = (L[2]-L[0])*1.0/(L[3]-L[1]);
//			angle = atan(slope)/CV_PI*180+90;
//			if((int)(angle/step+0.5)!=angIdx) continue;
//			else{
//				sum0 += L[0];
//				sum1 += L[1];
//				sum2 += L[2];
//				sum3 += L[3];
//			}
//		}
//		sum0 /= bucket[angIdx];
//		sum1 /= bucket[angIdx];
//		sum2 /= bucket[angIdx];
//		sum3 /= bucket[angIdx];
//
//		if(abs(sum3-sum1)<EPS) continue;
//		slope_mean=(sum2-sum0)*1.0/(sum3-sum1);
//
//		bs_mean = sum0-sum1*slope_mean;
        slope_mean = 0;
        bs_mean = 0;
        int maxlen = 0;
        for (int i = 0; i < lines.size(); i++) {
            Vec4i L;
            double slope;
            L = lines[i];
            if(abs(L[3] - L[1]) < EPS) continue;
            slope = (L[2] - L[0]) * 1.0 / (L[3] - L[1]);
            angle = atan(slope) / CV_PI * 180 + 90;
            if((int)(angle / step + 0.5) != angIdx) continue;
            int len = NORM2(L[2] - L[0], L[3] - L[1]);

            if(len > maxlen) {
                int b = L[0] - L[1] * slope;
                bs_mean = b;
                slope_mean = slope;
                sum0 = L[0];
                sum1 = L[1];
                sum2 = L[2];
                sum3 = L[3];
            }
        }
        line(image_draw, Point(sum0, sum1), Point(sum2, sum3), Scalar(0, 0, 255), 2, LINE_AA);
        imshow("Hough", image_draw);

//=========================PID=====================================
        cout << "angle =" << (int)angle  << "   slope = " << slope_mean << "  b = " << bs_mean << endl;
        cout << "Num of lines: " << bucket[angIdx] << endl;

        double errAng = angle - 90;
        double errB = bs_mean - 330;

        int num_line = bucket[angIdx];
        if(num_line == 0)
            return 0;

        if(flag == 0 && abs(errB) < 10) {
            flag = 1;
        }


        cout << flag << endl;

        if (flag == 0) {
            msg.linear.x = 0.1 + abs(bs_mean) / 5000;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        } else {
            msg.linear.x = 0.1 + abs(errB) / 5000;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = kp1 * (errAng) / 10 + kd1 * ( last_slope - slope_mean) ;
        }

        /*
                msg.linear.x=0.1;
        			msg.linear.y = 0;
        			msg.linear.z = 0;
        			msg.angular.x = 0;
        			msg.angular.y = 0;
        			msg.angular.z = 0;
        */
        pub.publish(msg);
        last_slope = slope_mean;

        ros::spinOnce();
        loop_rate.sleep();
        waitKey(5);

    }

    return 0;
}
