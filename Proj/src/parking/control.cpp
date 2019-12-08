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
double last_angle = 0;
double last_bs = 0;
double slope_mean = 0;
double bs_mean  = 0;
//double point_ground =0;
//double a_z=0;
//int fast_time=0;
//int line_state=-1;//-1->left  1->right
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

void drawRect(Mat& img, RotatedRect &rect, Scalar clr=(255,255,255), int thickness=1, int lineType=8) {
    Point2f rectPoints[4];
    rect.points(rectPoints);
    for(int j = 0; j < 4; j++) {
        line(img, rectPoints[j], rectPoints[(j + 1) % 4], clr, thickness, lineType);
    }
}

//检查数据
void check(double & angle, double & bs) {
    //剔除野点_角度
    if (last_angle == 0) {
        last_angle = angle;
    } else if (abs(angle - last_angle) > 10 && last_angle != 0 ) {
        angle = last_angle;
    }

    //剔除野点_截距
    if (last_bs == 0) {
        last_bs = bs;
    } else if (abs(bs - last_bs) > 1000 && last_bs != 0 ) {
        bs = last_bs;
    }
}

double PIDlinear(double angle, double bs) {
    const double kp_linear = 1 / 1000;
    const double ki_linear = 1 / 1000;
    const double kd_linear = 1 / 1000;
    double dist, last_dist;
    double vx;
    //计算距离
    dist = 374 - 1 / (tan(angle * (330 - bs)));
    last_dist = 374 - 1 / (tan(last_angle * (330 - last_bs)));

    vx = kp_linear * dist + ki_linear * (last_dist + dist) + kd_linear * (dist - last_dist);

    return vx;
}

double PIDangle(double angle, double bs) {
    const double kp_angle = 1 / 100;
    const double ki_angle = 1 / 100;
    const double kd_angle = 1 / 100;
    double omegaz;
    double errAng = angle - 90;
    double last_errAng = last_angle - 90;

    omegaz = kp_angle * errAng + ki_angle * (last_errAng + errAng) + kd_angle * (errAng - last_errAng);

    return omegaz;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Control");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg;
    ros::Rate loop_rate(20);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub_obs = nh.subscribe("/isObs", 2, obsCallback);
    VideoCapture capture;
    capture.open(0);
    if (!capture.isOpened()) {
        printf("摄像头没有正常打开，重新插拔工控机上的摄像头\n");
        return 0;
    }
    waitKey(2000);
    //Mat a;
    //int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    //int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

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
        imshow("img_trans", img_trans);


        // Find contours
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Mat cntrPic = Mat::zeros(dstImg.rows, dstImg.cols, CV_8UC3);
        findContours(img_trans, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
        // Bounding rectangle and Draw
        vector<RotatedRect> rects(contours.size());
        RotatedRect maxRect;
        double maxArea = 0;
        for(int i = 0; i < contours.size(); i++) {
            rects[i] = minAreaRect(Mat(contours[i]));
            //drawRect(cntrPic, rects[i]);
            double area = rects[i].size.width * rects[i].size.height;
            if(area < maxArea) continue;
            maxArea = area;
            maxRect = rects[i];
        }
        if(maxArea == 0) {
            cout << "Object not Found!!\n" << endl;
            continue;
        }
        drawRect(cntrPic, maxRect, Scalar(255, 255, 255), 2);

       imshow("img_trans", dstImg);
        
        // 找出面积最大的矩形的2条长边，二者均值作为轴线
        double angle;
        slope_mean = 0;
        bs_mean = 0;
        Point2f pts[4];
        // 矩形的四个点0,1,2,3按逆时针顺序排列
        maxRect.points(pts);
        circle(cntrPic, pts[0],10, Scalar(255,0,0));    //Red
        circle(cntrPic, pts[1],10, Scalar(0,255,0));    //Green
        circle(cntrPic, pts[2],10, Scalar(0,0,255));    // Blue
        circle(cntrPic, pts[3],10, Scalar(0,255,255));  // Yellow
        if(NORM2(pts[0].x-pts[1].x,pts[0].y-pts[1].y)<NORM2(pts[0].x-pts[3].x,pts[0].y-pts[3].y)) {
            // 边01是短边，取边01和边02中点连线作为中轴
            Point2f  L[2];
            L[0].x = (pts[0].x + pts[1].x) / 2;
            L[1].x = (pts[2].x + pts[3].x) / 2;
            L[0].y = (pts[0].y + pts[1].y) / 2;
            L[1].y = (pts[2].y + pts[3].y) / 2;
            line(cntrPic, L[0], L[1], Scalar(0, 0, 255), 2, 8);

            if(abs(L[0].y - L[1].y) > EPS) {
                slope_mean = (L[1].x - L[0].x) / (L[0].y - L[1].y);
                angle = atan(slope_mean) / CV_PI * 180 + 90;
                bs_mean = L[0].x - L[0].y * slope_mean;
            }
        } else {
            // 边03是短边，取边03和边12中点连线作为中轴
            Point2f  L[2];
            L[0].x = (pts[0].x + pts[3].x) / 2;
            L[1].x = (pts[1].x + pts[2].x) / 2;
            L[0].y = (pts[0].y + pts[3].y) / 2;
            L[1].y = (pts[1].y + pts[2].y) / 2;
            line(cntrPic, L[0], L[1], Scalar(0, 0, 255), 2, 8);

            if(abs(L[0].y - L[1].y) > EPS) {
                slope_mean = (L[1].x - L[0].x) / (L[1].y - L[0].y);
                angle = atan(slope_mean) / CV_PI * 180 + 90;
                bs_mean = L[0].x - L[0].y * slope_mean;
            }
        }
        imshow("Line", cntrPic);

//================================霍夫变换================================
        vector<Vec4i> lines; //包含4个int类型的结构体
        int rho = 1;
        double theta = CV_PI / 180;
        int threshold = 20;
        int min_line_len = 50 ;
        int max_line_gap = 0;
//        HoughLinesP(dstImg, lines, rho, theta, threshold, min_line_len, max_line_gap);
//        //HoughLinesP(dstImg,lines,rho,theta,threshold);
//        Mat image_draw = Mat::zeros(color.s ize(), CV_8UC3);
//        for(size_t i = 0; i < lines.size(); i++) {
//            Vec4i L = lines[i];
//            line(image_draw, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255, 0, 255), 3, LINE_AA);
//        }
//        imshow("Hough", image_draw);
//=============================直线分组=============================
        // fine * step = 180
//        const int fine = 36;
//        int bucket[fine] = {0};
//        int step = 180 / fine;
//        double angle;
//        // Slope == dx/dy
//        for (int i = 0; i < lines.size(); i++) {
//            Vec4i L;
//            double slope;
//            L = lines[i];
//            if(abs(L[3] - L[1]) < EPS) continue;
//            slope = (L[2] - L[0]) * 1.0 / (L[3] - L[1]);
//            angle = atan(slope) / CV_PI * 180 + 90;
//            bucket[(int)(angle / step + 0.5)]++;
//        }
//        int maxCnt = 0, angIdx = 0;
//        for(int i = 0; i < fine; i++) {
//            if(maxCnt < bucket[i]) {
//                maxCnt = bucket[i];
//                angIdx = i;
//            }
//        }
//        double sum0 = 0, sum1 = 0, sum2 = 0, sum3 = 0;
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
//        slope_mean = 0;
//        bs_mean = 0;
//        int maxlen = 0;
//        for (int i = 0; i < lines.size(); i++) {
//            Vec4i L;
//            double slope;
//            L = lines[i];
//            if(abs(L[3] - L[1]) < EPS) continue;
//            slope = (L[2] - L[0]) * 1.0 / (L[3] - L[1]);
//            angle = atan(slope) / CV_PI * 180 + 90;
//            if((int)(angle / step + 0.5) != angIdx) continue;
//            int len = NORM2(L[2] - L[0], L[3] - L[1]);
//
//            if(len > maxlen) {
//                int b = L[0] - L[1] * slope;
//                bs_mean = b;
//                slope_mean = slope;
//                sum0 = L[0];
//                sum1 = L[1];
//                sum2 = L[2];
//                sum3 = L[3];
//            }
//        }
//        line(image_draw, Point(sum0, sum1), Point(sum2, sum3), Scalar(0, 0, 255), 2, LINE_AA);
//        imshow("Hough", image_draw);



//=========================PID=====================================
        check(angle, bs_mean);
        int num_line = maxArea == 0 ? 0 : 1;


        cout << "angle =" << (int)angle  << "   slope = " << slope_mean << "  b = " << bs_mean << endl;
        cout << "Num of lines: " << num_line << endl;

        //double errAng = angle - 90;
        //double errB = bs_mean-330;
        if(num_line == 0) { //don't stop
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
            cout << "couldn't find H !!" << endl;
        }

        if(1) {
            flag = 1;
        }
        cout << flag << endl;

        if (flag == 0) {
            msg.linear.x = PIDlinear(angle, bs_mean);
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        } else {
            msg.linear.x = PIDlinear(angle, bs_mean) ;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = PIDangle(angle, bs_mean);
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
        //剔除野点
        last_angle = angle;
        last_bs = bs_mean;

        ros::spinOnce();
        loop_rate.sleep();
        waitKey(5);

    }

    return 0;
}
