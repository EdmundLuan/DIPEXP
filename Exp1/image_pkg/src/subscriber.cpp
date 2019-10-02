#include <stdlib.h>
#include <string.h>
#include <stdio.h>
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

int main(int argc, char **argv)
{
	VideoCapture capture;
	capture.open(0);//打开zed相机


	ROS_WARN("*****START");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
	ros::NodeHandle n;

	// ros::Rate loop_rate(10);//定义速度发布频率
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器


	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	waitKey(1000);
	Mat frame;//当前帧图片
	int nFrames = 0;//图片帧数
	int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
	int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高


	while (ros::ok())
	{
		capture.read(frame);
		if (frame.empty())
		{
			break;
		}

		Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片

		// 此处增加直方图均衡化
		Mat src, dst;
		src = frIn;
		// Convert to grayscale
		cvtColor(src, src, CV_BGR2GRAY);
		dst = src.clone();
		// Plot Histagram
		Mat dstHist;       
		int dims = 1;
		float hranges[] = {0, 256};
		const float *ranges[] = {hranges};   // 这里需要为const类型
		int size = 256;
		int channels = 0;
	
		//计算图像的直方图
		calcHist(&src, 1, &channels, Mat(), dstHist, dims, &size, ranges); // 画出图像src的直方图
		Mat dstImage(size, size, CV_8U, Scalar(0));
		//获取最大值和最小值
		double minValue = 0;
		double maxValue = 0;
		minMaxLoc(dstHist,&minValue, &maxValue, 0, 0);  //  在cv中用的是cvGetMinMaxHistValue
		
		//绘制出直方图
		//saturate_cast函数的作用即是：当运算完之后，结果为负，则转为0，结果超出255，则为255。
		int hpt = saturate_cast<int>(0.9 * size);
		for(int i = 0; i < 256; i++)
		{
			float binValue = dstHist.at<float>(i);           //   注意hist中是float类型   
			//拉伸到0-max
			int realValue = saturate_cast<int>(binValue * hpt/maxValue);
			line(dstImage,Point(i, size - 1),Point(i, size - realValue),Scalar(255));// 直方图的名字叫dstImage
		}

		// Apply Histogram Equalization
		//equalizeHist(src, dst);
		const short MAX_GRAY = 255;
		int numRow, numCol, minGrayScl, maxGrayScl;
		int prInt[MAX_GRAY + 1]; 
		double prFlt[MAX_GRAY + 1];

		// Initialize
		numRow = dst.rows;
		numCol = dst.cols;
		minGrayScl = 255;
		maxGrayScl = 0;
		memset(prInt, 0, sizeof(prInt));
		memset(prFlt, 0, sizeof(prFlt));
		// Frequency
		for (int i = 0; i < numRow ; i++)
			for (int j = 0; j < numCol; j++) {
				uchar grayScl = (uchar)dst.at<uchar>(i, j);
				++prInt[grayScl];
				//printf("[%d, %d]: %d\n", i, j, grayScl);
				if (grayScl < minGrayScl) minGrayScl = grayScl;
				if (grayScl > maxGrayScl) maxGrayScl = grayScl;
			}

		// Make histogram
		for (int i = minGrayScl; i < maxGrayScl + 1; i++) {
			prFlt[i] = (double)prInt[i] / (numRow) / (numCol);
			printf("prFlt[%d]: %lf ", i, prFlt[i]);
			if(i%5==0) putchar(10);
		}
		// Integral
		//prFlt[minGrayScl] *= (double)1 / 255;
		for (int i = 1; i < maxGrayScl + 1; i++) {
			//prFlt[i] *= (double)1 / 255;
			prFlt[i] += prFlt[i - 1];
		}
		// Transform
		for (int i = 0; i < numRow ; i++)
			for (int j = 0; j < numCol; j++) {
				uchar grayScl = dst.at<uchar>(i, j);
				dst.at<uchar>(i, j) = (uchar)(prFlt[grayScl] * 255 + 0.5f);
			}
		//Plot Histagram
		Mat dstHist1;       
		int dims1 = 1;
		float hranges1[] = {0, 256};
		const float *ranges1[] = {hranges1};   // 这里需要为const类型
		size = 256;
		channels = 0;
	
		//计算图像的直方图
		calcHist(&dst, 1, &channels, Mat(), dstHist1, dims1, &size, ranges1); 
		Mat dstImage1(size, size, CV_8U, Scalar(0));
		//获取最大值和最小值
		minValue = 0;
		maxValue = 0;
		minMaxLoc(dstHist1,&minValue, &maxValue, 0, 0);  //  在cv中用的是cvGetMinMaxHistValue
		
		//绘制出直方图
		//saturate_cast函数的作用即是：当运算完之后，结果为负，则转为0，结果超出255，则为255。
		int hpt1 = saturate_cast<int>(0.9 * size);
		for(int i = 0; i < 256; i++)
		{
			float binValue1 = dstHist1.at<float>(i);           //   注意hist中是float类型   
			//拉伸到0-max
			int realValue1 = saturate_cast<int>(binValue1 * hpt1/maxValue);
			line(dstImage1,Point(i, size - 1),Point(i, size - realValue1),Scalar(255));
		}

		imshow("Source", frIn);
		imshow("Gray", src);
		imshow("Equalized", dst);
		imshow("Histogram of Gray", dstImage);
		imshow("Histogram of Equalized", dstImage1);
		
		
		geometry_msgs::Twist cmd_red;

		// 车的速度值设置
		cmd_red.linear.x = LINEAR_X;
		cmd_red.linear.y = 0;
		cmd_red.linear.z = 0;
		cmd_red.angular.x = 0;
		cmd_red.angular.y = 0;
		cmd_red.angular.z = 0.2;

		pub.publish(cmd_red);

		ros::spinOnce();
//		loop_rate.sleep();
		waitKey(5);

	}


	return 0;
}


