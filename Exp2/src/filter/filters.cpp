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

typedef std::vector<std::complex<double> > vcd;

const double e = 2.718281828f;
const double pi = 3.141592654f;


//////////////////////滤波//////////////////

// 空域高斯掩模
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
    // Normalize
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

template<typename T>
inline void centralize(Mat &mtrx) {
    for(int i = 0; i < mtrx.rows; i++)
        for(int j = 0; j < mtrx.cols; j++) {
            if((i ^ j) & 1)
                mtrx.at<T>(i, j) *= -1;
        }
}

// My FFT in 1 dimension  
void fFt1dim(std::complex<double>*TD, vcd& FD, int r, int flag=1) {
//r为log2N，即迭代次数
// flag是标识符，为负数时执行逆变换
    int	count;				
    int		i, j, k;				 
    int		bfsize, p;
    double	angle;				//Omega 
    std::complex<double> *W, *X1, *X2, *X;
    count = 1 << r;			
 
    W  = new std::complex<double>[count / 2];
    X1 = new std::complex<double>[count];
    X2 = new std::complex<double>[count];
    // 计算加权系数
    for(i = 0; i < count / 2; i++) {
        angle = (flag > 0 ? -1 : 1) * 2 * pi * (i / count);
        W[i] = std::complex<double> (cos(angle), sin(angle));
    }
    // 将时域点写入X1
    memcpy(X1, TD, sizeof(std::complex<double>) * count);

    for(k = 0; k < r; k++) {		//k为蝶形运算的级数
        for(j = 0; j < 1 << k; j++) {
            bfsize = 1 << (r - k); //做蝶形运算两点间距离
            for(i = 0; i < bfsize / 2; i++) {
                p = j * bfsize;
                X2[i + p] = X1[i + p] + X1[i + p + bfsize / 2];
                X2[i + p + bfsize / 2] = (X1[i + p] - X1[i + p + bfsize / 2]) * W[i * (1 << k)];
            }
        }
        X  = X1;
        X1 = X2;
        X2 = X;
    }
    // 重新排序
    for(j = 0; j < count; j++) {
        p = 0;
        for(i = 0; i < r; i++) {
            if (j & (1 << i)) {
                p += 1 << (r - i - 1);
            }
        }
        FD[j] = X1[p];
    }
    delete W;
    delete X1;
    delete X2;
}

// Show Spectrum of Frequency domain 
inline void prtSpec(Mat cmplxImg) {
// Magnitude
    std::vector<Mat> channels;
    Mat mag;
    split(cmplxImg, channels);
    magnitude(channels[0], channels[1], mag);
    mag += Scalar::all(1);
    log(mag, mag);
// Normalize(0, 255)
    normalize(mag, mag, 0, 255, NORM_MINMAX);
    Mat spec;
    mag.convertTo(spec, CV_8U);
    imshow("Spectrum", spec);
}

// My 2-dimensional FFT with O(n^2 log n) efficiency, but relatively slower due to larger constant. 
void FFT(Mat input, Mat &output, int flag = 1) {
    int r = 0, k = 1, size = MAX(input.rows, input.cols);
    std::complex<double> *TD;
    TD = new std::complex<double>[input.rows];
    vcd zero(size, 0);
    std::vector<vcd> out(size, zero);

    for(; k < input.rows; (k <<= 1), r++);
    input.copyTo(output);
	// Transform by rows
    for(int u = 0; u < output.rows; u++)	{
        for(int v = 0; v < output.cols; v++) {
            TD[v] = output.at<Vec2f>(u, v)[0];
        }
        fFt1dim(TD, out[u], r, flag);
    }
    // Transpose
    for(int u = 0; u < size; u++)
        for(int v = 0; v <= u; v++) {
            std::complex<double> tmp = out[u][v];
            out[u][v] = out[v][u];
            out[v][u] = tmp;
        }

    k = 1;
    for(r = 0; k < input.cols; (k <<= 1), r++);
    // Transform by colums
	for(int v = 0; v < output.cols; v++)	{
        for(int u = 0; u < output.rows; u++) {
            TD[u] = out[v][u];
        }
        fFt1dim(TD, out[v], r, flag);
    }

    // Transpose back
    for(int u = 0; u < size; u++)
        for(int v = 0; v <= u; v++) {
            std::complex<double> tmp = out[u][v];
            out[u][v] = out[v][u];
            out[v][u] = tmp;
        }

    for(int u = 0; u < output.rows; u++)
        for(int v = 0; v < output.cols; v++) {
            output.at<Vec2f>(u, v)[0] = (float)out[u][v].real();
            output.at<Vec2f>(u, v)[1] = (float)out[u][v].imag();
        }

    delete TD;
}

void dFt(Mat input, Mat &output, int flag = 1) {
    if(flag >= 0)
        dft(input, input);
    else
        dft(input, input, DFT_INVERSE);
	input.copyTo(output);
    return;
}

// 理想低通滤波器函数
Mat ideal_lbrf_kernel(Mat src, float sigma) {
    Mat msk(src.rows, src.cols, CV_8U, Scalar(0));
    int cx = src.rows / 2, cy = src.cols / 2;
    sigma *= sigma;
    for(int i = 0; i < src.rows; i++)
        for(int j = 0; j < src.cols; j++) {
            if((i - cx) * (i - cx) + (j - cy) * (j - cy)  < sigma)
                msk.at<uchar>(i, j) = 1;
        }
    return msk;
}

// 频率域滤波函数
Mat freqFilt(Mat src) {
// Pad
    int originalR = src.rows;
    int originalC = src.cols;
    int paddedR = 1, paddedC = 1;
    for(; paddedR < originalR; paddedR <<= 1);
    for(; paddedC < originalC; paddedC <<= 1);
    Mat re(paddedR, paddedC, CV_32F, Scalar(0));
    copyMakeBorder(src, re, 0, paddedR - originalR, 0, paddedC - originalC, BORDER_CONSTANT, Scalar(0));
    // IMPORTANT! Don't f**king know why, but would incur ERRORS without it.
    re = Mat_<float>(re);
// blur:滤波器函数
    Mat blur =  ideal_lbrf_kernel(re, 200);
    //imshow("filt", Mat_<float>(blur));

// *(-1)^(x+y)
    centralize<float>(re);
// Filter
    Mat cmplxImg;	// Complex Image
    merge(std::vector<Mat>({Mat_<float>(re), Mat::zeros(re.size(), CV_32F)}), cmplxImg);

	//My FFT
	FFT(cmplxImg, cmplxImg);
	// FFT in openCV
    //dFt(cmplxImg, cmplxImg);
    //prtSpec(cmplxImg);

    for(int i = 0; i < cmplxImg.rows; i++)
        for(int j = 0; j < cmplxImg.cols; j++) {
            cmplxImg.at<Vec2f>(i, j)[0] *= blur.at<uchar>(i, j);
            cmplxImg.at<Vec2f>(i, j)[1] *= blur.at<uchar>(i, j);
    }
    prtSpec(cmplxImg);

	//My FFT
	FFT(cmplxImg, cmplxImg, -1);
	// FFT in openCV
    //dFt(cmplxImg, cmplxImg, -1);
    std::vector<Mat> parts;
    split(cmplxImg, parts);
    parts[0] = Mat_<float>(parts[0]);
    centralize<float>(parts[0]);
    normalize(parts[0], parts[0], 0, 255, NORM_MINMAX);
    Mat back;
    parts[0].convertTo(back, CV_8U);
    back = back(Rect(0, 0, originalC, originalR));

    return back;
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


/****************************** MAIN **************************************/

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
        Gaussian(frIn, dst, 2, 9);
        imshow("Spatial Gaussian", dst);
// 频域滤波函数
        dst = freqFilt(frIn);
        imshow("LPF", dst);
// 二值化
        binaryzation(frIn, frIn);
        imshow("Binarized",  frIn);
// 膨胀函数
        Dilate(frIn, SE, dst);
        imshow("Dilated", dst);
// 腐蚀函数
        Erode(frIn, SE, dst);
        imshow("Eroded", dst);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
