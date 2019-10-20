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

const double e = 2.718281828f;
const double pi = 3.141592654f;
const float ZeroF = 0.000001f;

#define sqr(x) ((x)*(x))

/*=============================== Function ==================================*/
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

// Gradient Calculation
Mat calcGrdnt(Mat img, Mat mskX, Mat mskY) {
    Mat ret(img.rows, img.cols, CV_32F, Scalar::all(0));
    if (mskX.rows != mskY.rows || mskX.cols != mskY.cols) {
        printf("\n2 Mask's sizes DO NOT match!\n");
        return ret;
    }
    for (int i = 0; i < img.rows; i++)
        for (int j = 0; j < img.cols; j++) {
            float tmpX = 0.0f, tmpY = 0.0f;
            for (int s = i - mskX.rows / 2; s - mskX.rows < i - mskX.rows / 2; s++) {
                if (s < 0 || s > img.rows) continue;
                for (int t = j - mskX.cols / 2; t - mskX.cols < j - mskX.cols / 2; t++) {
                    if (t < 0 || t > img.cols) continue;
                    tmpX += img.at<uchar>(s, t) * mskX.at<float>(s - i + mskX.rows / 2, t - j + mskX.cols / 2);
                    tmpY += img.at<uchar>(s, t) * mskY.at<float>(s - i + mskY.rows / 2, t - j + mskY.cols / 2);
                }
            }
            ret.at<float>(i, j) = abs(tmpX) + abs(tmpY);
            //ret.at<vec<float, 2 > >(i, j)[1] = abs(tmpX) < ZeroF ? (tmpY < 0 ? -pi / 2 : pi / 2 : ) : arctan(tmpY / tmpX);
        }
    return ret;
}

// Non-maximum suppression
void nmS(Mat input, Mat &output, int size) {
    output = input.clone();
    if (size < 2)
        return;
    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            float maxP = 0.0f;
            for (int s = i - size / 2; s - size < i - size / 2; s++) {
                if (s < 0 || s > input.rows) continue;
                for (int t = j - size / 2; t - size < j - size / 2; t++) {
                    if (t < 0 || t > input.cols) continue;
                    maxP = maxP < input.at<float>(s, t) ? input.at<float>(s, t) : maxP;
                }
            }
            float &outij = output.at<float>(i, j);
            outij = abs(outij - maxP) < ZeroF ? outij  : 0;
        }
    return ;
}

// Delay Thresholding
void delayThsh(Mat input, Mat &output, int tH, int tL) {
    input.copyTo(output);
    tL = tH / tL;
    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            if (input.at<float>(i, j) < tL)
                output.at<float>(i, j) = 0;
            else if (input.at<float>(i, j) < tH) {
                bool find = false;
                for (int s = i - 1; s <= i + 1 && !find; s++)
                    for (int t = j - 1; t <= j + 1 && !find; t++) {
                        if (s < 0 || s >= input.rows || t < 0 || t >= input.cols) continue;
                        if (input.at<float>(s, t) > tH)
                            find = true;
                    }
                if (!find)
                    output.at<float>(i, j) = 0;
                else
                    output.at<float>(i, j) = 255;
            } else
                output.at<float>(i, j) = 255;
        }
}

// Canny Filter
void CannyFilt(Mat input, Mat &output) {
    input.copyTo(output);

    Mat grd;
    Mat sobelX = (Mat_<float>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
    Mat sobelY = (Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
    // Get gradient (with Sobel operator)
    grd = calcGrdnt(input, sobelX, sobelY);
    // Non-maximum suppression
    nmS(grd, grd, 3);
    // Delay Thresholding
    delayThsh(grd, grd, 42, 2);
    // Normalize to [0, 255]
    normalize(grd, grd, 0, 255, NORM_MINMAX);
    grd.convertTo(output, CV_8U);
}

// Line detection via Hough Transform
void HoughLine(Mat input, Mat &output) {
    output = Mat(input.rows, input.cols, CV_8U, Scalar(0));
    int rhoMax = (int)sqrt(input.rows * input.rows + input.cols * input.cols);
    int thetaMax = 91;
    int threshold = 65;

    typedef std::vector<int> veci;
    veci zeros(rhoMax, 0);
    std::vector<veci> argCnt(thetaMax, zeros), argFlag(thetaMax, zeros);
    std::vector<int> thetas;
    std::vector<int> rhos;

    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            if(input.at<uchar>(i, j) < 250) continue;
            for (int theta = 0; theta < thetaMax; theta++) {
                int rho = (int)(cos(pi / 2 / thetaMax * theta) * i + sin(pi / 2 / thetaMax * theta) * j + 0.5);
                if(rho < rhoMax) {
                    argCnt.at(theta).at(rho)++;
                    if (argFlag[theta][rho] == 0 && argCnt[theta][rho] > threshold) {
                        rhos.push_back(rho);
                        thetas.push_back(theta);
                        argFlag[theta][rho] = 1;
                    }
                }
            }
        }
    printf("%d\n", thetas.size());
    for(int i = 0; i < thetas.size(); i++) {
        printf("rho=%d  theta=%f\n", rhos[i], 90.0 * thetas[i] / thetaMax);
    }
    putchar(10);
    // paint the lines here.
    for(int x = 0; x < input.rows; x++) {
        for(int i = 0; i < rhos.size(); i++) {
            if(abs(sin(pi / 2 / thetaMax * thetas[i])) < ZeroF) {	// sin(theta) == 0, a vertical line
                if(x != rhos[i]) continue;
                for(int y = 0; y < input.cols; y++) {
                    if(input.at<uchar>(x, y) < 250) continue;
                    output.at<uchar>(x, y) = 255;
                }
            } else {
                int y = (int)((rhos[i] - x * cos(pi / 2 / thetaMax * thetas[i])) / sin(pi / 2 / thetaMax * thetas[i]) + 0.5);
                //y = abs(y);
                if(y > 0 && y < output.cols) {
                    if(input.at<uchar>(x, y) < 250) continue;
                    output.at<uchar>(x, y) = 255;
                }
            }
        }
    }
}

// Circle Detection via Hough Transform
void HoughCirc(Mat input, Mat &output) {
    output = Mat(input.rows, input.cols, CV_8U, Scalar(0));
    int rMax = (int)(sqrt(sqr(input.rows) + sqr(input.cols)) + 0.5);
    int aMax = 20;
    int bMax = 20;
    int threshold = 300;
    typedef std::vector<int> vi;
    typedef std::vector<vi> vii;
    vi zero1d(bMax, 0);
    vii zero2d(aMax, zero1d);
    std::vector<vii> argCnt(rMax, zero2d), argFlag(rMax, zero2d);
    std::vector<int> as, bs, rs;

    for(int x = 0; x < input.rows; x++)
        for(int y = 0; y < input.cols; y++) {
            if(input.at<uchar>(x, y) < 250)	continue;
            for(int a = 0; a < aMax; a++) for(int b = 0; b < bMax; b++) {
                    int r = (int)(sqrt(sqr(x - a) + sqr(x - b)) + 0.5);
                    if(r > rMax) continue;
                    ++argCnt.at(r).at(a).at(b);
                    if(argFlag[r][a][b] == 0 && argCnt[r][a][b] > threshold) {
                        as.push_back(a);
                        bs.push_back(b);
                        rs.push_back(r);
                        argFlag.at(r).at(a).at(b) = 1;
                    }
                }
        }
    // Draw circles
    for(int i = 0; i < rs.size(); i++ )
        for(int x = 0; x < output.rows; x++) {
            int t1 = sqr(rs[i]) - sqr(x - as[i]);
            if(t1 < 0) continue;
            int tmp = (int)(sqrt(t1) + 0.5);
            int y = bs[i] - tmp;
            if(y < 0 || y >= output.cols ) continue;
            // if(input.at<uchar>(x,y)>250) continue;
            output.at<uchar>(x, y) = 255;
            y = bs[i] + tmp;
            if(y < 0 || y >= output.cols ) continue;
            // if(input.at<uchar>(x,y)>250) continue;
            output.at<uchar>(x, y) = 255;
        }
}

/******************************** MAIN **************************************/

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

    while (ros::ok()) {
        capture.read(frame);
        if (frame.empty()) {
            break;
        }
// Mat frIn = frame();//使用笔记本摄像头
        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        frIn = imread("/home/edmund/run/DIPEXP/Exp3/src/img_seg/test.png");
        imshow("Original", frIn);
// 转化成灰度图
        cvtColor(frIn, frIn, CV_BGR2GRAY);
        imshow("Convert to Gray", frIn);
// 高斯平滑处理
        Mat dst(frIn.rows, frIn.cols, CV_8U);
        Gaussian(frIn, frIn, 4, 5);
        imshow("Spatial Gaussian", frIn);
// 二值化
//        binaryzation(frIn, frIn);
//        imshow("Binarized",  frIn);
// Canny边缘检测
        Mat bndry;
        CannyFilt(frIn, bndry);
        binaryzation(bndry, bndry);
        imshow("Boundaries", bndry);
// Hough线检测
        Mat detect;
//        HoughLine(bndry, detect);
//        imshow("Line Detection", detect);
// Hough圆检测
        HoughCirc(bndry, detect);
        imshow("Circle Detection", detect);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
