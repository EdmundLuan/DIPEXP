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
using namespace std;

const double e = 2.718281828f;
const double pi = 3.141592654f;
const float ZeroF = 0.000001f;

#define sqr(x) ((x)*(x))

/*=============================== Function ==================================*/
void Swap(Vec3i &a,Vec3i &b)
{
    int temp[3];
    for (int i = 0; i < 3; ++i) {
        temp[i]=a[i];
        a[i]=b[i];
        b[i]=temp[i];
    }
}
void SelectionSort(vector<Vec3i>& center)
{
    int size=center.size();
    if(size<=1) return;
    int maxidx,i,j;
    for(i=0;i<size-1;++i)
    {
        maxidx=i;
        for(j=i+1;j<size;++j)
        {
            if(center[j][2]>center[maxidx][2] )
                maxidx=j;
        }
        Swap(center[i],center[maxidx]);
    }
    return;
}
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
    Mat ret(img.rows, img.cols, CV_32FC(2), Scalar::all(0));
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
            ret.at<Vec2f>(i, j)[0] = abs(tmpX) + abs(tmpY);
            ret.at<Vec2f>(i, j)[1] = abs(tmpX) < ZeroF ? (tmpY < 0 ? -pi / 2 : pi / 2 ) : atan(tmpY / tmpX);
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
            float &phi = input.at<Vec2f>(i, j)[1];
            float &mag = input.at<Vec2f>(i, j)[0];
            if(abs(phi) < pi / 8) { // Horizontal
                for(int s = i - size / 2; s - size < i - size / 2; s++) {
                    if (s < 0 || s >= input.rows) continue;
                    maxP = maxP < input.at<Vec2f>(s, j)[0] ? input.at<Vec2f>(s, j)[0] : maxP;
                }
            } else if (phi > 3 * pi / 8 || phi < -3 * pi / 8) { // Vertical
                for(int t = j - size / 2; t - size < j - size / 2; t++) {
                    if (t < 0 || t >= input.cols) continue;
                    maxP = maxP < input.at<Vec2f>(i, t)[0] ? input.at<Vec2f>(i, t)[0] : maxP;
                }
            } else if(phi > 0) { //+45 degree
                for(int k = -size / 2; k - (-size / 2) < size; k++) {
                    if(i + k < 0 || i + k >= input.rows || j + k < 0 || j + k >= input.cols) continue;
                    maxP = maxP < input.at<Vec2f>(i + k, j + k)[0] ? input.at<Vec2f>(i + k, j + k)[0] : maxP;

                }
            } else {	// -45 degree
                for(int k = -size / 2; k - (-size / 2) < size; k++) {
                    if(i + k < 0 || i + k >= input.rows || j - k < 0 || j - k >= input.cols) continue;
                    maxP = maxP < input.at<Vec2f>(i + k, j - k)[0] ? input.at<Vec2f>(i + k, j - k)[0] : maxP;
                }
          	}
            float &outij = output.at<Vec2f>(i, j)[0];
            outij = abs(outij - maxP) < 10 ? outij  : 0;
        }
	std::vector<Mat> chs;
	split(output, chs);
	output = chs[0].clone();
    return ;
}

// Delay Thresholding
void delayThsh(Mat input, Mat & output, int tH, float tL, int size) {
    input.copyTo(output);
    tL = tH / tL;
    for (int i = 0; i < input.rows; i++)
        for (int j = 0; j < input.cols; j++) {
            if (input.at<float>(i, j) < tL)
                output.at<float>(i, j) = 0;
            else if (input.at<float>(i, j) < tH) {
                bool find = false;
                for (int s = i - size/2; s-i+size/2 < size && !find; s++)
                    for (int t = j - size/2; t-j+size/2 < size && !find; t++) {
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
void CannyFilt(Mat input, Mat & output) {
    input.copyTo(output);

    Mat grd;
    Mat sobelX = (Mat_<float>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
    Mat sobelY = (Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
    // Get gradient (with Sobel operator)
    grd = calcGrdnt(input, sobelX, sobelY);
    std::vector<Mat> chs;
	split(grd, chs);
	chs[0].convertTo(chs[0],CV_8U);
	//imshow("Gradient", chs[0]);
	normalize(chs[1], chs[1], 0, 255, NORM_MINMAX);
	chs[1].convertTo(chs[1],CV_8U);
	//imshow("Phi", chs[1]);
    // Non-maximum suppression
    nmS(grd, grd, 3);
    // Delay Thresholding
    delayThsh(grd, grd, 200, 2, 5);
     //Normalize to [0, 255]
    normalize(grd, grd, 0, 255, NORM_MINMAX);
    grd.convertTo(output, CV_8U);
}

// Line detection via Hough Transform
void HoughLine(Mat input, Mat & output) {
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
                    //    if(input.at<uchar>(x, y) < 250) continue;
                    output.at<uchar>(x, y) = 255;
                }
            } else {
                int y = (int)((rhos[i] - x * cos(pi / 2 / thetaMax * thetas[i])) / sin(pi / 2 / thetaMax * thetas[i]) + 0.5);
                if(y > 0 && y < output.cols) {
                    //    if(input.at<uchar>(x, y) < 250) continue;
                    output.at<uchar>(x, y) = 255;
                }
            }
        }
    }
}

// Circle Detection via Hough Transform
void HoughCirc(Mat input, Mat & output) {
    output = Mat(input.rows, input.cols, CV_8U, Scalar(0));
    //int rMax = (int)(sqrt(sqr(input.rows) + sqr(input.cols)) + 0.5);
    int rMax = 150;
    int aMax = 200;
    int bMax = 200;
    int threshold = 400;
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
                    if(r >= rMax) continue;
                    ++argCnt.at(r).at(a).at(b);
                    if(argFlag[r][a][b] == 0 && argCnt[r][a][b] > threshold) {
                        as.push_back(a);
                        bs.push_back(b);
                        rs.push_back(r);
                        argFlag.at(r).at(a).at(b) = 1;
                    }
                }
        }
    printf("%d\n", rs.size());
    for(int i = 0; i < rs.size(); i++ ) {
        printf("r=%d  a=%d  b=%d\n", rs[i], as[i], bs[i]);
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

void myHoughCircles(const Mat& input,Mat& output,int r_max, int r_min,int d_r,int d_c,int threshold1,int threshold2){
    //double  time=getTickCount();
    /*
     * 遇到一个神奇的问题...创建scoretable[rows][cols][r_num]的时候会是的输入的参数input变成空的...这是什么情况,而且抛出段错误139,11
     * = =使用圆变换计算量爆炸...
     * 源码中用shift将浮点进行整数计算后shift回去的技巧可以学习一下
     *   if( (unsigned)x2 >= (unsigned)acols ||
                        (unsigned)y2 >= (unsigned)arows ) 这是一个判断是越界的简便方法。因为小于0后就会变大。显然大于arows，但是以有符号型来创建该两个变量
     */




    int rows = input.rows,  cols = input.cols,   vote[rows][cols],  dx[rows][cols],  dy[rows][cols];
    normalize(input, output, 0, 1, CV_MINMAX);
    cvtColor(output,output,CV_GRAY2BGR);
    Mat blur,visable_vote=Mat(rows,cols,CV_8UC1,Scalar::all(0));


    //GaussianBlur(input,blur,Size(1,1),1,1);

    for (int i = 0; i <rows ; ++i) {
        for (int j = 0; j < cols; ++j) {
            dx[i][j]=dy[i][j]=vote[i][j]=0;
        }
    }
    for(int i=1;i<rows-1;++i){
        for (int j = 1; j < cols-1; ++j){
            dx[i][j]=(
                    - 3*blur.ptr(i-1)[j-1] + 3*blur.ptr(i-1)[j+1]
                    - 10*blur.ptr(i)[j-1] + 10*blur.ptr(i)[j+1]
                    - 3*blur.ptr(i+1)[j-1] + 3*blur.ptr(i+1)[j+1]);
            dy[i][j]=(
                    - 3*blur.ptr(i-1)[j-1] - 10*blur.ptr(i-1)[j] - 3*blur.ptr(i-1)[j+1]
                    + 3*blur.ptr(i+1)[j-1] + 10*blur.ptr(i+1)[j] + 3*blur.ptr(i+1)[j+1]);
//            dx[i][j]=(
//                    - blur.ptr(i-1)[j-1] + blur.ptr(i-1)[j+1]
//                    - 2*blur.ptr(i)[j-1] + 2*blur.ptr(i)[j+1]
//                    - blur.ptr(i+1)[j-1] + blur.ptr(i+1)[j+1]);
//            dy[i][j]=(
//                    - blur.ptr(i-1)[j-1] - 2*blur.ptr(i-1)[j] - blur.ptr(i-1)[j+1]
//                    + blur.ptr(i+1)[j-1] + 2*blur.ptr(i+1)[j] + blur.ptr(i+1)[j+1]);
        }
    }
    vector<Point> edge;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {

            int vx,vy;

            vx=dx[i][j];
            vy=dy[i][j];
            if((!input.ptr(i)[j]) || ( vx==0 && vy==0 ) )
                continue;
            float magnitude=sqrtf(vx*vx+vy*vy);
            int slight_x,slight_y;
            slight_x= cvRound((vx<<10)/magnitude);
            slight_y= cvRound((vy<<10)/magnitude);
            int x0=j<<10,y0=i<<10,x1,y1;
            for (int k = 0; k < 2; ++k) {
                x1=x0+r_min*slight_x;
                y1=y0+r_min*slight_y;
                for (int l = r_min; l <=r_max ;x1+=slight_x,y1+=slight_y,++l) {
                    int x2=x1>>10,y2=y1>>10;
                    if( (unsigned)x2 >= (unsigned)cols || (unsigned)y2 >= rows) {
                        break;
                    }
                    else{
                        ++vote[y2][x2];
                        visable_vote.ptr(y2)[x2]+=10;
                    }
                }
                slight_x*=-1;
                slight_y*=-1;
            }
            edge.push_back(Point(j,i));
        }
    }
    if(edge.size()==0)
        return;

    vector<Vec3i> center;
    for (int i = 1; i < rows-1; ++i) {
        for (int j = 1; j < cols-1; ++j) {
            if(vote[i][j]>threshold1
            && vote[i][j]>vote[i][j-1] && vote[i][j]>vote[i][j+1]
            && vote[i][j]>vote[i-1][j] && vote[i][j]>vote[i+1][j])
                center.push_back(Vec3i(j,i,vote[i][j]));
            /*
//            int flag=0;
//            if(vote[i][j]>threshold)
//                for (int k = -1; k < 2; ++k) {
//                    for (int l = -1; l < 2; ++l) {
//                        if(flag<vote[i+k][j+l])
//                            flag=vote[i+k][j+l];
//                    }
//                }
//            else
//                continue;
//            if(flag==vote[i][j])
//                center.push_back(Point(j,i));
*/
        }
    }
    if(center.size()==0)
        return;

    SelectionSort(center);

    float _x,_y,_r_2,r_max_2,r_min_2;
    vector<Point3i> CIRCLES;

    r_max_2=r_max*r_max;
    r_min_2=r_min*r_min;
    int j=0;
    for (int eachcenter = 0; eachcenter < center.size(); ++eachcenter) {
        vector<Vec3i> extimate_radius;


        for( j = 0; j < CIRCLES.size(); j++ )
        {
            Point3i c=CIRCLES[j];
            if( (c.x - center[eachcenter][0])*(c.x - center[eachcenter][0]) + (c.y - center[eachcenter][1])*(c.y - center[eachcenter][1]) < d_c*d_c )
                break;
        }
        if( j < CIRCLES.size())
            continue;

        for (int eachedge = 0; eachedge < edge.size(); ++eachedge) {
            _x=center[eachcenter][0]-edge[eachedge].x;
            _y=center[eachcenter][1]-edge[eachedge].y;
            _r_2=_x*_x+_y*_y;

            if(_r_2>r_min_2 && _r_2<r_max_2)
            {
                extimate_radius.push_back(Vec3i(0,0,sqrtf(_r_2)));
            }
        }
        if(extimate_radius.size()==0)
            continue;

        int num_r=extimate_radius.size(), start_idx,start_r, best_r, best_r_num, this_radius;

        start_r=best_r=extimate_radius[num_r-1][2];
        start_idx=num_r-1;
        best_r_num=1;


        SelectionSort(extimate_radius);

        for (int eachr = num_r-1; eachr >=0 ; --eachr) {
            this_radius=extimate_radius[eachr][2];
            if(this_radius  -   start_r  >  d_r) //d_r圆环分辨率，将相近的圆环视为1个圆。
            {
                float cur_r=extimate_radius[ (eachr+start_idx)/2][2];
                if( (start_idx-eachr)*best_r >=  best_r_num * cur_r ) //判断最佳圆半径准则---越大的圆环需要越多的点才能认为是圆，如果从大到小遍历半径这里是<=
                {
                    best_r=cur_r;
                    best_r_num=start_idx-eachr;
                }
                start_idx=eachr;
                start_r=this_radius;
            }
        }
        if(best_r_num >threshold2)
            CIRCLES.push_back(Point3i(center[eachcenter][0],center[eachcenter][1],best_r));
    }

//    for (int m = 0; m < center.size(); ++m) {
//        cout<<"x:"<<center[m][0]<<" y:"<<center[m][1]<<" vote: "<<center[m][2]<<endl;
//    }
    cout<<"/*** in circle ***/"<<endl;
    for (int m = 0; m < CIRCLES.size(); ++m) {
        cout<<"x:"<<CIRCLES[m].x<<" y:"<<CIRCLES[m].y<<" r: "<<CIRCLES[m].z<<endl;
        circle(output,Point(CIRCLES[m].x,CIRCLES[m].y),CIRCLES[m].z,Scalar(0,0,255),3);
        line(output,Point(CIRCLES[m].x,CIRCLES[m].y),Point(CIRCLES[m].x,CIRCLES[m].y),Scalar(0,255,0),3);
    }

    imshow("vote",visable_vote);
    imshow("circle",output);
    //cout<<"FPS:"<<getTickFrequency()/(getTickCount()-time)<<"\n"<<endl;
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
        frIn = imread("/home/zdh/Exp3/src/img_seg/test.png");
        imshow("Original", frIn);
// 转化成灰度图
        cvtColor(frIn, frIn, CV_BGR2GRAY);
        imshow("Convert to Gray", frIn);
// 高斯平滑处理
        Gaussian(frIn, frIn, 3, 3);
        imshow("Spatial Gaussian", frIn);
// 二值化
//        binaryzation(frIn, frIn);
//        imshow("Binarized",  frIn);
// Canny边缘检测
        Mat bndry = frIn.clone();
        CannyFilt(frIn, bndry);
        //binaryzation(bndry, bndry);
        imshow("Boundaries", bndry);
        Mat canny ;
        Canny(frIn, canny, 100, 200);
        imshow("OpenCV Canny", canny);
// Hough线检测
        Mat detect;
	    //HoughLine(bndry, detect);
        //imshow("Line Detection", detect);
// Hough圆检测
        HoughCirc(canny, detect);
        imshow("Circle Detection", detect);
		//myHoughCircles(bndry,detect,150,20,2,10,30,20);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}

