#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


void kCanny(const Mat&src,Mat&output,int kernalsize,int sigma,int max,int min);
void kHoughLines (const Mat& input,Mat& output,double d_rho, double d_theta);
void kHoughCircles(const Mat& input,Mat& output,int r_max, int r_min,int d_r,int d_c,int threshold1,int threshold2);
/*查找表免去switch烦恼
 * i:非极大值抑制时候用的扇区
 * j:比较方向x1,y1,x2,y2对中心的位置偏移
 */
int T[4][4]={{0,-1,0,1},
             {-1,1,1,-1},
             {-1,0,1,0},
             {-1,-1,1,1}};
Mat raw;
int main(int argc, char *argv[])
{
//    VideoCapture capture;
//    capture.open(2);
//    while(waitKey(1))
//    {
        raw=imread("/home/kevin/workSpace/CV_ws/src/ex3/data/2.jpeg");
        //raw=imread("../data/lena.jpg");
//        capture.read(raw);
//        if(!raw.data)
//            break;

        imshow("raw",raw);
        Mat gray,circle_img,line_img;
        cvtColor(raw,gray,CV_BGR2GRAY);



//        /****************Canny****************/
//        kCanny(gray,gray,3,1,9,3);//Sobel用60-20
//        imshow("myCanny",gray);


        /***************霍夫线变换***************/
        kHoughLines(gray,line_img,1,CV_PI/180);
        imshow("line_img",raw);

//
//        /***************霍夫圆变换***************/
        kHoughCircles(gray,circle_img,150,120,2,10,20,20);
        waitKey(100000);
//    }

}

void kCanny(const Mat&src,Mat& output,int kernalsize,int sigma,int max,int min)
{
    double  time=getTickCount();
    GaussianBlur(src,src,Size(kernalsize,kernalsize),sigma,sigma);

    int rows=src.rows,cols=src.cols,temp;
    float g_x[rows][cols],g_y[rows][cols],mag[rows+1][cols+1],angle[rows][cols],data[rows][cols];
    for (int k = 0; k < rows; ++k)
        for (int i = 0; i < cols; ++i)
            data[k][i]=src.ptr(k)[i];

    for(int i=1;i<rows;++i){
        for (int j = 1; j < cols; ++j){
            /*
             * 2*2算子
             */
            g_x[i][j]=0.5*(
                    - data[i-1][j-1] + data[i-1][j]
                    - data[i][j-1]   + data[i][j]);
            g_y[i][j]=0.5*(
                    + data[i-1][j-1] + data[i-1][j]
                    - data[i][j-1]   - data[i][j]);
            /*
             * 3*3sobel算子
             */
//            g_x[i][j]=(
//                    - data[i-1][j-1] + data[i-1][j+1]
//                    - 2*data[i][j-1] + 2*data[i][j+1]
//                    - data[i+1][j-1] + data[i+1][j+1]);
//            g_y[i][j]=(
//                    + data[i-1][j-1] + 2*data[i-1][j] + data[i-1][j+1]
//                    - data[i+1][j-1] - 2*data[i+1][j] - data[i+1][j+1]);
            mag[i][j]=sqrtf(g_x[i][j]*g_x[i][j] + g_y[i][j]*g_y[i][j]);
            angle[i][j]=atan2f(g_y[i][j],g_x[i][j]);   //±pi →0
        }
    }

    for (int i = 1; i < rows; ++i){
        for (int j = 1; j < cols; ++j){
            g_x[i][j]=g_y[i][j]=0; //没用的数据清零再用。
            if(mag[i][j]<2)//值太小的默认不是边沿
                continue;
            temp= (int)( 4*(2+angle[i][j]/ CV_PI+ 1.0/8) )%4;
            if((mag[i][j] < mag[i+T[temp][0]][j+T[temp][1]]) || (mag[i][j]  <  mag[i+T[temp][2]][j+T[temp][3]])){
            } else{ //非最大值抑制的同时进行标记大阈值和小阈值的数据。
                if(mag[i][j]>max)
                    g_x[i][j]=1;
                else if(mag[i][j]>min)
                    g_y[i][j]=1;
            }
        }
    }
    output=Mat(rows,cols,CV_8UC1,Scalar::all(0));
    for (int i = 1; i < rows; ++i) {
        for (int j = 1; j < cols; ++j) {
            if(g_x[i][j]&&!g_y[i][j]) {
                output.ptr(i)[j]=255;
                for (int k = -1; k < 2; ++k) {
                    for (int l = -1; l < 2; ++l) {
                        if(g_x[i+k][j+l]!=0){
                            g_x[i][j]=1;
                            output.ptr(i)[j]=255;
                        }
                    }
                }
            }
        }
    }
//    cout<<"/*** in mycanny ***/"<<endl;
//    cout<<"FPS:"<<getTickFrequency()/(getTickCount()-time)<<"\n"<<endl;
}
void kHoughLines (const Mat& input,Mat& output,double d_rho, double d_theta)
{
    Canny(input,input,80,40);
    int rows=input.rows,cols=input.cols;
    int theta_num=(CV_PI/d_theta);                  //x轴刻度数
    int hr=sqrt(rows*rows+cols*cols)/d_rho-1;    //y轴刻度数
    int rho_num=2*hr+1;                            //y轴刻度数

    Mat scoretable(theta_num,rho_num,CV_32FC1,Scalar::all(0));

    for (int i = 0; i < rows; ++i){
        for (int j = 0; j <cols ; ++j){
            if(input.ptr(i)[j]){
                for (int k = 0; k < theta_num; ++k) {
                    scoretable.ptr<float>(k) [ cvRound(( (cos(k*d_theta)*i + sin(k*d_theta)*j) / d_rho) + hr)]+=1;
                }
            }
        }
    }
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;

    minMaxIdx(scoretable,minp,maxp);
//    output=Mat(rows,cols,CV_8UC1,Scalar::all(0));
    double a,b,c;
    Mat tmp = Mat(Size(rho_num,theta_num), CV_8UC3);
    for (int i = 0; i < theta_num; ++i) {
        for (int j = 0; j <rho_num ; ++j) {
            float r = pow((float)scoretable.ptr<float>(i)[j]/maxv,0.3)*255;

            float b = pow(((float)maxv-(float)scoretable.ptr<float>(i)[j])/maxv,3)*255;
            float g = (r+g)/4;
            if((float)scoretable.ptr<float>(i)[j]/maxv>0.965){r=255,b=255;g=255;}
            else if ((float)scoretable.ptr<float>(i)[j]/maxv<0.02){r=0,b=0;g=0;}
            tmp.at<Vec3b>(i,j)[0] = cvRound(b);
            tmp.at<Vec3b>(i,j)[1] = cvRound(g);
            tmp.at<Vec3b>(i,j)[2] = cvRound(r);

//            if(scoretable.ptr<float>(i)[j]>cvRound(maxv*0.99))
//            {
//                a=cos(i*d_theta);b=sin(i*d_theta);c=j*d_rho;
//                if(abs(a)<1e-3){
//                    line(raw,Point(0,rows-c),Point(cols,rows-c),Scalar(0,0,255));
//                }
//                else if(abs(b)<1e-3){
//                    line(raw,Point(c,rows-0),Point(c,rows-rows),Scalar(0,0,255));
//                }
//                else{
//                    line(raw,Point(cvRound(c/b),0),Point(cvRound(a/b),rows),Scalar(0,0,255));
//                }
//            }
            if(scoretable.ptr<float>(i)[j]>maxv*0.3)
            {
                a=cos(i*d_theta);b=sin(i*d_theta);c=(j-hr)*d_rho;
                if(1-abs(a)<1e-3){
                    line(raw,Point(0,c),Point(cols,c),Scalar(0,0,255));
                }else{
                    line(raw,Point(cvRound(c/b),0),Point(cvRound((c-rows*a)/b),rows),Scalar(0,0,255));
                }
            }
        }
    }


//    cout<<scoretable<<endl;
    normalize(scoretable,scoretable,1,0,NORM_MINMAX);
//    normalize(tmp,tmp,1,0,NORM_MINMAX);
    imshow("hot",tmp);
//    imshow("rho-theta",scoretable);
}

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
void kHoughCircles(const Mat& input,Mat& output,int r_max, int r_min,int d_r,int d_c,int threshold1,int threshold2)
{
    double  time=getTickCount();
    /*
     * 遇到一个神奇的问题...创建scoretable[rows][cols][r_num]的时候会是的输入的参数input变成空的...这是什么情况,而且抛出段错误139,11
     * = =使用圆变换计算量爆炸...
     * 源码中用shift将浮点进行整数计算后shift回去的技巧可以学习一下
     *   if( (unsigned)x2 >= (unsigned)acols ||
                        (unsigned)y2 >= (unsigned)arows ) 这是一个判断是越界的简便方法。因为小于0后就会变大。显然大于arows，但是以有符号型来创建该两个变量
     */




    int rows = input.rows,  cols = input.cols,   vote[rows][cols],  dx[rows][cols],  dy[rows][cols];
    cvtColor(input,output,CV_GRAY2BGR);
    Mat blur,visable_vote=Mat(rows,cols,CV_8UC1,Scalar::all(0));


    GaussianBlur(input,blur,Size(1,1),1,1);

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
    cout<<"FPS:"<<getTickFrequency()/(getTickCount()-time)<<"\n"<<endl;
}
