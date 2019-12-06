#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;
int HBmax = 126, HBmin = 88, Smax = 255, Smin = 45, HGmax = 80, HGmin = 48, HYmax = 32, HYmin = 27, HRmax = 179, HRmin = 164;
static void onChange(int, void*)
{
	;
}

int main()
{
	VideoCapture capture;
	capture.open(0);
	while (waitKey(1)) {
		Mat raw;
		capture.read(raw);
		//Mat raw=imread("../data/data.jpeg");
		resize(raw, raw, Size(300, 300));
		GaussianBlur(raw, raw, Size(3, 3), 1, 1);
		Mat hsvimg, thresholdimgB, thresholdimgG, thresholdimgY, thresholdimgR;
		cvtColor(raw, hsvimg, CV_BGR2HSV);
		imshow("hsvimg", hsvimg);
		createTrackbar("Hmax", "hsvimg", &HGmax, 179, onChange, 0);
		createTrackbar("Hmin", "hsvimg", &HGmin, 179, onChange, 0);
		createTrackbar("Smax", "hsvimg", &Smax, 255, onChange, 0);
		createTrackbar("Smin", "hsvimg", &Smin, 255, onChange, 0);


		vector<vector<Point> > contours;
		vector<Vec4i> h;

		inRange(hsvimg, Scalar(HGmin, Smin, 46), Scalar(HGmax, Smax, 255), thresholdimgG);
		inRange(hsvimg, Scalar(HYmin, Smin, 46), Scalar(HYmax, Smax, 255), thresholdimgY);
		inRange(hsvimg, Scalar(HBmin, Smin, 46), Scalar(HBmax, Smax, 255), thresholdimgB);
		inRange(hsvimg, Scalar(HRmin, Smin, 46), Scalar(HRmax, Smax, 255), thresholdimgR);

		findContours(thresholdimgG, contours, h, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		if (h.size() == 0) continue;
		int idx = 0;
		for ( ; idx >= 0; idx = h[idx][0] ) {
			Scalar color( 255, 255, 255 );
			drawContours( raw, contours, idx, color, CV_FILLED, 8, h );
		}
		imshow("Green", thresholdimgG);
		imshow("Yellow", thresholdimgY);
		imshow("Blue", thresholdimgB);
		imshow("Red", thresholdimgR);
		imshow("rawimg", raw);
		waitKey(1);
	}
	return 0;
}
