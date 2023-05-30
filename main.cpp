// title : linetracer
// date : 21.11.17, 23.5.12 updated
// author : sungryul lee

#include <iostream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include "dxl.hpp"
#include "vision.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main()
{
	string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12 ! \
			nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! \
			video/x-raw, format=(string)BGR !appsink";
	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse !\
			rtph264pay pt=96 ! udpsink host=203.234.58.121 port=8001 sync=false";
	string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse !\
			rtph264pay pt=96 ! udpsink host=203.234.58.121 port=8002 sync=false";
    
    // host는 데이터를 수신하는 컴퓨터의 ip주소를 적어준다.
    
    struct timeval start,end1,end2,end3;
    double diff1,diff2,diff3;
    
    VideoCapture source(src, CAP_GSTREAMER); 
    if (!source.isOpened()){ cout << "Video error" << endl; return -1; }
    VideoWriter writer1(dst1, 0, (double)30, cv::Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl;return -1;}
	VideoWriter writer2(dst2, 0, (double)30, cv::Size(640, 90), true);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl;return -1;}
	
	cv::Mat frame, res, dst;
	Dxl dxl;
	Linetracer lt;
	int leftvel,rightvel;
	bool mode=false;
	char ch;
	signal(SIGINT, ctrlc); 				//시그널 핸들러 지정
	if (!dxl.open()){ cout << "dxl open error" << endl; return -1; }

	while (true)
	{
		gettimeofday(&start,NULL);
		source >> frame;
		if (frame.empty()){ cout << "frame empty" << endl; return -1; }

		Mat roi = frame(Rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4));
		//writer2 << roi;
		lt.detectLine(roi, res);
		//lt.detectLines(roi, res);
		//cout << "error: " << lt.getError() << endl;
		
		leftvel = 200+0.3*lt.getError();
		rightvel = -(200-0.3*lt.getError());
		//leftvel = 100+0.15*lt.getError();
		//rightvel = -(100-0.15*lt.getError());		

		if(mode) dxl.setVelocity(leftvel, rightvel);
		//dxl.setVelocity(100+0.15*lt.getError(), -(100-0.15*lt.getError()));
		//dxl.setVelocity(50+0.12*lt.getError(), -(50-0.12*lt.getError()));
		gettimeofday(&end1,NULL);
				
		//cv::imshow("frame", frame);
		//cv::imshow("roi", roi);
		cv::imshow("res",res);	
		//vconcat(roi,res,dst);
		//cv::imshow("dst", dst);
		writer1 << frame;
		writer2 << res;
		
		gettimeofday(&end2,NULL);

		//cv::waitKey(2);					
		if(dxl.kbhit())
		{ 
			ch = dxl.getch();
			if(ch == 'q') break;
			else if(ch == 's') mode = true;			
		}
		if (ctrl_c_pressed) break;        
		//usleep(5000);
		gettimeofday(&end3,NULL);
		
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
		diff2 = end2.tv_sec + end2.tv_usec / 1000000.0 - end1.tv_sec - end1.tv_usec / 1000000.0;
		diff3 = end3.tv_sec + end3.tv_usec / 1000000.0 - end2.tv_sec - end2.tv_usec / 1000000.0;
		//cout  << "period1: " << diff1 << endl;
		//cout  << "period2: " << diff2 << endl;
		//cout  << "period3: " << diff3 << endl;
		cout << "err:" << lt.getError() <<",lvel:"<<leftvel <<",rvel:"<<rightvel<<",time:" <<diff1+diff2+diff3 << endl;
				
	}

	dxl.close();
	return 0;
}

