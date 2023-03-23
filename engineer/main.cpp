#include "GxCamera/GxCamera.h"
#include<condition_variable>
#include<mutex>
#include<thread>
#include<vector>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include "station.h"
#include"AngleSolver.h"

using namespace cv;

int main()
{
    camera cam;
    ExchangeStation a;
    AngleSolver an;
    Mat src ,dst;
    double distance = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    cam.autoConnect();
    //cv::waitKey(1000);
    cam.startAcquiring();
    //cv::waitKey(1000);
    double K[9] = { 1290.864025394572, 0, 644.4408716202423,
                    0, 1285.638482830289, 531.8367613053998,
                    0, 0, 1 };
    double coeffs[5] = { -0.2453696281444591, 0.5105847268054721, -5.796001846910601e-05, -0.001719445214522469, -0.867509205599535 };


    cv::Mat camera_K = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_32FC1);


    while (1)
    {

        cam.getsrc(src);
        if (src.empty())
        {
            cout << "src empty\n";
            continue;
        }
        //Mat src= imread("/home/zhouhb/桌面/exchangefuben/4.png");
        src.copyTo(dst);
        a.judgeStation(dst);
        an.Solver("/home/zhouhb/桌面/exchangefuben/camera_params.xml", 1,a.exchangeStation1,dst,a.corners);
        //imshow("111", dst);
        waitKey(1);

    }
    return 0;
}

