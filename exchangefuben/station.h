/*****************************************************************//**
 * \file   station.h
 * \author YYY
 * \date   March.9 2023
 *********************************************************************/

#pragma once
#ifndef STATION
#define STATION

#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<iostream>

using namespace cv;
using namespace std;

typedef struct
{
    Point left_up;           //左上角的点
    Point right_down;        //右上角的点
    Point left_down;         //左下角的点
    Point right_up;          //右上角点
    Point center;            //中心点
}exchangeStation;

class ExchangeStation
{
public:
    /*
    * @brief:分通道，二值化，开闭运算
    * @Input:src 原图
    * @Output:输出图
    */
    void preTreatment(Mat src,Mat &out);

    /*
    * @brief:在rect中找到x,y的最大最小值
    * @Input: mod=1取最小值，其他是最大值。
    * @Output:lim是输出的最小值
    */
    void getCenterPointX(int& lim, vector<Rect> rect,int mod);
    void getCenterPointY(int& lim, vector<Rect> rect, int mod);

    void mergeRectangles(const std::vector<cv::Rect>& input_rects, std::vector<cv::Rect>& output_merged_rects, std::vector<cv::Rect>& output_isolated_rects, float overlap_threshold);
    /*
    * @brief:主要的识别函数
    * @Input:src输入的图片，origin是用来debug的
    * @Return:没啥意义
    */
    int getPoint(Mat & src, Mat origin);

    /*
    * @brief:测两点距离
    * @Input:要测的两点
    * @Return:两点的距离
    */
    int getDistance(Point a, Point b);

    /*
    * @brief:为了能够得到更好的右上角
    * @Input:
    *		better_right_on_point：Getdistance（）得到的右上角，
    *		left_down：得到一个框框的长宽
    *		src： 图片
    * @Return:没啥意义
    */
    int betterJudge(Mat image, RotatedRect left_up, RotatedRect right_down, RotatedRect left_down, Point better_right_on_point );

    /*show show need!!!*/
    void check(Mat src, exchangeStation* exchangeStation);

    void  judgeStation(Mat src);

    bool isRectOutOfMat( const cv::Mat& mat,const cv::Rect& rect);

    RotatedRect left_up;           //左上角的矩形
    RotatedRect right_down;        //右下角的矩形
    RotatedRect left_down;         //左下角的矩形
    Point better_right_on_point;   //更好的右上角
    vector<vector<Point>> contours;//捕捉到的轮廓
    vector<Vec4i> hierarchy;       //画图用的
    Point station_center;          //兑换站中心点
    vector<RotatedRect> pre_rect;  //捕捉到的轮廓的外接矩形
    vector <int> distances;        //一些距离
    vector<Point>corners;          //一些角点
    exchangeStation exchangeStation1;
    exchangeStation* pExchangeStation = &exchangeStation1;
    Mat out;
};

#endif
