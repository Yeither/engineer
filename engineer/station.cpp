#include"station.h"

/*****************************************************************//**
 * \file   station.cpp
 * \author YYY
 * \date   March.9 2023
 * \brief  兑换站识别
 *********************************************************************/


void ExchangeStation::preTreatment(Mat src, Mat &out)
{
    Mat out1,out2;
    out.copyTo(out1);
    out.copyTo(out2);
    cv::Mat image1(src.size(), CV_32FC3);
    cv::Mat image2(src.size(), CV_32FC3);
    cvtColor(src, image1, COLOR_BGR2HSV);  // 调用内置函数
    cvtColor(src, image2, COLOR_BGR2HSV);  // 调用内置函数
    Scalar hsvRedLo( 0,  40,  40);
    Scalar hsvRedHi(10, 255, 255);
    Scalar hsvBlueLo(100,  40,  40);
    Scalar hsvBlueHi(140, 255, 255);
    inRange(image1, hsvBlueLo, hsvBlueHi, image1);
    inRange(image2, hsvRedLo, hsvRedHi, image2);
    threshold(image1, out1, 1, 255, THRESH_BINARY);
    threshold(image2, out2, 1, 255, THRESH_BINARY);
    //imshow("out1", out1);// waitKey(0);
    //imshow("out2", out2);// waitKey(0);
    Mat notout1,notout2;
    out.copyTo(notout1);
    out.copyTo(notout2);
    cv::bitwise_not(out1,notout1);
    cv::bitwise_not(out2,notout2);
    Mat out12,notout12;
    multiply(notout1,notout2,notout12);
    multiply(out1,out2,out12);
    out= out12+notout12;
    cv::bitwise_not(out,out);

    /*
    vector<Mat> mv;
    split(image, mv);
    image = mv[0] - mv[2];
    cv::boxFilter(image,out,-1,Size(3, 3),Point(-1,-1), true,BORDER_DEFAULT);
    threshold(image, image, 300, 255, THRESH_OTSU);*/
    //获取自定义核
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(out, out, element);
    //erode(image, out, element);
    Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(out, out, element2);  dilate(out, out, element2);
    imshow("", out);// waitKey(0);

}

void ExchangeStation::getCenterPointX(int & lim, vector<Rect> rect,int mod)
{
    Rect temp;
    if (rect.size() > 0)
    {
        for (int i = 0; i < (rect.size() - 1); i++)
        {
            for (int j = (rect.size() - 1); j > i; j--)
            {
                if (rect[i].x > rect[j].x)
                {
                    temp = rect[j];
                    rect[j] = rect[i];
                    rect[i] = temp;
                }
            }
        }
        if (mod == 1)
        {
            lim = rect[0].x;
        }
        else
        {
            lim = rect[rect.size() - 1].x;
        }
    }
    else
    {
        return;
    }
}

void ExchangeStation::getCenterPointY(int& lim, vector<Rect> rect, int mod)
{
    Rect temp;
    if (rect.size() > 0)
    {
        for (int i = 0; i < (rect.size() - 1); i++)
        {
            for (int j = (rect.size() - 1); j > i; j--)
            {
                if (rect[i].y > rect[j].y)
                {
                    temp = rect[j];
                    rect[j] = rect[i];
                    rect[i] = temp;
                }
            }
        }
        if (mod == 1)
        {
            lim = rect[0].y;
        }
        else
        {
            lim = rect[rect.size() - 1].y;
        }
    }
}

int ExchangeStation::getDistance(Point a, Point b)
{
    int distance = pow(((a.x - b.x) * (a.x - b.x) + (b.y - a.y) * (b.y - a.y)), 0.5);
    return distance;
}

void ExchangeStation::mergeRectangles(const std::vector<cv::Rect>& input_rects, std::vector<cv::Rect>& output_merged_rects, std::vector<cv::Rect>& output_isolated_rects, float overlap_threshold)
{
    // 进行 NMS 合并
    std::vector<int> indices;
    int size = input_rects.size();
    // 创建一个与已知向量相同大小的新向量
    std::vector<float> newVector(size);
    cv::dnn::NMSBoxes(input_rects, newVector, 0.0f, overlap_threshold, indices);

    // 将合并后的矩形添加到输出向量中
    output_merged_rects.reserve(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        output_merged_rects.push_back(input_rects[indices[i]]);
    }

    // 找出未被合并的孤立矩形
    std::set<int> merged_indices(indices.begin(), indices.end());
    output_isolated_rects.reserve(input_rects.size() - merged_indices.size());
    for (int i = 0; i < input_rects.size(); ++i) {
        if (merged_indices.find(i) == merged_indices.end()) {
            output_isolated_rects.push_back(input_rects[i]);
        }
    }
}

int ExchangeStation::getPoint(Mat & src,Mat origin)
{
    //找边缘点
    findContours(src, contours, hierarchy, RETR_EXTERNAL, 4);
    /*
    Mat imageContours1 = Mat::zeros(src.size(), CV_8UC1);
    Mat Contours = Mat::zeros(src.size(), CV_8UC1);  //绘制
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            Point P = Point(contours[i][j].x, contours[i][j].y);
        }
        drawContours(imageContours1, contours, i, Scalar(255,255,0), 2, 8, hierarchy);
        imshow("Contours Image", imageContours1); waitKey(0);
    }
    imshow("Contours Image", imageContours1); waitKey(0);
    */
    //找最小矩形
    vector<RotatedRect> rect;
    vector<int> real;
    for (int i = 0; i < contours.size(); i++)
    {
        if ((contours[i].size()) > 5)
        {
            pre_rect.push_back(minAreaRect(contours[i]));
        }
        else
        {
            continue;
        }
    }

        for (int i = 0; i < pre_rect.size(); i++)
    {
        Mat imageContours3 = Mat::zeros(src.size(), CV_8UC3);
        Point2f vertices[4];
        for (size_t i = 0; i < pre_rect.size(); i++)
        {
            pre_rect[i].points(vertices);
            for (int j = 0; j < 4; j++)
                line(imageContours3, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
            //imshow("imageContours3", imageContours3); waitKey(0);
        }
        imageContours3 = imageContours3 + origin;
        imshow("imageContours3", imageContours3); //waitKey(0);
    }

    //筛选矩形
    real.reserve(pre_rect.size());
    for (int i = 0; i < pre_rect.size(); i++)
    {
        for (int j = 0; j < pre_rect.size() ; j++)
        {
            if ((pre_rect[i].center.x - pre_rect[j].center.x < 30) &&
                (pre_rect[i].center.x - pre_rect[j].center.x > -30))
            {
                if((pre_rect[i].size.height/pre_rect[i].size.width < 7)
                   && (pre_rect[i].size.area() + pre_rect[j].size.area() > 500))
                {
                    real[i] = 1;
                }
                else
                {
                    continue;
                }
            }
        }
    }
    //筛选矩形2
    for (auto i = real.size(); i > 0; i--)
    {
        if (real[i] != 0)
        {
            for (int j = 0; j < pre_rect.size(); j++)
            {
                if ((pre_rect[i].center.x - pre_rect[j].center.x < 3) && (pre_rect[i].center.x - pre_rect[j].center.x > -3))
                {
                    real[i] = 1;
                }
                else
                {
                    continue;
                }
            }
        }
    }
    std::vector<cv::Rect> output_merged_rects; std::vector<cv::Rect> output_isolated_rects;std::vector<cv::Rect> input_rect;
    for (int i = 0; i < pre_rect.size(); i++)
    {
        if ((pre_rect[i].size.area()>85)&&((pre_rect[i].size.area() < 9000))&&(real[i]))
            //if(1)
        {
            rect.push_back(pre_rect[i]);
        }
        else
        {
            continue;
        }
        for (int j = 0; j < pre_rect.size(); ++j)
        {
            input_rect.push_back(pre_rect[i].boundingRect2f());
        }
    }
    mergeRectangles(input_rect, output_merged_rects, output_isolated_rects, 0.1);
    for (int j = 0; j < output_isolated_rects.size(); ++j)
    {
        output_merged_rects.push_back(output_isolated_rects[j]);
    }
    if (output_merged_rects.size() < 4)
    {
        return 0;
    }
    /*
        for (int i = 0; i < rect.size(); i++)
    {
        Mat imageContours3 = Mat::zeros(src.size(), CV_8UC3);
        Point2f vertices[4];
        for (size_t i = 0; i < rect.size(); i++)
        {
        rect[i].points(vertices);
        for (int j = 0; j < 4; j++)
            line(imageContours3, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
        imshow("imageContours3", imageContours3); waitKey(0);
        }
        imageContours3 = imageContours3 + origin;
        imshow("imageContours3", imageContours3); waitKey(0);
    }
    */
    //找到兑换站中心点
    int rect_x_max, rect_x_min;
    int rect_y_max, rect_y_min;
    int center_x, center_y;
    ExchangeStation::getCenterPointX(rect_x_min, output_merged_rects, 1);
    ExchangeStation::getCenterPointX(rect_x_max, output_merged_rects, 0);
    ExchangeStation::getCenterPointY(rect_y_min, output_merged_rects, 1);
    ExchangeStation::getCenterPointY(rect_y_max, output_merged_rects, 0);
    center_x = 0.5 * (rect_x_max + rect_x_min);
    center_y = 0.5 * (rect_y_max + rect_y_min);
    vector<RotatedRect> right_up;
    RotatedRect right_on;

    //通过中心点筛选四个角的矩形
    for (int i = 0; i <= rect.size()-1; i++)
    {
        if ((rect[i].center.x < center_x) && (rect[i].center.y < center_y))
        {
            left_up = rect[i];
            continue;
        }
        else if ((rect[i].center.x < center_x) && (rect[i].center.y > center_y))
        {
            left_down = rect[i];
            continue;
        }
        else if ((rect[i].center.x > center_x) && (rect[i].center.y > center_y))
        {
            right_down = rect[i];
            continue;
        }
        else
        {
            right_up.push_back(rect[i]);
            continue;
        }
    }

    //判断四个角点
    if (right_up.size() > 1)
    {
        RotatedRect temp;
        for (int i = 0; i < right_up.size(); i++)
        {
            for (int j = i; j < right_up.size(); j++)
            {
                if (right_up[i].center.x > right_up[j].center.x)
                {
                    temp = right_up[i];
                    right_up[i] = right_up[j];
                    right_up[j] = temp;
                }
            }
        }

        /*
                Mat imageContours3 = Mat::zeros(src.size(), CV_8UC3);
        Point2f vertices[4];
            right_on.points(vertices);
            for (int j = 0; j < 4; j++)
            line(imageContours3, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
            imageContours3 = imageContours3 + origin;
        imshow("imageContours3", imageContours3); waitKey(0);
        */
        //求两个小灯的中心点估算右上角的点
        right_on = right_up[right_up.size()/2];
        better_right_on_point.x = (right_up[0].center.x + right_up[right_up.size() - 1].center.x) / 2;
        better_right_on_point.y = (right_up[0].center.y + right_up[right_up.size() - 1].center.y) / 2;
    }
        //右上角的矩形数量不够
    else
    {
        right_on.center = right_down.center + left_up.center - left_down.center;
        right_on.angle = 90.0;
        right_on.size = left_down.size;
        better_right_on_point = right_on.center;
    }
    //筛选右上角的点
    station_center = ((left_up.center) + (right_down.center)) * 0.5;
    /*Mat imageContours = Mat::zeros(src.size(), CV_8UC3);
    line(imageContours, better_right_on_point, right_down.center, Scalar(255, 255, 0), 3, LINE_8, 0);
    line(imageContours, right_down.center, left_down.center, Scalar(255, 255, 0), 3, LINE_8, 0);
    line(imageContours, left_down.center, left_up.center, Scalar(255, 255, 0), 3, LINE_8, 0);
    line(imageContours, left_up.center, better_right_on_point, Scalar(255, 255, 0), 3, LINE_8, 0);
    imageContours = imageContours + origin;
    imshow("imageContours", imageContours); waitKey(0);*/
    return 1;
}

int ExchangeStation::betterJudge(Mat image, RotatedRect left_up, RotatedRect right_down, RotatedRect left_down, Point better_right_on_point )
{
    int length_add = left_down.size.width * 1.3;
    int height_add = left_down.size.height * 1.3;
    int up = better_right_on_point.y + height_add;
    int down = better_right_on_point.y - height_add;
    int right = better_right_on_point.x + length_add;
    int left = better_right_on_point.x - length_add;
    if (up > src.rows)
    {
        up = src.rows;
    }
    if (down < 0)
    {
        down = 0;
    }
    if (right > src.cols)
    {
        right > src.cols;
    }
    if (left < 0)
    {
        left = 0;
    }

    Rect mask;
    mask.height = 2 * height_add;
    mask.width = 2 * length_add;
    mask.x = better_right_on_point.x - length_add;
    mask.y = better_right_on_point.y - height_add;
    Rect station[4] = { left_up.boundingRect(), right_down.boundingRect() , left_down.boundingRect() , mask };
    Mat ROI_img1(src, left_up.boundingRect());
    Mat ROI_img2(src, right_down.boundingRect());
    Mat ROI_img3(src, left_down.boundingRect());
    Mat ROI_img4(src, mask);
    Mat roi[4] = { ROI_img1 , ROI_img2 , ROI_img3 , ROI_img4 };
    for (int i = 0; i < 4; i++)
    {
        cv::goodFeaturesToTrack(image, corners, 1, 0.8, 1, roi[i]);
    }
}


void ExchangeStation::check(Mat src, exchangeStation* exchangeStation)
{
    if (((better_right_on_point.x - left_up.center.x) / (-better_right_on_point.y + right_down.center.y) > 1.5) ||
        ((better_right_on_point.x - left_up.center.x) / (-better_right_on_point.y + right_down.center.y) < 0.5))
    {
        cout << "can not find!";
        return;
    }

    if(getDistance(exchangeStation->left_down,left_down.center)<10||
       getDistance(exchangeStation->left_up,left_up.center)<10 ||
       getDistance(exchangeStation->right_down,right_down.center)<10)
    {
         ;
    }
    else
    {
        exchangeStation->center.x = (right_down.center.x - left_down.center.x)*0.5;
        exchangeStation->center.y = (left_up.center.y - left_up.center.y) * 0.5;
        exchangeStation->left_down = left_down.center;
        exchangeStation->left_up = left_up.center;
        exchangeStation->right_down = right_down.center;
        exchangeStation->right_up = better_right_on_point;
    }

    //show show need
    Mat imageContours = Mat::zeros(src.size(), CV_8UC3);
    for (int i = 0; i < 4; i++)
    {
        line(imageContours, corners[i%4], corners[(i+1)%4], Scalar(255, 255, 0), 3, LINE_8, 0);
    }
    imageContours = imageContours + src;
    imshow("imageContours", imageContours);
    //imwrite("C:/Users/y8615/Desktop/imageContours2.jpg", imageContours);
}

void  ExchangeStation::judgeStation(Mat src)
{
    ExchangeStation::preTreatment(src, ExchangeStation::out);
    ExchangeStation::getPoint(ExchangeStation::out, src);
    ExchangeStation::betterJudge(out,left_up,right_down,left_down,better_right_on_point);
    ExchangeStation::check(src, ExchangeStation::pExchangeStation);

    pre_rect.clear();
    hierarchy.clear();
    contours.clear();
    distances.clear();
    corners.clear();
}
