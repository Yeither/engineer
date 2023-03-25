#pragma once
#include"station.h"

class AngleSolver
{
public:

    void setCameraParam(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeff);
    int setCameraParam(const char* filePath, int camId);
    void solveAngles(exchangeStation exchangeStation);
    void setStationSize(double width, double height);//240
    void PinHole_solver();
    void P4P_solver();
    void compensateAngle();
    void compensateOffset();
    void getAngle(exchangeStation exchangeStation, int& X, int& Y, int& Z);
    void showDebugInfo(bool showCurrentResult, bool ifWind, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams);
    void Solver(const char* filePath, int camId, exchangeStation exchangeStation,Mat image,std::vector<Point_<float>> object_corners);
    //void eulerAngle(cv::Mat image, std::vector<cv::Point2f> object_corners);
    //void get_3D_point(const cv::Mat& inputImage, const std::vector<cv::Point2f>& targetPoints, std::vector<cv::Point3d>& world_points);
    void pixel2world(const cv::Mat& camera_matrix, const std::vector<cv::Point2f>& pixel_points, std::vector<cv::Point3f>& world_points) ;
    void computePerpendicular(const vector<Point3f>& points, vector<double>* result);
    double cos_distance(Point pa,Point pb,Point pc);
    int getDistance(Point a, Point b);
    void my_soolver(vector<Point2f> armorVertices);
//Camera params
    Mat CAMERA_MATRIX;    //IntrinsicMatrix		  fx,fy,cx,cy
    Mat DISTORTION_COEFF; //DistortionCoefficients k1,k2,p1,p2
    //Targets
    vector<Point2f> targetContour;
    Point2f targetCenter;
    // calculated by solvePnP
    //s[R|t]=s'  s->world coordinate;s`->camera coordinate
    Mat rVec;    //rot rotation between camera and target center
    int X, Y, Z;
    Mat tVec;  //trans tanslation between camera and target center

    double yaw ;
    double pitch ;
    double roll ;
    std::vector<cv::Point3f> world_points;
    vector<Point3f> STATION_POINTS_3D;
    //Results
    float y_yaw;
    float x_pitch;
    double distance;
    double euler_y_;
    double GUN_CAM_DISTANCE_Y;
};
