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
    void Solver(const char* filePath, int camId, exchangeStation exchangeStation);
    void eulerAngle();

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

    float alpha, beta, gamma;//天性叔

    vector<Point3f> STATION_POINTS_3D;
    //Results
    float y_yaw;
    float x_pitch;
    double distance;
    double euler_y_;
    double GUN_CAM_DISTANCE_Y;
};
