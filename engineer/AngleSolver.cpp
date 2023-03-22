#include"AngleSolver.h"

void AngleSolver::setCameraParam(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeff)
{
    camera_matrix.copyTo(CAMERA_MATRIX);
    distortion_coeff.copyTo(DISTORTION_COEFF);
}

//get camera_matrix  distortion_coeff
int AngleSolver::setCameraParam(const char* filePath, int camId)
{
    cv::FileStorage fsRead;
    fsRead.open(filePath, cv::FileStorage::READ);
    if (!fsRead.isOpened())
    {
        cout << "Failed to open xml" << endl;
        return -1;
    }

    //fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

    Mat camera_matrix;
    Mat distortion_coeff;
    switch (camId)
    {
        case 1:
            fsRead["CAMERA_MATRIX_1"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_1"] >> distortion_coeff;
            break;
        case 2:
            fsRead["CAMERA_MATRIX_2"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_2"] >> distortion_coeff;
            break;
        case 3:
            fsRead["CAMERA_MATRIX_3"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_3"] >> distortion_coeff;
            break;
        case 4:
            fsRead["CAMERA_MATRIX_4"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_4"] >> distortion_coeff;
            break;
        case 5:
            fsRead["CAMERA_MATRIX_5"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_5"] >> distortion_coeff;
            break;
        case 6:
            fsRead["CAMERA_MATRIX_6"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_6"] >> distortion_coeff;
            break;
        case 7:
            fsRead["CAMERA_MATRIX_7"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_7"] >> distortion_coeff;
            break;
        case 8:
            fsRead["CAMERA_MATRIX_8"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_8"] >> distortion_coeff;
            break;
        default:
            cout << "WRONG CAMID GIVEN!" << endl;
            break;
    }
    setCameraParam(camera_matrix, distortion_coeff);
    fsRead.release();
    return 0;
}

//get STATION_POINTS_3D
void AngleSolver::setStationSize( double width, double height)
{
    //240*240
    double half_x = width / 2.0;
    double half_y = height / 2.0;
    STATION_POINTS_3D.push_back(Point3f(-half_x, half_y, 0));   //tl top left
    STATION_POINTS_3D.push_back(Point3f(half_x, half_y, 0));	//tr top right
    STATION_POINTS_3D.push_back(Point3f(half_x, -half_y, 0));   //br below right
    STATION_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0));  //bl below left
}

//in getAngle
void AngleSolver::solveAngles(exchangeStation exchangeStation)
{
    vector<Point2f> armorVertices;  // bl->tl->tr->br     左下 左上 右上 右下
    armorVertices.push_back(exchangeStation.left_down);
    armorVertices.push_back(exchangeStation.left_up);
    armorVertices.push_back(exchangeStation.right_up);
    armorVertices.push_back(exchangeStation.right_down);
    targetContour = armorVertices;
    targetCenter = exchangeStation.center;

    solvePnP(STATION_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);

    GUN_CAM_DISTANCE_Y = 35;                        //-75;/////////////////!!!!!!!!!!!!!!!!!!!!!!
    tVec.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;    ///暂时不知道
    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);
    distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);


    // Target is too far, using PinHole solver
    if (distance > 5000)
    {
        PinHole_solver();
    }
        // Target is moderate, using PnP solver
    else
    {
        P4P_solver();
    }
}

//in getAngle
void AngleSolver::P4P_solver()
{
    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);

    double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;
    x_pitch = -atan(tan_pitch) * 180 / CV_PI;
    y_yaw = atan(tan_yaw) * 180 / CV_PI;
}

//in getAngle
void AngleSolver::PinHole_solver()
{
    double fx = CAMERA_MATRIX.at<double>(0, 0);
    double fy = CAMERA_MATRIX.at<double>(1, 1);
    double cx = CAMERA_MATRIX.at<double>(0, 2);
    double cy = CAMERA_MATRIX.at<double>(1, 2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(targetCenter);

    //对像素点去畸变
    undistortPoints(in, out, CAMERA_MATRIX, DISTORTION_COEFF, cv::noArray(), CAMERA_MATRIX);
    pnt = out.front();
    //cout<<pnt<<endl;
    //去畸变后的比值
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;
    //cout<<"rnew"<<rxNew<<endl;
    y_yaw = atan(rxNew) / CV_PI * 180;
    x_pitch = -atan(ryNew) / CV_PI * 180;
}

//get x_pitch
void AngleSolver::compensateAngle()
{
    compensateOffset();
}

// in compensateAngle()
void AngleSolver::compensateOffset()
{
    float camera_target_height = distance * sin(x_pitch / 180 * CV_PI);
    float gun_target_height = camera_target_height + GUN_CAM_DISTANCE_Y;           /////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    float gun_pitch_tan = gun_target_height / (distance * cos(x_pitch / 180 * CV_PI));
    x_pitch = atan(gun_pitch_tan) / CV_PI * 180;
}


void AngleSolver::getAngle(exchangeStation exchangeStation, int& X, int& Y, int& Z)
{
    solveAngles(exchangeStation);
    X = (int)(tVec.at<double>(0, 0));
    Y = (int)(tVec.at<double>(1, 0));
    Z = (int)(tVec.at<double>(2, 0));
}

void AngleSolver::showDebugInfo(bool showCurrentResult, bool ifWind, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams)
{
    if (showCurrentResult)
    {
        Mat angleImage = Mat::zeros(250, 600, CV_8UC3);
        putText(angleImage, "Yaw: " + to_string(y_yaw), Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Pitch: " + to_string(x_pitch), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Distance: " + to_string(distance), Point(100, 150), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        if (!ifWind) {
            putText(angleImage, "X:" + to_string((int)(tVec.at<double>(0))), Point(50, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(angleImage, "Y:" + to_string((int)(tVec.at<double>(1))), Point(250, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(angleImage, "Z:" + to_string((int)(tVec.at<double>(2))), Point(400, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        }
        namedWindow("AngleSolver");
        imshow("AngleSolver", angleImage);
    }
}

void AngleSolver::eulerAngle()
{
    //yaw--alpha        pitch----beta         roll--gamma
    float theta = 30 * CV_PI / 180; // 假设正方体的面与图像夹角为30度
    cv::Mat R = (cv::Mat_<float>(3, 3) << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0,           0, 1); // 假设旋转矩阵为Rz(theta)
    cv::Mat tvec_copy = tVec.clone();
    std::vector<cv::Point3f> object_points;
    object_points.push_back(cv::Point3f(X,Y,Z)); // 正方体特征点坐标，这里假设正方体边长为250
    // ...

    // 2. 将旋转矩阵转换为Z-Y-X欧拉角对应的旋转矩阵
    cv::Mat R_zxy;
    cv::Rodrigues(cv::Vec3f(R.at<float>(2, 0), R.at<float>(1, 0), R.at<float>(0, 0)), R_zxy);

    // 3. 通过正方体特征点在图像上的坐标计算出它们在相机坐标系下的坐标
    cv::Mat object_points_mat(object_points);
    cv::Mat camera_points_mat;
    cv::transform(object_points_mat, camera_points_mat, R_zxy.t());//, -R_zxy.t() * tvec

    // 4. 使用相机的内参矩阵将相机坐标系下的坐标转换为图像坐标
    cv::Mat camera_matrix = CAMERA_MATRIX;
    cv::Mat distortion_coefficients = cv::Mat::zeros(5, 1, CV_32FC1); // 假设相机没有畸变
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(camera_points_mat, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), camera_matrix, distortion_coefficients, image_points);

    // 5. 根据投影得到的四个角点之间的距离比例计算出正方体的朝向
    float dx = image_points[1].x - image_points[0].x;
    float dy = image_points[2].y - image_points[0].y;
    float dz = dx / sqrt(2.f); // 假设正方体边长为250
    float pitch = atan2(dy, dz);
    float yaw = atan2(-image_points[1].y + image_points[0].y, dx);
    float roll = atan2(-R.at<float>(0, 1), R.at<float>(0, 0)); // 计算roll角，具体公式可以参考文末的链接
    pitch=pitch*180/CV_PI;
    yaw=yaw*180/CV_PI;
    roll=roll*180/CV_PI;
    std::cout << "pitch: " << pitch << ", yaw: " << yaw << ", roll: " << roll << std::endl;

}


void AngleSolver::Solver(const char* filePath, int camId, exchangeStation exchangeStation)
{
    setCameraParam(filePath, camId);
    setStationSize(240, 240);
    compensateAngle();
    getAngle(exchangeStation, X, Y, Z);
    eulerAngle();
    STATION_POINTS_3D.clear();
}

