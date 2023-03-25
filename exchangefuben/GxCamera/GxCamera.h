#include"GxSDK/GxIAPI.h"
#include"GxSDK/DxImageProc.h"
//#include<GxIAPI.h>
//#include<DxImageProc.h>
#include"opencv2/opencv.hpp"
#include<iostream>
#include<thread>

using namespace std;
using namespace cv;

#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }


class camera
{
public:
    camera();
    ~camera();

    GX_STATUS initlib();
    GX_STATUS closelib();
    GX_STATUS connectDeviceBySN(string cameraSN);
    GX_STATUS connectDeviceByIndex(string camIndex);
    GX_STATUS autoConnect();
    GX_STATUS closeDev();
    GX_STATUS startAcquiring();
    GX_STATUS stopAcquiring();
    GX_STATUS snapCvMat();
    void imgProgressCallBack(GX_FRAME_CALLBACK_PARAM* frameData);
    void GetErrorString(GX_STATUS errorStatus);
    void cvtGxFrameToCvMat(GX_FRAME_DATA & srcFrame, cv::Mat & dstMat);
    void getsrc(Mat &dst);
    void run();


private:
    Mat src;
    bool isWriting;
    bool isReading;
    bool isAcquiring;
    GX_DEV_HANDLE camHandle;
    GX_FRAME_DATA gxFrame;
    bool isColorCam;
};
