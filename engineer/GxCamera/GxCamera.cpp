#include"GxCamera.h"

namespace GxCamera
{
    Mat src;
    bool isReading;
    bool isWriting;
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (!GxCamera::isReading)
{
    GxCamera::isWriting = 1;
    int rows = pFrame->nHeight;
    int cols = pFrame->nWidth;
    src = Mat::zeros(Size(cols, rows), CV_8UC3);
    int cvType = CV_8UC1;
    int bayerType = BAYERRG;
    //cvImage.data = (uchar*)srcFrame.pImgBuf;
    DxRaw8toRGB24((void*)pFrame->pImgBuf, GxCamera::src.data,
    pFrame->nWidth, pFrame->nHeight,
    RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
    //将bayer图像转化为RGB图像，参数为(指向愿图像8为数据缓冲区的指针，指向目标图像数据缓冲区的指针，图像高，图像宽，转换算法类型，图像格式类型，是否上下翻转)
    GxCamera::isWriting = 0;
}
else
cout << " ";
}
}

camera::camera()
{
    initlib();
    isWriting = 1;
    isReading = 0;
    camHandle = NULL;
    GxCamera::isReading = 0;
    GxCamera::isWriting = 0;
    isAcquiring = 0;
    isColorCam = true;
}

camera::~camera()
{
    if (camHandle != NULL)
    {
        closeDev();								//关闭已经打开的相机
    }
    closelib();
}

GX_STATUS camera::initlib()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXInitLib();
    GX_VERIFY(status);
    return status;
}

GX_STATUS camera::closelib()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseLib();
    GX_VERIFY(status);
    return status;
}

GX_STATUS camera::autoConnect()
{
    if (camHandle != NULL)
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        status = closeDev();								//关闭已经打开的相机
        GX_VERIFY(status);							//根据状态判断是否结束函数并返回错误码
    }
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t devNum = 0;
    status = GXUpdateDeviceList(&devNum,1000);
    if(status == GX_STATUS_SUCCESS && devNum > 0)
    {
        GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[devNum];
        size_t size = devNum * sizeof(GX_DEVICE_BASE_INFO);

        status = GXGetAllDeviceBaseInfo(baseinfo,&size);
        //delete []baseinfo;
        GX_OPEN_PARAM* stOpenParam = new GX_OPEN_PARAM;	//大恒打开设备接口专用结构体
        stOpenParam->accessMode = GX_ACCESS_EXCLUSIVE;	//将设备访问模式设置为独占模式
        stOpenParam->openMode = GX_OPEN_SN;				//打开方式设置为由SN打开
        stOpenParam->pszContent = const_cast<char*>(baseinfo[0].szSN);
        //标准C字符串，由openmode决定，这里显然是设备的SN码
        status = GXOpenDevice(stOpenParam, &camHandle);
        //通过上面定义的结构体，打开设备，并由返回值判断是否打开成功
        //接口返回的设备句柄
        //结构体
        GX_VERIFY(status);								//确认状态

        //status = readCameraParams();					//读取开启相机的参数
        GX_VERIFY(status);								//确认状态

        return status;
    }
    else
        return GX_STATUS_ERROR;
}

GX_STATUS camera::startAcquiring()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //注册回调函数
    GXRegisterCaptureCallback(camHandle, NULL, GxCamera::OnFrameCallbackFun);
    // 发送开始采集命令
    status = GXSendCommand(camHandle, GX_COMMAND_ACQUISITION_START);
    GX_VERIFY(status);

    //采集开启
    isAcquiring = true;



    return status;
}

GX_STATUS camera::snapCvMat()		//采集图片(CvMat格式)
{
    while(1)
    {
        if(!isReading)
        {
            if(camHandle == NULL)
                break;
            isWriting = 1;
            GX_FRAME_DATA* frameData = new GX_FRAME_DATA;
            GX_STATUS status = GX_STATUS_SUCCESS;
            status = GXGetImage(camHandle, frameData, 500);//获取当前图像，参数为(设备句柄，存储获取图像的指针，取图超时时间）
            GX_VERIFY(status);
            cvtGxFrameToCvMat(*frameData, src);
            isWriting = 0;
            return status;
        }
    }
}

void camera::cvtGxFrameToCvMat(GX_FRAME_DATA& srcFrame, Mat& dstMat)
//转一个Mat出来
{
    int rows = srcFrame.nHeight;
    int cols = srcFrame.nWidth;
    int cvType = CV_8UC1;
    int bayerType = BAYERRG;
    Mat cvImage;

    switch (srcFrame.nPixelFormat)
    {
        case GX_PIXEL_FORMAT_MONO8:			cvType = CV_8UC1; break;
        case GX_PIXEL_FORMAT_MONO8_SIGNED:	cvType = CV_8SC1; break;
        case GX_PIXEL_FORMAT_BAYER_GR8:		cvType = CV_8UC3; bayerType = BAYERGR; break;
        case GX_PIXEL_FORMAT_BAYER_RG8:		cvType = CV_8UC3; bayerType = BAYERRG; break;
        case GX_PIXEL_FORMAT_BAYER_GB8:		cvType = CV_8UC3; bayerType = BAYERGB; break;
        case GX_PIXEL_FORMAT_BAYER_BG8:		cvType = CV_8UC3; bayerType = BAYERBG; break;
        default:											  break;
    }

    if (isColorCam)
    {
        DxRaw8toRGB24(srcFrame.pImgBuf, cvImage.data,
                      srcFrame.nWidth, srcFrame.nHeight,
                      RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
        //将bayer图像转化为RGB图像，参数为(指向愿图像8为数据缓冲区的指针，指向目标图像数据缓冲区的指针，图像高，图像宽，转换算法类型，图像格式类型，是否上下翻转)
    }
    else
    {
        //cvImage.data = (uchar*)srcFrame.pImgBuf;
        memcpy(cvImage.data, srcFrame.pImgBuf, srcFrame.nImgSize);
    }
    // 将CvMat拷贝到dstMatch中
    cvImage.copyTo(dstMat);
}

GX_STATUS camera::stopAcquiring()					//停止采集
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    //发送停止采集命令
    status = GXSendCommand(camHandle, GX_COMMAND_ACQUISITION_STOP);
    GX_VERIFY(status);

    //释放图像缓冲区buffer
    free(gxFrame.pImgBuf);
    gxFrame.pImgBuf = NULL;

    //采集关闭
    isAcquiring = false;

    return status;
}

GX_STATUS camera::closeDev()					//关闭设备
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool confSavable = false;
    status = GXIsImplemented(camHandle, GX_ENUM_USER_SET_DEFAULT, &confSavable);
    //查询设备是否支持某种功能，参数为（设备句柄（打开设备的时候返回的那个），功能码ID，bool返回值（是否支持该功能））
    //这次查询的功能应该是是否支持保存用户自定义设置
    GX_VERIFY(status);								//确认状态
    if (confSavable == true)
    {
        status = GXSetEnum(camHandle, GX_ENUM_USER_SET_SELECTOR, GX_ENUM_USER_SET_SELECTOR_USERSET0);
        //设置枚举值，参数为（设备句柄，功能码ID，用户要设置的枚举值），这里把当前存储的设置档位调到用户设置0（类似于多组设置同时存在，选择其中一组生效）
        GX_VERIFY(status);							//确认状态
        status = GXSendCommand(camHandle, GX_COMMAND_USER_SET_SAVE);
        //发送指令，参数为（设备句柄，下达的指令），这里下达的指令为保存参数组
        GX_VERIFY(status);;							//确认状态
        status = GXSetEnum(camHandle, GX_ENUM_USER_SET_DEFAULT, GX_ENUM_USER_SET_DEFAULT_USERSET0);
        //设置枚举值，参数为（设备句柄，功能码ID，用户要设置的枚举值），这里把默认的设置档位调到用户设置0（类似于多组设置同时存在，选择其中一组生效）
        GX_VERIFY(status);							//确认状态
    }
    status = GXCloseDevice(camHandle);				//官方API，关闭设备，传入参数为设备句柄
    GX_VERIFY(status);								//确认状态
    camHandle = NULL;								//把设备句柄设置为空指针
    return status;									//完事儿跑路
}

void camera::run()
{
    // GX_STATUS (*captureTr)();
    //captureTr = this->snapCvMat;
    std::function<GX_STATUS(void)> captureTr = std::bind(&camera::snapCvMat, this);
    thread capture(captureTr);
    capture.detach();
}

void camera::getsrc(Mat &dst)
{
    while(1)
    {
        if(!GxCamera::isWriting)
        {
            GxCamera::isReading = 1;
            GxCamera::src.copyTo(dst);
            GxCamera::isReading = 0;
            break;
        }
        cout << " ";
    }
}

void camera::GetErrorString(GX_STATUS errorStatus)
{
    char* error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&errorStatus, NULL, &size);
    //获取程序的最新错误信息，参数为(获取错误码，错误信息缓冲区地址，错误信息缓冲区大小)
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return;
    }

    // Get error description
    emStatus = GXGetLastError(&errorStatus, error_info, &size);
    //获取程序的最新错误信息，参数为(获取错误码，错误信息缓冲区地址，错误信息缓冲区大小)
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete[]error_info;
        error_info = NULL;
    }
}
