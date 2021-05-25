#include <cmath>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include "CameraApi.h"
#include <camera_info_manager/camera_info_manager.h>

using namespace Eigen;
using namespace cv;
using namespace std;

//设备描述信息
int                     iCameraCounts = 1;
int                     iStatus=-1;
tSdkCameraDevInfo       tCameraEnumList;
int                     hCamera;
tSdkCameraCapbility     tCapability;      
tSdkFrameHead           sFrameInfo;
BYTE*			        pbyBuffer;
IplImage *iplImage = NULL;
int                     channel=3;
