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

unsigned char * g_pRgbBuffer;     //处理后数据缓存区

using namespace Eigen;
using namespace cv;
using namespace std;

class MvRos
{
    ros::NodeHandle nh_;  
    image_transport::ImageTransport it_;  
    image_transport::Publisher pub;
	ros::Publisher cam_info_pub;
    //相机信息发布
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
	int                     iCameraCounts = 1;
	int                     iStatus=-1;
	tSdkCameraDevInfo       tCameraEnumList;
	int                     hCamera;
	tSdkCameraCapbility     tCapability;      
	tSdkFrameHead           sFrameInfo;
	BYTE*			        pbyBuffer;
	IplImage *iplImage = NULL;
	int                     channel=3;
	int                     ExposureTime=400;
	string                  CameraParam="package://mvros/config/CameraParam.yaml";

    public:
		MvRos():it_(nh_){    
        //相机信息发布
		//ros::param::get("ExposureTime", ExposureTime);
		//ros::param::get("CameraParam", CameraParam);

        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
//    image_transport::CameraPublisher image_pub_;
        cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/mindvision/camera_info", 1);
        pub = it_.advertise("/mindvision/image_raw", 1); 
        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, "mindvision", CameraParam)); 
		
		CameraSdkInit(1);

		//枚举设备，并建立设备列表
		iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
		printf("state = %d\n", iStatus);

		printf("count = %d\n", iCameraCounts);
		//没有连接设备

		//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
		iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

		//初始化失败
		printf("state = %d\n", iStatus);

		//获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
		CameraGetCapability(hCamera,&tCapability);

		//
		g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
		//g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

		/*让SDK进入工作模式，开始接收来自相机发送的图像
		数据。如果当前相机是触发模式，则需要接收到
		触发帧以后才会更新图像。    */
		CameraPlay(hCamera);

		CameraSetAeState(hCamera, false);
		CameraSetExposureTime(hCamera, 700);

		CameraSetAnalogGain(hCamera, 10);
		CameraSetGamma(hCamera, 50);
		CameraSetContrast(hCamera, 200);

		/*其他的相机参数设置
		例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
		CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
		CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
		更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
		*/

		if(tCapability.sIspCapacity.bMonoSensor){
		channel=1;
		CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
		}else{
		channel=3;
		CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
		}

		while (1) {
		// 读取相机
		if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
			{
			CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
				
			cv::Mat matImage(
				cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
				sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
				g_pRgbBuffer
				);
				//发布图片消息
			cv_bridge::CvImage cv_image;

			cv_image.image = matImage;

			cv_image.encoding = "bgr8";
			cv_image.header.frame_id = "mindvision_frame";
			cv_image.header.stamp = ros::Time::now();
			pub.publish(cv_image.toImageMsg());

			sensor_msgs::CameraInfoPtr ci_(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
				
			ci_->header.frame_id = cv_image.header.frame_id;
			ci_->header.stamp = cv_image.header.stamp;
			cam_info_pub.publish(*ci_);

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera,pbyBuffer);
			}
		}    
		CameraUnInit(hCamera);
		//注意，现反初始化后再free
		free(g_pRgbBuffer);
		}
        ~MvRos()  
        {  
            //cv::destroyWindow(IN_WINDOW);  
            //cv::destroyWindow(OUT_WINDOW);  
        }  
};

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "mindvision");
    ros::NodeHandle nh_;  
  	MvRos mv;
	ros::spin();
    return 0;
}


