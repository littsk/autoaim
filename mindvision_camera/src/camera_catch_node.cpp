#include "CameraApi.h" // 相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "auto_aim_interface/msg/camera_image.hpp"

using namespace cv;
using namespace std::chrono_literals;

class CameraCatchNode : public rclcpp::Node{
public:
    CameraCatchNode():Node("camera_catch_node"){

        CameraSdkInit(1);

        // 枚举设备，并建立设备列表
        iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
        RCLCPP_INFO(this->get_logger(), "Enumerate state = %d\n", iStatus);
        RCLCPP_INFO(this->get_logger(), "Camera count = %d\n", iCameraCounts); //specify the number of camera that are found
        if(iCameraCounts == 0){
            RCLCPP_ERROR(this->get_logger(), "There is no camera detected!");
            return;
        }

        // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
        // 初始化失败
        RCLCPP_INFO(this->get_logger(), "Init state = %d", iStatus);
        if (iStatus != CAMERA_STATUS_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Init failed!");
            return;
        }


        // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera,&tCapability);

        // 让SDK进入工作模式，开始接收来自相机发送的图像数据。如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像。    
        CameraPlay(hCamera);

        /*  
        其他的相机参数设置
            例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
            CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
            CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
        更多的参数的设置方法，参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
        */

        if(tCapability.sIspCapacity.bMonoSensor){
            channel=1;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
        }
        else{
            channel=3;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
        }

        // set the camera frame speed to the fastest
        if(CameraSetFrameSpeed(hCamera, tCapability.iFrameSpeedDesc - 1) != CAMERA_STATUS_SUCCESS){
            RCLCPP_INFO(this->get_logger(), "Camera speed set to the fastest!\n");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Camera speed set wrong!\n");
        }

        CameraSetAeState(hCamera, auto_expose);  // whether to set exposure time manually
        if(!auto_expose && CameraSetExposureTime(hCamera, exposure_time) != CAMERA_STATUS_SUCCESS){
            RCLCPP_INFO(this->get_logger(), "Expoture time set wrong!\n");
        }

        RCLCPP_INFO(this->get_logger(), "Camera Catch Node Lauched!\n");

        auto qos_policy = rclcpp::SensorDataQoS();
        image_pub = this->create_publisher<auto_aim_interface::msg::CameraImage>("image_raw", 5);

    }


    void pub_image(){
        if(transmission_speed == Fast){
            tSdkImageResolution roi = {0};
            roi.iIndex = 0xff;     //manually set the resolution

            roi.iWidth = 960;
            roi.iWidthFOV = 1280;

            roi.iHeight = 768;
            roi.iHeightFOV = 1024;

            roi.iHOffsetFOV = 160;
            roi.iVOffsetFOV = 128;

            if(CameraSetImageResolution(hCamera, &roi) == CAMERA_STATUS_SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Transmission speed change to Fast mode, and the resolution goes down accordingly!");
            }
        }
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS){

            auto image_msg = auto_aim_interface::msg::CameraImage();
            image_msg.data.reserve(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

            CameraImageProcess(hCamera, pbyBuffer, image_msg.data.data(), &sFrameInfo);
            image_msg.time_stamp = sFrameInfo.uiTimeStamp;
            image_msg.height = sFrameInfo.iHeight;
            image_msg.width = sFrameInfo.iWidth;
            image_pub->publish(image_msg);

            RCLCPP_INFO(this->get_logger(), "published a image\n");
            
            
            // cv::Mat matImage(
            //         cv::Size(sFrameInfo.iWidth,sFrameInfo.iHeight), 
            //         sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
            //         image_msg.data.data()
            //         );
            // imshow("Opencv Demo", matImage);
            // waitKey(2);

            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
        }
    }

private:

    int                     iCameraCounts = 1;   // how many cameras are linked to the mother board
    int                     iStatus = -1;        //
    tSdkCameraDevInfo       tCameraEnumList;     //
    int                     hCamera;             // handle of the camera
    tSdkCameraCapbility     tCapability;         // 设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 1000;// the max frames of image transmited from camera
    int                     channel = 3;         // the channel of image, usually be 3

    bool                    auto_expose = false;  // whether to auto tune the exposure time
    int                     exposure_time = 4000;// if auto_expose == false, it will work
    
    /*
    if transmission_speed == Fast, resolution of the image will change, from 1280 * 1024 to 960 * 768
    and at the same time, the transmission speed will change from 120hz to 192hz
    */
    typedef enum transmission_speed_ {Normal, Fast} transmission_speed_;
    transmission_speed_ transmission_speed = Normal;

    rclcpp::Publisher<auto_aim_interface::msg::CameraImage>::SharedPtr image_pub;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCatchNode>();
    while(rclcpp::ok()){
        node->pub_image();
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}