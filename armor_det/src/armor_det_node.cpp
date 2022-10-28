#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "auto_aim_interface/msg/camera_image.hpp"

#include "opencv2/opencv.hpp"

#include "cuda.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "NvInfer.h"
#include "NvOnnxParser.h"

#include <vector>
#include <string>
#include <iostream>
#include <functional>

#include "armor_det/tools.hpp"

using namespace std::placeholders;

class ArmorDetNode : public rclcpp::Node{
public:
    ArmorDetNode():Node("armor_det_node"){
        Logger logger;

        // read the serialzed engine file
        const char * engine_file_path = "/home/littsk/Desktop/backup/FastestDet/nx_int8_armor_det.engine";
        std::vector<uint8_t> engine_string;
        size_t engine_size = read_engine_file(engine_file_path, engine_string);
        
        /* 
        decoder engine
        create execute context
        */
        nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(logger); 
        engine = runtime->deserializeCudaEngine(engine_string.data(), engine_size);
        context = engine->createExecutionContext();
        
        //allocate the memory for input and output
        input_idx = engine->getBindingIndex("x"), output_idx = engine->getBindingIndex("y");
        nvinfer1::Dims input_dims = engine->getBindingDimensions(input_idx);
        nvinfer1::Dims output_dims = engine->getBindingDimensions(output_idx);
        size_t input_size = get_dim_size(input_dims), output_size = get_dim_size(output_dims);
        cudaMalloc(&buffers[input_idx], input_size * sizeof(float));
        cudaMalloc(&buffers[output_idx], output_size * sizeof(float));

        RCLCPP_INFO(this->get_logger(), "execute context build successful\n");
        
        //====================================================================================//

        auto qos_policy = rclcpp::SensorDataQoS();
        image_sub = this->create_subscription<auto_aim_interface::msg::CameraImage>(
            "image_raw", 5, std::bind(&ArmorDetNode::sub_callback, this, _1)
        );

    }
private:
    nvinfer1::ICudaEngine * engine;
    nvinfer1::IExecutionContext * context;
    int input_idx, output_idx;
    void * buffers[2];

    rclcpp::Subscription<auto_aim_interface::msg::CameraImage>::SharedPtr image_sub;
    
    //====================================================================================//
    void sub_callback(const auto_aim_interface::msg::CameraImage::SharedPtr image_msg)
    {   
        RCLCPP_INFO(this->get_logger(), "here we are\n");
        std::vector<uchar> data = image_msg->data;
        cv::Mat matImage(
            cv::Size((int)image_msg->width, (int)image_msg->height), 
            CV_8UC3,
            data.data()
        );
        cv::imwrite("~/Desktop/test.jpg", matImage);
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}