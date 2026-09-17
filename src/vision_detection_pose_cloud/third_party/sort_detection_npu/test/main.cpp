#include <iostream>
#include <opencv2/opencv.hpp>
#include "object_region_detection.h"

#include <sys/time.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
 
using namespace std;
using namespace cv;
 
int main(int argc, char** argv) 
{
    std::unique_ptr<ObjectRegionDetection> ObjectRegionDetector = std::make_unique<ObjectRegionDetection>();

    google::InitGoogleLogging("");
 


    std::string folder_path = "/home/neardi/workspace/project/sort_detection_npu/data/yq/*";
    std::string output_path = "/home/neardi/workspace/project/sort_detection_npu/data/image1_result/";

    std::vector<cv::String> paths;
    cv::glob(folder_path, paths, false);

    for (const auto& path : paths) {
        std::cout << "path=" << path << std::endl;
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::cout << "错误: 无法读取图像，图像为空!" << std::endl;
            continue;; // 或者处理错误
        }
        std::string::size_type iPos = path.find_last_of('/') + 1;
        std::string img_name = path.substr(iPos, path.length() - iPos); 
        std::cout << "img_name=" << img_name << std::endl;
 
        int localization_success = ObjectRegionDetector->DetectRegion(frame);
 
        // imshow("result", frame);
        // waitKey(0);
        std::string write_path = output_path + img_name.substr(0, img_name.length() - 4) + ".bmp";
        std::cout << "write_path=" << write_path << std::endl;
        cv::imwrite(write_path, frame);
    }

 
    return 0;
}