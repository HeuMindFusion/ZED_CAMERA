#ifndef __ZED_CAMERA__
#define __ZED_CAMERA__

#include <stdio.h>
#include <string.h>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>


namespace BYTECAT
{
    cv::Mat slMat2cvMat(sl::Mat& input);
    cv::Mat splitImage(const cv::Mat& image);
    void saveImage(sl::Mat& zed_image, std::string filename);
    void savePointCloud(sl::Camera& zed, std::string filename);

    //std::string  PointCloudFormatExt = ".ply";
    //sl::Mat depth_image_zed(image_width, image_height, sl::MAT_TYPE::U8_C4);
    //depth_map.getValue(100, 100, &depth_value);
}


#endif