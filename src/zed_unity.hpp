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
}


#endif