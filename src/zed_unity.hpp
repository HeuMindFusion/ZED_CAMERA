#ifndef __ZED_CAMERA__
#define __ZED_CAMERA__

#include <stdio.h>
#include <string.h>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

namespace BYTECAT
{
    cv::Mat slMat2cvMat(sl::Mat& input);
   
}


#endif