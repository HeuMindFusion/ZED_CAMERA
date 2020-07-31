#include <iostream>
#include "zed_unity.hpp"
#include <stdio.h>
//#include "point_cloud_unity.hpp"

int main()
{
    sl::Camera zed;

    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE; // no depth computation required here
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_parameters.coordinate_units = sl::UNIT::MILLIMETER; // Use millimeter units (for depth measurements)

                                                             // Open the camera
    sl::ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Camera Open " << zed_open_state << " Exit program." << std::endl;
        return EXIT_FAILURE;
    }
    char key = 0;
    auto camera_info = zed.getCameraInformation();
    sl::Mat depth_map, depth_image_zed;
    sl::Mat zed_left_image, zed_right_image;
    std::string stereoWinName = "stereo image";
    cv::namedWindow(stereoWinName, 0);
    
    bool startFlag = false;
    std::cout << "Press space to start recording" << std::endl;
    std::cout << "Press Esc to exit" << std::endl;
    int count = 0;
    
    while (key != 27)
    {
        auto returned_state = zed.grab();
        if (returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(zed_left_image, sl::VIEW::LEFT);
            zed.retrieveImage(zed_right_image, sl::VIEW::RIGHT);
            zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
            cv::Mat cvImage_left = BYTECAT::slMat2cvMat(zed_left_image);
            cv::Mat cvImage_right = BYTECAT::slMat2cvMat(zed_right_image);

            float depth_value = 0;
            cv::Mat depth(depth_map.getHeight(), depth_map.getWidth(), CV_32FC1, depth_map.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat stereoImg;
            cv::hconcat(cvImage_left, cvImage_right, stereoImg);
            //BYTECAT::drawLine(stereoImg);
            if (key == 32)
                startFlag = true;
            if (startFlag)
            {
                std::string leftName = cv::format("../data/left%d.png", count);
                std::cout<<leftName<<std::endl;
                std::string rightName = cv::format("../data/right%d.png", count);
                std::string depthName = cv::format("../data/depth%d.png", count);
                cv::imwrite(leftName, cvImage_left);
                cv::imwrite(rightName, cvImage_right);
                cv::Mat depth_ushort(cv::Size(depth.cols, depth.rows), CV_16UC1);
                BYTECAT::convert2ushort(depth, depth_ushort);
                cv::imwrite(depthName, depth_ushort);
                count += 1;
            }
            cv::imshow(stereoWinName, stereoImg);

        }
        key = cv::waitKey(1);
    }
    return 0;
}