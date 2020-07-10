#include "zed_unity.hpp"
#include <stdio.h>
int main(int argc, char **argv) {
    // Create a ZED Camera object
    sl::Camera zed;
    
    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    //init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY; // no depth computation required here
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_parameters.coordinate_units = sl::UNIT::MILLIMETER; // Use millimeter units (for depth measurements)


    // Open the camera
    sl::ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
        printf("Camera Open", zed_open_state, "Exit program.");
        return EXIT_FAILURE;
    }
    
    cv::String win_name = "left camera";
    cv::namedWindow(win_name,0);
    
    sl::Mat zed_image;
    
    // Initialise camera setting
    auto camera_info = zed.getCameraInformation();
    const int image_width = camera_info.camera_configuration.resolution.width;
    const int image_height = camera_info.camera_configuration.resolution.height;
    sl::Resolution new_image_size(image_width, image_height);
    //sl::Mat depth_image_zed(image_width, image_height, sl::MAT_TYPE::U8_C4);
    
    sl::Mat depth_map;
    // Capture new images until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
        // Check that grab() is successful
        auto returned_state = zed.grab();
        if (returned_state == sl::ERROR_CODE::SUCCESS) 
        {
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
            cv::Mat cvImage_left = BYTECAT::slMat2cvMat(zed_image);
            cv::imshow(win_name, cvImage_left);
           
            float depth_value = 0;
            depth_map.getValue(100, 100, &depth_value);
            
            cv::Mat depth(depth_map.getHeight(), depth_map.getWidth(), CV_32FC1, depth_map.getPtr<sl::uchar1>(sl::MEM::CPU));
            BYTECAT::savePointCloud(zed, "test");
            cv::imwrite("kk.png", depth);
            cv::imwrite("kk.bmp", depth);
            std::cout << "depth" << depth_value << std::endl;
            std::cout << "depth----" << depth.at<float>(100, 100) << std::endl;
            std::cout << "depth" << depth.channels() << std::endl;
        }else 
        {
            printf("Error during capture : ", returned_state);
            break;
        }
        
        key = cv::waitKey(10);
           
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}
