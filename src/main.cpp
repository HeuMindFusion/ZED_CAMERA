#include "zed_unity.hpp"
#include <stdio.h>
int main(int argc, char **argv) {
    // Create a ZED Camera object
    sl::Camera zed;
    
    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY; // no depth computation required here
    


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
    sl::Mat depth_image_zed(image_width, image_height, sl::MAT_TYPE::U8_C4);
    

    // Capture new images until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
        // Check that grab() is successful
        auto returned_state = zed.grab();
        if (returned_state == sl::ERROR_CODE::SUCCESS) {

            // Retrieve left image
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            zed.retrieveImage(depth_image_zed, sl::VIEW::DEPTH);
            convertUnit(depth_image_zed, zed.getInitParameters().coordinate_units, sl::UNIT::MILLIMETER);
            auto state = depth_image_zed.write("test.png");

            cv::Mat cvImage_left = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat depth_image_ocv = BYTECAT::slMat2cvMat(depth_image_zed);
            cv::imshow(win_name, cvImage_left);
            cv::imshow("Depth", depth_image_ocv);
            std::vector<cv::Mat> rgbChannels(4);
            split(depth_image_ocv, rgbChannels);
            cv::imshow("test", rgbChannels[1]);

            int v1 = static_cast<int>(depth_image_ocv.at<cv::Vec3b>(100, 100)[0]);
            int v2 = static_cast<int>(depth_image_ocv.at<cv::Vec3b>(100, 100)[1]);
            int v3 = static_cast<int>(depth_image_ocv.at<cv::Vec3b>(100, 100)[2]);
            int v4 = static_cast<int>(depth_image_ocv.at<cv::Vec3b>(100, 100)[3]);
            std::cout << v1<<","<< v2 << "," << v3 << "," << v4 << "," << std::endl;
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
