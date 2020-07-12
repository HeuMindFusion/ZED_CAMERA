#include "zed_unity.hpp"
#include <stdio.h>
#include "point_cloud_unity.hpp"


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
    cv::namedWindow("depth", 0);
    
    sl::Mat zed_image;
    // Initialise camera setting
    auto camera_info = zed.getCameraInformation();
    const int image_width = camera_info.camera_configuration.resolution.width;
    const int image_height = camera_info.camera_configuration.resolution.height;
    sl::Resolution new_image_size(image_width, image_height);
   
    
    sl::Mat depth_map, depth_image_zed;
    sl::Mat data_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = BYTECAT::createRGBVisualizer(p_pcl_point_cloud);
    p_pcl_point_cloud->points.resize(new_image_size.area());
    // Capture new images until 'q' is pressed
    viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    viewer->setCameraClipDistances(0.1, 1000);
    int count = 0;
    char key = ' ';
    while (key != 'q') 
    {
        // Check that grab() is successful
        std::string leftName =  cv::format("../data/left%d.png", count);
        std::string depthName = cv::format("../data/depth%d.png", count);
        std::string cloudPointName = cv::format("../data/cloudPoint%d", count);
       
        auto returned_state = zed.grab();
        if (returned_state == sl::ERROR_CODE::SUCCESS) 
        {
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
            cv::Mat cvImage_left = BYTECAT::slMat2cvMat(zed_image);
            cv::imshow(win_name, cvImage_left);
           
            float depth_value = 0;
           
            
            cv::Mat depth(depth_map.getHeight(), depth_map.getWidth(), CV_32FC1, depth_map.getPtr<sl::uchar1>(sl::MEM::CPU));
            zed.retrieveImage(depth_image_zed, sl::VIEW::DEPTH, sl::MEM::CPU, new_image_size);
            cv::Mat depth_image_ocv = BYTECAT::slMat2cvMat(depth_image_zed);
            cv::imshow("depth", depth_image_ocv);

            zed.retrieveMeasure(data_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, new_image_size);
            BYTECAT::zed2pointCloud(data_cloud, p_pcl_point_cloud);
            viewer->updatePointCloud(p_pcl_point_cloud);
            viewer->spinOnce(10);
            if (key == 's')
            {
                std::cout << "save left image,depth image and point cloud file......" << std::endl;
                BYTECAT::savePointCloud(zed, cloudPointName);
                
                cv::Mat depth_ushort(cv::Size(depth.cols, depth.rows), CV_16UC1);
                BYTECAT::saveDepth(depth, depth_ushort);
                cv::imwrite(leftName, cvImage_left);
                cv::imwrite(depthName, depth_ushort);
                count = count + 1;
            }

        }
        else 
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
