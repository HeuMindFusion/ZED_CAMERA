#include "point_cloud_unity.hpp"

namespace BYTECAT
{
    void zed2pointCloud(const sl::Mat& data_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud)
    {
        float* p_data_cloud = data_cloud.getPtr<float>();
        int index = 0;

        // Check and adjust points for PCL format
        for (auto& it : p_pcl_point_cloud->points) {
            float X = p_data_cloud[index];
            if (!isValidMeasure(X)) // Checking if it's a valid point
                it.x = it.y = it.z = it.rgb = 0;
            else {
                it.x = X;
                it.y = p_data_cloud[index + 1];
                it.z = p_data_cloud[index + 2];
                it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
            }
            index += 4;
        }
    
    }

    inline float convertColor(float colorIn)
    {
        uint32_t color_uint = *(uint32_t*)&colorIn;
        unsigned char* color_uchar = (unsigned char*)&color_uint;
        color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
        return *reinterpret_cast<float*> (&color_uint);
    }

    std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) 
    {
        // Open 3D viewer and add point cloud
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
        viewer->setBackgroundColor(0.12, 0.12, 0.12);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        return (viewer);
    }


}//namespace 