#ifndef __POINT_CLOUD_UNITY__
#define __POINT_CLOUD_UNITY__

#include <sl/Camera.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace BYTECAT
{
    void zed2pointCloud(const sl::Mat& data_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud);
    std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    float convertColor(float colorIn);
}


#endif