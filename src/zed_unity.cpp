#include "zed_unity.hpp"

namespace BYTECAT
{
    cv::Mat slMat2cvMat(sl::Mat& input)
    {
        int cv_type = -1;
        switch (input.getDataType())
        {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
        }
        return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));

    }

    cv::Mat splitImage(const cv::Mat& image)
    {
        std::vector<cv::Mat> rgbChannels(4);
        split(image, rgbChannels);
        return rgbChannels[3];

        //test
        /*cv::imshow("test", rgbChannels[1]);
        int v1 = static_cast<int>(image.at<cv::Vec3b>(100, 100)[0]);
        int v2 = static_cast<int>(image.at<cv::Vec3b>(100, 100)[1]);
        int v3 = static_cast<int>(image.at<cv::Vec3b>(100, 100)[2]);
        int v4 = static_cast<int>(image.at<cv::Vec3b>(100, 100)[3]);
        std::cout << v1<<","<< v2 << "," << v3 << "," << v4 << "," << std::endl;*/
    }


    void saveImage(sl::Mat& zed_image, std::string filename)
    {
        cv::Mat cvImage = slMat2cvMat(zed_image);
        cv::imwrite(filename, cvImage);
    }

    void savePointCloud(sl::Camera& zed, std::string filename) 
    {
        std::cout << "Saving Point Cloud... " << std::endl;

        sl::Mat point_cloud;
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

        auto state = point_cloud.write((filename + ".ply").c_str());

        if (state == sl::ERROR_CODE::SUCCESS)
            std::cout << "Point Cloud has been saved under " << filename << ".ply" << std::endl;
        else
            std::cout << "Failed to save point cloud... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
    }


    void zed2pclCloud()
    {

    }

    void convert2ushort(const cv::Mat& depth, cv::Mat& depthOut)
    {
        for (int rows = 0; rows < depth.rows; rows++)
        {
            for (int cols = 0; cols < depth.cols; cols++)
            {
                depthOut.at<ushort>(rows, cols) = static_cast<ushort>(depth.at<float>(rows, cols));
            }
        }       
    }

    void drawLine(cv::Mat& img)
    {
        const uchar interval = 20;
        for (int row = interval; row < img.rows;)
        {
            
           cv::line(img, cv::Point(0, row),
               cv::Point(img.cols, row), cv::Scalar(0, 0, 255));
           row += interval;
        }

    }

    void image2video(const int& count, const int& frameWidth, const int& frameHeight)
    {
        double fps = 30;
        bool iscolor = true;
        cv::VideoWriter Writer;
        Writer = cv::VideoWriter("../data/stereo01.mp4", cv::CAP_OPENCV_MJPEG, 
            fps, cv::Size(frameWidth, frameHeight), iscolor);
        for (int i = 0; i < count; i++)
        {
            std::string fileName = cv::format("../data/left%d.png", i);
            cv::Mat img = cv::imread(fileName);
            if (img.empty())
            {
                std::cout << "image empty!!" << std::endl;
                continue;
            }
            //Writer.write(img);
            Writer << img;
        }
        Writer.release();
    }

}