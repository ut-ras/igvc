#ifndef VISION_NODE_H
#define VISION_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace vision_node
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImagePolicy2;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy3;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy4;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy5;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy6;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy7;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy8;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImagePolicy9;

  class VisionNode
  {
  public:
    VisionNode(const ros::NodeHandle& nh);
    ~VisionNode();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;
    double m_match_threshold;
    int m_white_threshold;
    int m_num_cameras;
    std::string m_base_frame_id;
//    std::string m_image_1_frame_id;
//    std::string m_image_2_frame_id;
//    std::string m_image_3_frame_id;

    std::vector<std::string> m_image_frame_ids;

    ros::Publisher m_ground_cloud_pub;
    ros::Publisher m_obstacle_cloud_pub;

    message_filters::Synchronizer<ImagePolicy2>* m_sync2;
    message_filters::Synchronizer<ImagePolicy3>* m_sync3;
    message_filters::Synchronizer<ImagePolicy4>* m_sync4;
    message_filters::Synchronizer<ImagePolicy5>* m_sync5;
    message_filters::Synchronizer<ImagePolicy6>* m_sync6;
    message_filters::Synchronizer<ImagePolicy7>* m_sync7;
    message_filters::Synchronizer<ImagePolicy8>* m_sync8;
    message_filters::Synchronizer<ImagePolicy9>* m_sync9;

    bool m_have_clouds;

    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid;
//    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid1;
//    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid2;
//    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid3;

    tf::TransformListener m_tf_listener;

    std::vector<image_geometry::PinholeCameraModel> m_cam_models;
//    image_geometry::PinholeCameraModel m_cam_model_1;
//    image_geometry::PinholeCameraModel m_cam_model_2;
//    image_geometry::PinholeCameraModel m_cam_model_3;

    sensor_msgs::PointCloud2 m_obstacle_cloud_msg;
    sensor_msgs::PointCloud2 m_ground_cloud_msg;

    void imageCallback2(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1);
    void imageCallback3(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2);
    void imageCallback4(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3);
    void imageCallback5(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4);
    void imageCallback6(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5);
    void imageCallback7(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6);
    void imageCallback8(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6, const sensor_msgs::ImageConstPtr& image7);
    void imageCallback9(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6, const sensor_msgs::ImageConstPtr& image7, const sensor_msgs::ImageConstPtr& image8);

    void processImages(std::vector<sensor_msgs::ImageConstPtr>& images);
    bool colorsMatch(std::vector<CvScalar> colors, std::vector<sensor_msgs::ImageConstPtr>& images, CvScalar matched_color);
    bool transformGridToCameras(std::vector<sensor_msgs::ImageConstPtr>& images, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds);
    double colorDistance(CvScalar a, CvScalar b);
    void parseEncoding(CvScalar s, std::string encoding, unsigned char& red, unsigned char& green, unsigned char& blue);
    void generateGroundGrid(double resolution, double x_min, double x_max, double y_min, double y_max);
  };
}

#endif //VISION_NODE_H
