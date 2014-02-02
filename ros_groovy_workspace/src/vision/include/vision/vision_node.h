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
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> TripleImagePolicy;

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
    std::string m_base_frame_id;

    ros::Publisher m_ground_cloud_pub;
    ros::Publisher m_obstacle_cloud_pub;
    message_filters::Synchronizer<TripleImagePolicy>* m_sync;

    bool m_have_clouds;

    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid;
    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid1;
    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid2;
    pcl::PointCloud<pcl::PointXYZRGB> m_ground_grid3;

    tf::TransformListener m_tf_listener;

    image_geometry::PinholeCameraModel m_cam_model_1;
    image_geometry::PinholeCameraModel m_cam_model_2;
    image_geometry::PinholeCameraModel m_cam_model_3;

    sensor_msgs::PointCloud2 m_obstacle_cloud_msg;
    sensor_msgs::PointCloud2 m_ground_cloud_msg;

    double colorDistance(CvScalar a, CvScalar b);
    void parseEncoding(CvScalar s, std::string encoding, unsigned char& red, unsigned char& green, unsigned char& blue);
    void generateGroundGrid(double resolution, double x_min, double x_max, double y_min, double y_max);
    void imageCallback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3);
  };
}

#endif //VISION_NODE_H
