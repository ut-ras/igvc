#ifndef CLOUD_ASSIMILATOR_H
#define CLOUD_ASSIMILATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <laser_geometry/laser_geometry.h>
#include <string.h>

namespace vision
{
  class CloudAssimilator
  {
  public:
    CloudAssimilator(const ros::NodeHandle& nh);
    ~CloudAssimilator();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;
    double m_neighborhood_radius;
    int m_min_neighbors;
    std::string m_sensor_frame_id;

    ros::Subscriber m_left_sub;
    ros::Subscriber m_right_sub;
    ros::Publisher m_cloud_pub;
    ros::Publisher m_scan_pub;

    tf::TransformListener m_tf_listener;

    bool m_have_new_left_cloud;
    bool m_have_new_right_cloud;

    pcl::PointCloud<pcl::PointXYZ> m_left_cloud;
    pcl::PointCloud<pcl::PointXYZ> m_right_cloud;

    ros::Time m_last_sample_time;

    void processClouds();

    void leftCallback(const sensor_msgs::PointCloud2ConstPtr& image);
    void rightCallback(const sensor_msgs::PointCloud2ConstPtr& image);
  };
}

#endif //CLOUD_ASSIMILATOR_H
