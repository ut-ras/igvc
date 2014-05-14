#ifndef COLOR_VISION_NODE_H
#define COLOR_VISION_NODE_H

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
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

namespace vision
{
  class ColorRegion
  {
  public:
    cv::Vec3b min;
    cv::Vec3b max;

    bool isWithin(cv::Vec3b color)
    {
      return (color.val[0] >= min.val[0]) && (color.val[0] <= max.val[0]) && (color.val[1] >= min.val[1]) && (color.val[1] <= max.val[1]) && (color.val[2] >= min.val[2]) && (color.val[2] <= max.val[2]);
    }
  };

  class ColorVisionNode
  {
  public:
    ColorVisionNode(const ros::NodeHandle& nh);
    ~ColorVisionNode();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;
    double m_match_threshold;
    int m_lightness_threshold;
    int m_white_threshold;
    int m_num_cameras;
    std::string m_base_frame_id;
    double m_resolution;
    double m_x_min;
    double m_x_max;
    double m_y_min;
    double m_y_max;

    double m_sample_period;
    double m_sample_resolution;
    double m_sample_x_min;
    double m_sample_x_max;
    double m_sample_y_min;
    double m_sample_y_max;
    int m_num_color_regions;

    double m_max_image_time_lag;

    ros::Publisher m_ground_cloud_pub;
    ros::Publisher m_obstacle_cloud_pub;

    bool m_have_clouds;

    pcl::PointCloud<pcl::PointXYZ> m_ground_grid;
    pcl::PointCloud<pcl::PointXYZ> m_sample_grid;

    tf::TransformListener m_tf_listener;

    std::vector<ros::Subscriber> m_image_subs;
    std::vector<image_geometry::PinholeCameraModel> m_cam_models;

    sensor_msgs::PointCloud2 m_obstacle_cloud_msg;
    sensor_msgs::PointCloud2 m_ground_cloud_msg;

    std::vector<sensor_msgs::Image> m_images;
    std::vector<bool> m_images_valid;

    std::vector<std::vector<ColorRegion> > m_drivable_regions;

    ros::Time m_last_sample_time;

    bool sampleKnownGround(std::vector<sensor_msgs::Image>& images, std::vector<IplImage*> cv_images);

    bool allImagesValid();
    bool imagesSynced();

    void filterCloud(pcl::PointCloud<pcl::PointXYZ> in, pcl::PointCloud<pcl::PointXYZ>& out);

    void imageCallback(const sensor_msgs::ImageConstPtr& image, int idx);

    void processImages(std::vector<sensor_msgs::Image>& images);
    bool colorsMatch(std::vector<CvScalar> colors, std::vector<sensor_msgs::Image>& images, CvScalar& matched_color);
    bool transformGridToCameras(std::vector<sensor_msgs::Image>& images, pcl::PointCloud<pcl::PointXYZ> grid, std::vector<pcl::PointCloud<pcl::PointXYZ> >& clouds);
    double colorDistance(CvScalar a, CvScalar b);
    void parseEncoding(CvScalar s, std::string encoding, unsigned char& red, unsigned char& green, unsigned char& blue);
    void generateGroundGrid(double resolution, double x_min, double x_max, double y_min, double y_max, pcl::PointCloud<pcl::PointXYZ>& grid);
  };
}

#endif //COLOR_VISION_NODE_H
