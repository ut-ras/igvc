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

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "pcl_conversions/pcl_conversions.h"

namespace vision
{
  class ColorRegion
  {
  public:
    cv::Vec3b min;
    cv::Vec3b max;
    bool initialized;
    double mean[3];
    double var[3];
    double num_colors_added;

    ColorRegion()
    {
      initialized = false;
    }

    void expand(unsigned char c0, unsigned char c1, unsigned char c2)
    {
      min.val[0] = (min.val[0] > c0) ? min.val[0] - c0 : 0;
      min.val[1] = (min.val[1] > c1) ? min.val[1] - c1 : 0;
      min.val[2] = (min.val[2] > c2) ? min.val[2] - c2 : 0;

      max.val[0] = ((255 - max.val[0]) > c0) ? max.val[0] + c0 : 255;
      max.val[1] = ((255 - max.val[1]) > c1) ? max.val[1] + c1 : 255;
      max.val[2] = ((255 - max.val[2]) > c2) ? max.val[2] + c2 : 255;
    }

    void print()
    {
      std::cerr << "c0:[" << (int) min.val[0] << "," << (int) max.val[0] << "], c1:[" << (int) min.val[1] << "," << (int) max.val[1] << "], c2:[" << (int) min.val[2] << "," << (int) max.val[2] << "]\n";
    }

    bool contains(cv::Vec3b color)
    {
      return (color.val[0] >= min.val[0]) && (color.val[0] <= max.val[0]) && (color.val[1] >= min.val[1]) && (color.val[1] <= max.val[1]) && (color.val[2] >= min.val[2]) && (color.val[2] <= max.val[2]);
    }

    void addProbableColor(cv::Vec3b color, double std_dev_factor)
    {
      if (!initialized)
      {
        mean[0] = color.val[0];
        mean[1] = color.val[1];
        mean[2] = color.val[2];
        var[0] = 0;
        var[1] = 0;
        var[2] = 0;
        num_colors_added = 1.0;
        initialized = true;
      }
      else
      {
        assert(num_colors_added > 0.0);
        if (num_colors_added >= 2.0)
        {
          var[0] = (num_colors_added - 2) / (num_colors_added - 1) * var[0] + (color.val[0] - mean[0]) * (color.val[0] - mean[0]) / num_colors_added;
          var[1] = (num_colors_added - 2) / (num_colors_added - 1) * var[1] + (color.val[1] - mean[1]) * (color.val[1] - mean[1]) / num_colors_added;
          var[2] = (num_colors_added - 2) / (num_colors_added - 1) * var[2] + (color.val[2] - mean[2]) * (color.val[2] - mean[2]) / num_colors_added;
        }
        mean[0] = (color.val[0] + (num_colors_added - 1) * mean[0]) / num_colors_added;
        mean[1] = (color.val[1] + (num_colors_added - 1) * mean[1]) / num_colors_added;
        mean[2] = (color.val[2] + (num_colors_added - 1) * mean[2]) / num_colors_added;
        num_colors_added += 1.0;
      }
      min.val[0] = mean[0];
      min.val[1] = mean[1];
      min.val[2] = mean[2];
      max = min;

      //      ROS_INFO("(%d,%d,%d) => (%g,%g), (%g,%g), (%g,%g)", color.val[0], color.val[1], color.val[2], mean[0], var[0], mean[1], var[1], mean[2], var[2]);

      expand(std_dev_factor * sqrt(var[0]), std_dev_factor * sqrt(var[1]), std_dev_factor * sqrt(var[2]));
      //      expand(std_dev_factor * var[0], std_dev_factor * var[1], std_dev_factor * var[2]);
    }

    void addColor(cv::Vec3b color)
    {
      if (!initialized)
      {
        min = color;
        max = color;
        initialized = true;
      }
      else
      {
        if (color.val[0] < min.val[0])
        {
          min.val[0] = color.val[0];
        }
        if (color.val[1] < min.val[1])
        {
          min.val[1] = color.val[1];
        }
        if (color.val[2] < min.val[2])
        {
          min.val[2] = color.val[2];
        }
        if (color.val[0] > max.val[0])
        {
          max.val[0] = color.val[0];
        }
        if (color.val[1] > max.val[1])
        {
          max.val[1] = color.val[1];
        }
        if (color.val[2] > max.val[2])
        {
          max.val[2] = color.val[2];
        }
      }
    }
  };

  class ColorVisionNode
  {
  public:
    ColorVisionNode(const ros::NodeHandle &nh);
    ~ColorVisionNode();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;
    std::string m_base_frame_id;
    double m_resolution;
    double m_x_min;
    double m_x_max;
    double m_y_min;
    double m_y_max;
    double m_sample_period;
    // double m_sample_x_min;
    // double m_sample_x_max;
    // double m_sample_y_min;
    // double m_sample_y_max;
    // double m_initial_sample_x_min;
    // double m_initial_sample_x_max;
    // double m_initial_sample_y_min;
    // double m_initial_sample_y_max;
    pcl::PointCloud<pcl::PointXYZ> m_sample_polygon;
    pcl::PointCloud<pcl::PointXYZ> m_initial_sample_polygon;
    int m_num_color_regions;
    bool m_only_use_initial_colors;

    int m_known_bad_h_min;
    int m_known_bad_s_min;
    int m_known_bad_v_min;
    int m_known_bad_h_max;
    int m_known_bad_s_max;
    int m_known_bad_v_max;

    double m_std_dev_factor;

    int m_h_expansion;
    int m_s_expansion;
    int m_v_expansion;

    bool m_first_sample;

    bool m_publish_debug_images;

    ros::Publisher m_ground_cloud_pub;
    ros::Publisher m_obstacle_cloud_pub;

    ros::Publisher m_initially_sampled_region_pub;
    ros::Publisher m_sampled_region_pub;
    ros::Publisher m_thresholded_image_pub;

    sensor_msgs::Image m_initial_sample_image;

    bool m_have_clouds;
    bool m_image_coordinate_lists_initialized;

    pcl::PointCloud<pcl::PointXYZ> m_ground_grid;
    pcl::PointCloud<pcl::PointXYZ> m_sample_boundaries;

    std::vector<cv::Point> m_ground_image_points;
    std::vector<cv::Point> m_sample_area_contour;

    struct color_compare
    {
      bool operator()(const cv::Vec3b &lhs, const cv::Vec3b &rhs) const
      {
        unsigned long lhs_val = (lhs.val[0] << 0) + (lhs.val[1] << 16) + (lhs.val[2] << 8);
        unsigned long rhs_val = (rhs.val[0] << 0) + (rhs.val[1] << 16) + (rhs.val[2] << 8);
        return lhs_val < rhs_val;
      }
    };
    typedef std::set<cv::Vec3b, color_compare> ColorPalette;
    typedef ColorPalette::iterator ColorPaletteIterator;
    ColorPalette m_color_palette;
    std::vector<ColorRegion> m_color_regions;

    tf::TransformListener m_tf_listener;

    ros::Subscriber m_image_sub;
    image_geometry::PinholeCameraModel m_cam_model;

    ros::Time m_last_sample_time;

    void drawRegion(cv::Mat mat, cv::vector<cv::Point> contour, CvScalar line_color);
    void updateColorRegions(cv::Mat mat, cv::vector<cv::Point> contour);
    void thresholdImage(cv::Mat mat, std::vector<ColorRegion> regions, cv::Mat &thresholded);
    bool transformGridToCamera(std::vector<sensor_msgs::Image> &images, pcl::PointCloud<pcl::PointXYZ> grid, std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds);
    void generateGroundGrid();
    void generateSampleRegion(bool initial);
    bool isKnownObstacleColor(cv::Vec3b color);
    bool isGroundColor(cv::Vec3b color);
    void classifyGroundGrid(cv::Mat &mat, std_msgs::Header image_header);
    bool transformCloudToCamera(std_msgs::Header image_header, pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<cv::Point> &image_points);
    bool isOnImage(cv::Mat mat, int x, int y);
    void clampContourToImage(cv::Mat &mat, cv::vector<cv::Point> &contour);
    cv::Vec3b rgb2hsv(cv::Vec3b rgb);

    void parsePoints(std::string point_string, pcl::PointCloud<pcl::PointXYZ>& points);
    void parseColors(std::string point_string, std::vector<cv::Vec3b> &colors);

    void imageCallback(const sensor_msgs::ImageConstPtr &image);
  };
}

#endif //COLOR_VISION_NODE_H
