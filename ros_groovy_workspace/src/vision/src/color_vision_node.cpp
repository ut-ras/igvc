#include "vision/color_vision_node.h"

namespace vision
{
  ColorVisionNode::ColorVisionNode(const ros::NodeHandle& nh) :
      m_nh(nh), m_tf_listener(ros::Duration(30.0))
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("base_frame_id", m_base_frame_id, std::string("/base_link"));

    m_nh.param("resolution", m_resolution, 0.05);
    m_nh.param("x_min", m_x_min, 0.0);
    m_nh.param("x_max", m_x_max, 2.0);
    m_nh.param("y_min", m_y_min, -1.0);
    m_nh.param("y_max", m_y_max, 1.0);

    m_nh.param("sample_x_min", m_sample_x_min, 0.0);
    m_nh.param("sample_x_max", m_sample_x_max, 0.75);
    m_nh.param("sample_y_min", m_sample_y_min, -0.4);
    m_nh.param("sample_y_max", m_sample_y_max, 0.4);
    m_nh.param("sample_period", m_sample_period, 1.0);

    m_nh.param("initial_sample_x_min", m_initial_sample_x_min, 0.0);
    m_nh.param("initial_sample_x_max", m_initial_sample_x_max, 3.0);
    m_nh.param("initial_sample_y_min", m_initial_sample_y_min, -0.6);
    m_nh.param("initial_sample_y_max", m_initial_sample_y_max, 0.6);

    m_nh.param("known_bad_h_min", m_known_bad_h_min, 0);
    m_nh.param("known_bad_h_max", m_known_bad_h_max, 255);
    m_nh.param("known_bad_s_min", m_known_bad_s_min, 0);
    m_nh.param("known_bad_s_max", m_known_bad_s_max, 30);
    m_nh.param("known_bad_v_min", m_known_bad_v_min, 225);
    m_nh.param("known_bad_v_max", m_known_bad_v_max, 255);

    m_nh.param("std_dev_factor", m_std_dev_factor, 2.5);
    m_nh.param("num_color_regions", m_num_color_regions, 50);
    m_nh.param("h_expansion", m_h_expansion, 5);
    m_nh.param("s_expansion", m_s_expansion, 5);
    m_nh.param("v_expansion", m_v_expansion, 5);

    m_nh.param("publish_debug_images", m_publish_debug_images, true);

    int subscriber_queue_size = 1;
    m_nh.param("subscriber_queue_size", subscriber_queue_size, 1);

    generateGroundGrid();
    generateSampleRegion(true);
    m_first_sample = true;

    m_have_clouds = false;
    m_image_coordinate_lists_initialized = false;

    ros::NodeHandle ns_nh;
    sensor_msgs::CameraInfoConstPtr camera_info_ptr;
    while(!camera_info_ptr && ros::ok())
    {
      camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", ns_nh, ros::Duration(1.0));
      ROS_WARN_THROTTLE(1.0, "Waiting for camera info");
    }
    m_cam_model.fromCameraInfo(camera_info_ptr);

    m_ground_cloud_pub = ns_nh.advertise<sensor_msgs::PointCloud2>("ground", 1, true);
    m_obstacle_cloud_pub = ns_nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1, true);
    m_image_sub = ns_nh.subscribe<sensor_msgs::Image>("image_raw", subscriber_queue_size, boost::bind(&ColorVisionNode::imageCallback, this, _1));

    if(m_publish_debug_images)
    {
      m_initially_sampled_region_pub = ns_nh.advertise<sensor_msgs::Image>("initially_sampled_region", 1, true);
      m_sampled_region_pub = ns_nh.advertise<sensor_msgs::Image>("sampled_region", 1, true);
      m_thresholded_image_pub = ns_nh.advertise<sensor_msgs::Image>("thresholded_image", 1, true);
    }

    m_last_sample_time = ros::Time(0);
  }

  ColorVisionNode::~ColorVisionNode()
  {
  }

  bool ColorVisionNode::isKnownObstacleColor(cv::Vec3b color)
  {
    return (color.val[0] >= m_known_bad_h_min) && (color.val[0] <= m_known_bad_h_max) && (color.val[1] >= m_known_bad_s_min) && (color.val[1] <= m_known_bad_s_max) && (color.val[2] >= m_known_bad_v_min) && (color.val[2] <= m_known_bad_v_max);
  }

  bool ColorVisionNode::transformCloudToCamera(std_msgs::Header image_header, pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<cv::Point>& image_points)
  {
    cloud.header.stamp = image_header.stamp;
    if(!m_tf_listener.waitForTransform(image_header.frame_id, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.05), ros::Duration(0.001)))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Transform between " << image_header.frame_id << " and " << cloud.header.frame_id << " at time " << cloud.header.stamp << " failed!");
      return false;
    }
    pcl::PointCloud<pcl::PointXYZ> camera_cloud;
    pcl_ros::transformPointCloud(image_header.frame_id, cloud, camera_cloud, m_tf_listener);

    image_points.resize(cloud.size());
    for(unsigned int i = 0; i < cloud.size(); i++)
    {
      cv::Point3d cloud_point(camera_cloud.points[i].x, camera_cloud.points[i].y, camera_cloud.points[i].z);
      image_points[i] = m_cam_model.project3dToPixel(cloud_point);
    }

    return true;
  }

  void ColorVisionNode::generateGroundGrid()
  {
    m_ground_grid.points.clear();
    for(double x = m_x_min; x <= m_x_max; x += m_resolution)
    {
      for(double y = m_y_min; y <= m_y_max; y += m_resolution)
      {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = 0;
        m_ground_grid.points.push_back(point);
      }
    }
    m_ground_grid.header.frame_id = m_base_frame_id;
    m_ground_grid.height = 1;
    m_ground_grid.width = m_ground_grid.points.size();
  }

  void ColorVisionNode::generateSampleRegion(bool initial)
  {
    m_sample_boundaries.points.clear();
    if(initial)
    {
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_initial_sample_x_min, m_initial_sample_y_min, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_initial_sample_x_min, m_initial_sample_y_max, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_initial_sample_x_max, m_initial_sample_y_max, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_initial_sample_x_max, m_initial_sample_y_min, 0));
    }
    else
    {
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_sample_x_min, m_sample_y_min, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_sample_x_min, m_sample_y_max, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_sample_x_max, m_sample_y_max, 0));
      m_sample_boundaries.points.push_back(pcl::PointXYZ(m_sample_x_max, m_sample_y_min, 0));
    }
    m_sample_boundaries.header.frame_id = m_base_frame_id;
    m_sample_boundaries.height = 1;
    m_sample_boundaries.width = m_sample_boundaries.points.size();

    ROS_INFO("Setting sample bounds to (%g,%g) (%g,%g) (%g,%g) (%g,%g) (%s)", m_sample_boundaries.points[0].x, m_sample_boundaries.points[0].y, m_sample_boundaries.points[1].x, m_sample_boundaries.points[1].y, m_sample_boundaries.points[2].x, m_sample_boundaries.points[2].y, m_sample_boundaries.points[3].x, m_sample_boundaries.points[3].y,
        initial? "true" : "false");
  }

  void ColorVisionNode::drawRegion(cv::Mat mat, cv::vector<cv::Point> contour, CvScalar line_color)
  {
    for(unsigned int i = 0; i < contour.size(); i++)
    {
      cv::line(mat, contour[i], contour[(i + 1) % contour.size()], line_color, 2);
    }
  }

  bool ColorVisionNode::isOnImage(cv::Mat mat, unsigned int x, unsigned int y)
  {
    return (x < mat.cols) && (y < mat.rows);
  }

  void ColorVisionNode::updateColorRegions(cv::Mat mat, cv::vector<cv::Point> contour)
  {
    //find roi
    unsigned int min_x = std::numeric_limits<unsigned int>::max();
    unsigned int min_y = std::numeric_limits<unsigned int>::max();
    unsigned int max_x = 0;
    unsigned int max_y = 0;
    for(unsigned int i = 0; i < contour.size(); i++)
    {
      if(contour[i].x < min_x)
      {
        min_x = contour[i].x;
      }
      if(contour[i].y < min_y)
      {
        min_y = contour[i].y;
      }
      if(contour[i].x > max_x)
      {
        max_x = contour[i].x;
      }
      if(contour[i].y > max_y)
      {
        max_y = contour[i].y;
      }
    }

    //generate mask
    cv::Mat mask(mat.rows, mat.cols, CV_8U, cv::Scalar(0));

    const cv::Point* contours[1] = {&contour[0]};
    int contours_n[1] = {contour.size()};

    cv::fillPoly(mask, contours, contours_n, 1, cv::Scalar(255));

    //gather all the pixels in the polygon
//    std::vector<cv::Vec3b> color_palette;
    for(unsigned int y = min_y; y <= max_y; y++)
    {
      for(unsigned int x = min_x; x <= max_x; x++)
      {
        if(!isOnImage(mat, x, y))
        {
          ROS_WARN_ONCE("Some parts of the sample space are not on the image!");
          continue;
        }

        unsigned char val = mask.at<unsigned char>(y, x); //
        if(val > 0)
        {
          cv::Vec3b rgb_color = mat.at<cv::Vec3b>(y, x);
          cv::Vec3b hsv_color = rgb2hsv(rgb_color);
          if(!isKnownObstacleColor(hsv_color)) //don't look at colors we know are bad
          {
            m_color_palette.insert(hsv_color);
          }
          else
          {
            ROS_DEBUG_THROTTLE(1.0, "Bad color excluded!");
          }
        }
      }
    }

    if(m_color_palette.size() == 0)
    {
      ROS_WARN_THROTTLE(1.0, "Color palette is empty!");
      return;
    }

    //cluster into many smaller ranges to increase identification fidelity
    cv::Mat p = cv::Mat::zeros(m_color_palette.size(), 3, CV_32F);

    int p_idx = 0;
    for(ColorPaletteIterator iter = m_color_palette.begin(); iter != m_color_palette.end(); iter++)
    {
      p.at<float>(p_idx, 0) = (float) iter->val[0] / 255.0f;
      p.at<float>(p_idx, 1) = (float) iter->val[1] / 255.0f;
      p.at<float>(p_idx, 2) = (float) iter->val[2] / 255.0f;
      p_idx++;
    }
//    for(int i = 0; i < m_color_palette.size(); i++)
//    {
//      p.at<float>(i, 0) = (float) m_color_palette[i].val[0] / 255.0f;
//      p.at<float>(i, 1) = (float) m_color_palette[i].val[1] / 255.0f;
//      p.at<float>(i, 2) = (float) m_color_palette[i].val[2] / 255.0f;
//    }

    //(re)cluster the color set into K groups
    cv::Mat best_labels, centers, clustered;
    int K = m_num_color_regions;
    cv::kmeans(p, K, best_labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

    m_color_regions.clear();
    m_color_regions.resize(K);
    p_idx = 0;
    for(ColorPaletteIterator iter = m_color_palette.begin(); iter != m_color_palette.end(); iter++)
    {
      int cluster_index = best_labels.at<int>(0, p_idx);

      m_color_regions[cluster_index].addProbableColor(*iter, m_std_dev_factor);
      p_idx++;
    }

//    for(int i = 0; i < color_palette.size(); i++)
//    {
//      int cluster_index = best_labels.at<int>(0, i);
//      m_color_regions[cluster_index].addColor(color_palette[i]);
//    }

    //add more leeway
    if(m_h_expansion > 0 || m_s_expansion > 0 || m_v_expansion > 0)
    {
      for(unsigned int i = 0; i < m_color_regions.size(); i++)
      {
        m_color_regions[i].expand(m_h_expansion, m_s_expansion, m_v_expansion);
      }
    }
  }

  bool ColorVisionNode::isGroundColor(cv::Vec3b color)
  {
    for(unsigned int j = 0; j < m_color_regions.size(); j++)
    {
      if(m_color_regions[j].contains(color))
      {
        return true;
      }
    }
    return false;
  }

  void ColorVisionNode::thresholdImage(cv::Mat rgb_mat, std::vector<ColorRegion> regions, cv::Mat& thresholded)
  {
    cv::Mat hsv_mat;
    cvtColor(rgb_mat, hsv_mat, CV_RGB2HSV);

    thresholded = cv::Mat(hsv_mat.rows, hsv_mat.cols, CV_8U, cv::Scalar(255));
    for(unsigned int i = 0; i < hsv_mat.rows * hsv_mat.cols; i++)
    {
      cv::Vec3b color = hsv_mat.at<cv::Vec3b>(i);
      if(isGroundColor(color))
      {
        thresholded.at<unsigned char>(i) = 0;
      }
    }
  }

  cv::Vec3b ColorVisionNode::rgb2hsv(cv::Vec3b rgb)
  {
    unsigned char r = rgb.val[0];
    unsigned char g = rgb.val[1];
    unsigned char b = rgb.val[2];

    unsigned char rgbMin, rgbMax, h, s, v;

    rgbMin = r < g? (r < b? r : b) : (g < b? g : b);
    rgbMax = r > g? (r > b? r : b) : (g > b? g : b);

    v = rgbMax;
    if(v == 0)
    {
      h = 0;
      s = 0;
      return cv::Vec3b(h, s, v);
    }

    s = 255 * long(rgbMax - rgbMin) / v;
    if(s == 0)
    {
      h = 0;
      return cv::Vec3b(h, s, v);
    }

    if(rgbMax == r)
    {
      h = 0 + 43 * (g - b) / (rgbMax - rgbMin);
    }
    else if(rgbMax == g)
    {
      h = 85 + 43 * (b - r) / (rgbMax - rgbMin);
    }
    else
    {
      h = 171 + 43 * (r - g) / (rgbMax - rgbMin);
    }

    return cv::Vec3b(h, s, v);
  }

  void ColorVisionNode::classifyGroundGrid(cv::Mat& rgb_mat, std_msgs::Header image_header)
  {
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;
    for(unsigned int i = 0; i < m_ground_image_points.size(); i++)
    {
      if(!isOnImage(rgb_mat, m_ground_image_points[i].x, m_ground_image_points[i].y))
      {
        continue;
      }
      cv::Vec3b rgb_color = rgb_mat.at<cv::Vec3b>(m_ground_image_points[i].y, m_ground_image_points[i].x);
      cv::Vec3b hsv_color = rgb2hsv(rgb_color);

      pcl::PointXYZRGB point;
      point.x = m_ground_grid[i].x;
      point.y = m_ground_grid[i].y;
      point.z = m_ground_grid[i].z;
      point.r = rgb_color.val[0];
      point.g = rgb_color.val[1];
      point.b = rgb_color.val[2];
      if(isGroundColor(hsv_color))
      {
        ground_cloud.points.push_back(point);
      }
      else
      {
        obstacle_cloud.points.push_back(point);
//        std::cerr << (int) hsv_color.val[0] << ", " << (int)  hsv_color.val[1] << ", " << (int)  hsv_color.val[2] << std::endl;
      }
    }

    sensor_msgs::PointCloud2 ground_cloud_msg;
    pcl::toROSMsg(ground_cloud, ground_cloud_msg);
    ground_cloud_msg.header.frame_id = m_ground_grid.header.frame_id;
    ground_cloud_msg.header.stamp = image_header.stamp;
    m_ground_cloud_pub.publish(ground_cloud_msg);

    sensor_msgs::PointCloud2 obstacle_cloud_msg;
    pcl::toROSMsg(obstacle_cloud, obstacle_cloud_msg);
    obstacle_cloud_msg.header.frame_id = m_ground_grid.header.frame_id;
    obstacle_cloud_msg.header.stamp = image_header.stamp;
    m_obstacle_cloud_pub.publish(obstacle_cloud_msg);
  }

  void ColorVisionNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
//    if(m_first_sample)
//    {
//      ROS_INFO_ONCE("(first) Getting contour from (%g,%g) (%g,%g) (%g,%g) (%g,%g)", m_sample_boundaries.points[0].x, m_sample_boundaries.points[0].y, m_sample_boundaries.points[1].x, m_sample_boundaries.points[1].y, m_sample_boundaries.points[2].x, m_sample_boundaries.points[2].y, m_sample_boundaries.points[3].x, m_sample_boundaries.points[3].y);
//    }
//    else
//    {
//      ROS_INFO_ONCE("Getting contour from (%g,%g) (%g,%g) (%g,%g) (%g,%g)", m_sample_boundaries.points[0].x, m_sample_boundaries.points[0].y, m_sample_boundaries.points[1].x, m_sample_boundaries.points[1].y, m_sample_boundaries.points[2].x, m_sample_boundaries.points[2].y, m_sample_boundaries.points[3].x, m_sample_boundaries.points[3].y);
//    }

    if(!m_image_coordinate_lists_initialized)
    {
      if(!transformCloudToCamera(image->header, m_ground_grid, m_ground_image_points) || !transformCloudToCamera(image->header, m_sample_boundaries, m_sample_area_contour))
      {
        return;
      }
      m_image_coordinate_lists_initialized = true;
    }

    sensor_msgs::Image ros_img = *image;
    cv_bridge::CvImagePtr bridge_img = cv_bridge::toCvCopy(ros_img, ros_img.encoding);
    cv::Mat rgb_mat = bridge_img->image;

    if((ros::Time::now() - m_last_sample_time) > ros::Duration(m_sample_period))
    {
      updateColorRegions(rgb_mat, m_sample_area_contour);
    }

    classifyGroundGrid(rgb_mat, image->header);

    if(m_publish_debug_images)
    {
      CvScalar line_color;
      line_color.val[0] = 255;
      line_color.val[1] = 255;
      line_color.val[2] = 0;
      drawRegion(bridge_img->image, m_sample_area_contour, line_color);

      if(m_first_sample)
      {
//        ROS_INFO_ONCE("Contour points are (%d,%d) (%d,%d) (%d,%d) (%d,%d)", m_sample_area_contour[0].x, m_sample_area_contour[0].y, m_sample_area_contour[1].x, m_sample_area_contour[1].y, m_sample_area_contour[2].x, m_sample_area_contour[2].y, m_sample_area_contour[3].x, m_sample_area_contour[3].y);
        m_initial_sample_image = *bridge_img->toImageMsg();
      }
      else
      {
//        ROS_INFO_ONCE("Contour points are (%d,%d) (%d,%d) (%d,%d) (%d,%d)", m_sample_area_contour[0].x, m_sample_area_contour[0].y, m_sample_area_contour[1].x, m_sample_area_contour[1].y, m_sample_area_contour[2].x, m_sample_area_contour[2].y, m_sample_area_contour[3].x, m_sample_area_contour[3].y);
        m_sampled_region_pub.publish(bridge_img->toImageMsg());
      }

//      cv::Mat thresholded;
//      thresholdImage(rgb_mat, m_color_regions, thresholded);
    }

    if(m_first_sample)
    {
      generateSampleRegion(false);
      transformCloudToCamera(image->header, m_sample_boundaries, m_sample_area_contour);
      m_first_sample = false;
    }
  }

  void ColorVisionNode::spin()
  {
    ROS_INFO("ColorVisionNode started.");
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      m_initial_sample_image.header.stamp = ros::Time::now();
      m_initially_sampled_region_pub.publish(m_initial_sample_image);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_vision_node");
  ros::NodeHandle nh("~");

  vision::ColorVisionNode node(nh);
  node.spin();

  return 0;
}
