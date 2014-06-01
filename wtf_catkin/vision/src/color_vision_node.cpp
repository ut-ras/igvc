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
    m_nh.param("known_bad_h_max", m_known_bad_h_max, 0);
    m_nh.param("known_bad_s_min", m_known_bad_s_min, 0);
    m_nh.param("known_bad_s_max", m_known_bad_s_max, 0);
    m_nh.param("known_bad_v_min", m_known_bad_v_min, 0);
    m_nh.param("known_bad_v_max", m_known_bad_v_max, 0);

    m_nh.param("std_dev_factor", m_std_dev_factor, 2.5);
    m_nh.param("num_color_regions", m_num_color_regions, 50);
    m_nh.param("h_expansion", m_h_expansion, 5);
    m_nh.param("s_expansion", m_s_expansion, 5);
    m_nh.param("v_expansion", m_v_expansion, 5);

    m_nh.param("only_use_initial_colors", m_only_use_initial_colors, true);

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
    return (color.val[0] > m_known_bad_h_min) && (color.val[0] < m_known_bad_h_max) && (color.val[1] > m_known_bad_s_min) && (color.val[1] < m_known_bad_s_max) && (color.val[2] > m_known_bad_v_min) && (color.val[2] < m_known_bad_v_max);
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

  bool ColorVisionNode::isOnImage(cv::Mat mat, int x, int y)
  {
    return (x >= 0) && (y >= 0) && (x < mat.cols) && (y < mat.rows);
  }

  void ColorVisionNode::updateColorRegions(cv::Mat mat, cv::vector<cv::Point> contour)
  {
    //find roi
    int min_x = std::numeric_limits<unsigned int>::max();
    int min_y = std::numeric_limits<unsigned int>::max();
    int max_x = 0;
    int max_y = 0;
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
    for(int y = min_y; y <= max_y; y++)
    {
      for(int x = min_x; x <= max_x; x++)
      {
        if(!isOnImage(mat, x, y))
        {
          ROS_WARN_ONCE("Some parts of the sample space are not on the image!");
          continue;
        }

        assert(x >= 0);
        assert(y >= 0);

        unsigned char val = mask.at<unsigned char>(y, x);
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
    for(int i = 0; i < hsv_mat.rows * hsv_mat.cols; i++)
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

  bool lineIntersection(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, cv::Point& intersection) 
  {
    float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
    float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;
     
    float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // If d is zero, there is no intersection
    if (d == 0) return false;
     
    // Get the x and y
    float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
    float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
    float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;
     
    // Check if the x and y coordinates are within both lines
    if ( x < std::min(x1, x2) || x > std::max(x1, x2) ||
    x < std::min(x3, x4) || x > std::max(x3, x4) ) return false;
    if ( y < std::min(y1, y2) || y > std::max(y1, y2) ||
    y < std::min(y3, y4) || y > std::max(y3, y4) ) return false;
     
    // Return the point of intersection
    intersection.x = x;
    intersection.y = y;
    return true;
  }

  enum Edge
  {
    LEFT,
    RIGHT,
    TOP,
    BOTTOM
  };

  void ColorVisionNode::clampContourToImage(cv::Mat& mat, cv::vector<cv::Point>& contour)
  {
    int min_x = std::numeric_limits<unsigned int>::max();
    int min_y = std::numeric_limits<unsigned int>::max();
    int max_x = 0;
    int max_y = 0;
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
      ROS_INFO("Parsing %d,%d", contour[i].x, contour[i].y);
    }

    // cv::Mat mask(max_y-min_y, max_x=min_x, CV_8U, cv::Scalar(0));

    // cv::vector<cv::Point> shifted_contour = contour;;
    // for(unsigned int i = 0; i < shifted_contour.size(); i++)
    // {
    //   shifted_contour[i].x -= min_x;
    //   shifted_contour[i].y -= min_y;
    // }

    // const cv::Point* contours[1] = {&contour[0]};
    // int contours_n[1] = {contour.size()};

    // std::vector<Point>& polyContour
    // cv::fillPoly(mask, contours, contours_n, 1, cv::Scalar(255));
    // approxPolyDP(mask, polyContour, accuracy, true);

  }

  // void ColorVisionNode::clampContourToImage(cv::Mat& mat, cv::vector<cv::Point>& contour)
  // {
  //   std::cerr << "clamping!" << std::endl;
  //   cv::Point ul(0, 0);
  //   cv::Point ur(mat.cols-1, 0);
  //   cv::Point ll(0, mat.rows-1);
  //   cv::Point lr(mat.cols-1, mat.rows-1);

  //   assert(contour.size() == 4);// TODO: REMOVE!

  //   cv::vector<cv::Point> clamped_contour;
  //   for(unsigned int i = 0; i < contour.size(); i++)
  //   {
  //     std::cerr << "wat" << i << std::endl;
  //     if(isOnImage(mat, contour[i].x, contour[i].y))
  //     {
  //       clamped_contour.push_back(contour[i]);
  //     }
  //     else //not on the image, replace with two intersection points
  //     {
  //       unsigned int p1_idx_1 = (i == 0)? contour.size() - 1 : i - 1;
  //       unsigned int p1_idx_2 = i;
  //       unsigned int p2_idx_1 = (i == contour.size() - 1)? 0 : i + 1;
  //       unsigned int p2_idx_2 = i;
  //       cv::Point p1, p2;

  //       int num_to_skip = 0;
  //       while(!isOnImage(mat, contour[p1_idx_1].x, contour[p1_idx_1].y))
  //       {
  //         p1_idx_2 = p1_idx_1;
  //         p1_idx_1 = (p1_idx_1 == 0)? contour.size() - 1 : p1_idx_1 - 1;
  //         ROS_INFO("1 Skipping to %d->%d (%d) ", p1_idx_1, p1_idx_2, contour.size());
  //         assert(p1_idx_1 != i); //looped => something horrible happened
  //       }
  //       while(!isOnImage(mat, contour[p2_idx_1].x, contour[p2_idx_1].y))
  //       {
  //         int last_p2_1 = p2_idx_1;
  //         int last_p2_2 = p2_idx_2;
  //         p2_idx_2 = p2_idx_1; // 2.2 <= 1
  //         p2_idx_1 = (p2_idx_1 == contour.size() - 1)? 0 : p2_idx_1 + 1; // 2.1 <= 1+1 = 2
  //         ROS_INFO("2 Skipping from %d->%d to %d->%d (%d)", last_p2_1, last_p2_2, p2_idx_1, p2_idx_2, contour.size());
  //         assert(last_p2_1 != p2_idx_1);
  //         //2 Skipping to 1->0 (4)
  //         //p2_idx_1 = 1
  //         //p2_idx_2 = 0

  //         assert(p2_idx_1 != i); //looped => something horrible happened
  //         num_to_skip++;
  //       }

  //       Edge edge1;
  //       cv::Point intersection;
  //       if(lineIntersection(contour[p1_idx_1], contour[p1_idx_2], ul, ur, intersection))
  //       {
  //         p1 = intersection;
  //         edge1 = TOP;
  //       }
  //       else if(lineIntersection(contour[p1_idx_1], contour[p1_idx_2], ur, lr, intersection))
  //       {
  //         p1 = intersection;
  //         edge1 = RIGHT;
  //       }
  //       else if(lineIntersection(contour[p1_idx_1], contour[p1_idx_2], lr, ll, intersection))
  //       {
  //         p1 = intersection;
  //         edge1 = BOTTOM;
  //       }
  //       else if(lineIntersection(contour[p1_idx_1], contour[p1_idx_2], ll, ul, intersection))
  //       {
  //         p1 = intersection;
  //         edge1 = LEFT;
  //       }
  //       else
  //       {
  //         assert(0); // this should never happen
  //       }

  //       Edge edge2;
  //       if(lineIntersection(contour[p2_idx_1], contour[p2_idx_2], ul, ur, intersection))
  //       {
  //         p2 = intersection;
  //         edge2 = TOP;
  //       }
  //       else if(lineIntersection(contour[p2_idx_1], contour[p2_idx_2], ur, lr, intersection))
  //       {
  //         p2 = intersection;
  //         edge2 = RIGHT;
  //       }
  //       else if(lineIntersection(contour[p2_idx_1], contour[p2_idx_2], lr, ll, intersection))
  //       {
  //         p2 = intersection;
  //         edge2 = BOTTOM;
  //       }
  //       else if(lineIntersection(contour[p2_idx_1], contour[p2_idx_2], ll, ul, intersection))
  //       {
  //         p2 = intersection;
  //         edge2 = LEFT;
  //       }
  //       else
  //       {
  //         assert(0); // this should never happen
  //       }

  //       if(edge1 == edge2)
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(p2);
  //       }
  //       else if(edge1 == TOP && edge2 == LEFT)
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(ul);
  //         clamped_contour.push_back(p2);
  //         ROS_INFO("Area contained UL");
  //       }
  //       else if(edge1 == TOP && edge2 == RIGHT)
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(ur);
  //         clamped_contour.push_back(p2);
  //         ROS_INFO("Area contained UR");
  //       }
  //       else if(edge1 == BOTTOM && edge2 == LEFT)
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(ll);
  //         clamped_contour.push_back(p2);
  //         ROS_INFO("Area contained LL");
  //       }
  //       else if(edge1 == BOTTOM && edge2 == RIGHT)
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(lr);
  //         clamped_contour.push_back(p2);
  //         ROS_INFO("Area contained LR");
  //       }
  //       else if((edge1 == LEFT && edge2 == RIGHT) || (edge1 == RIGHT && edge2 == LEFT))
  //       {
  //         clamped_contour.push_back(p1);
  //         clamped_contour.push_back(lr);
  //         clamped_contour.push_back(p2);
  //         ROS_INFO("Area contained L");
  //       }
  //       else
  //       {
  //         assert(0); // this should never happen
  //       }

  //       i += num_to_skip;
  //     }
  //   }

  //   std::cerr << "Modified contour: ";
  //   for(unsigned int i = 0; i < clamped_contour.size(); i++)
  //   {
  //     std::cerr << "(" << clamped_contour[i].x << "," << clamped_contour[i].y << ")";
  //     if(i != (clamped_contour.size()-1))
  //     {
  //       std::cerr << ",";
  //     }
  //   }
  //   contour = clamped_contour;
  // }

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

    sensor_msgs::Image ros_img = *image;
    cv_bridge::CvImagePtr bridge_img = cv_bridge::toCvCopy(ros_img, ros_img.encoding);
    cv::Mat rgb_mat = bridge_img->image;

    if(!m_image_coordinate_lists_initialized)
    {
      if(!transformCloudToCamera(image->header, m_ground_grid, m_ground_image_points) || !transformCloudToCamera(image->header, m_sample_boundaries, m_sample_area_contour))
      {
        return;
      }
      clampContourToImage(rgb_mat, m_sample_area_contour);
      m_image_coordinate_lists_initialized = true;
    }

    if(m_first_sample || (((ros::Time::now() - m_last_sample_time) > ros::Duration(m_sample_period)) && !m_only_use_initial_colors))
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
      clampContourToImage(rgb_mat, m_sample_area_contour);
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
