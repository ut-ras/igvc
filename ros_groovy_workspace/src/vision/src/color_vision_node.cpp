#include "vision/color_vision_node.h"

namespace vision
{
  ColorVisionNode::ColorVisionNode(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("white_threshold", m_white_threshold, 10);
    m_nh.param("lightness_threshold", m_lightness_threshold, 180);
    m_nh.param("base_frame_id", m_base_frame_id, std::string("/base_link"));
    m_nh.param("num_cameras", m_num_cameras, 2);
    m_nh.param("max_image_time_lag", m_max_image_time_lag, 0.1);
    m_nh.param("match_threshold", m_match_threshold, 10.0);

    m_ground_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ground", 1, true);
    m_obstacle_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obstacles", 1, true);

    m_nh.param("resolution", m_resolution, 0.05);
    m_nh.param("x_min", m_x_min, 0.0);
    m_nh.param("x_max", m_x_max, 2.0);
    m_nh.param("y_min", m_y_min, -0.75);
    m_nh.param("y_max", m_y_max, 0.75);

    m_nh.param("sample_resolution", m_sample_resolution, 0.01);
    m_nh.param("sample_x_min", m_sample_x_min, 0.0);
    m_nh.param("sample_x_max", m_sample_x_max, 2.0);
    m_nh.param("sample_y_min", m_sample_y_min, -1.0);
    m_nh.param("sample_y_max", m_sample_y_max, 1.0);
    m_nh.param("sample_period", m_sample_period, 2.0);

    generateGroundGrid(m_resolution, m_x_min, m_x_max, m_y_min, m_y_max, m_ground_grid);
    generateGroundGrid(m_sample_resolution, m_sample_x_min, m_sample_x_max, m_sample_y_min, m_sample_y_max, m_sample_grid);

    m_have_clouds = false;

    m_image_subs.resize(m_num_cameras);
    for(unsigned int i = 0; i < m_num_cameras; i++)
    {
      std::stringstream image_topic, info_topic, frame;
      image_topic << "/image_" << i;
      info_topic << "/camera_info_" << i;

      sensor_msgs::CameraInfoConstPtr camera_info_ptr;
      while(!camera_info_ptr && ros::ok())
      {
        camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic.str(), m_nh, ros::Duration(0.1));
        ROS_WARN_THROTTLE(1.0, "Waiting for camera #%d's info", i);
      }
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_ptr);
      m_cam_models.push_back(model);

      ros::SubscribeOptions input_options = ros::SubscribeOptions::create<sensor_msgs::Image>(image_topic.str(), 1, boost::bind(&ColorVisionNode::imageCallback, this, _1, i), ros::VoidPtr(), m_nh.getCallbackQueue());
      m_image_subs.at(i) = m_nh.subscribe(input_options);
      ROS_INFO_STREAM("Subscribed to " << m_image_subs.at(i).getTopic());
    }

    m_last_sample_time = ros::Time(0);
  }

  ColorVisionNode::~ColorVisionNode()
  {
  }

  void ColorVisionNode::generateGroundGrid(double resolution, double x_min, double x_max, double y_min, double y_max, pcl::PointCloud<pcl::PointXYZ>& grid)
  {
    grid.points.clear();
    for(double x = x_min; x <= x_max; x += resolution)
    {
      for(double y = y_min; y <= y_max; y += resolution)
      {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = 0;
        grid.points.push_back(point);
      }
    }
    grid.header.frame_id = m_base_frame_id;
    grid.height = 1;
    grid.width = grid.points.size();
  }

  double ColorVisionNode::colorDistance(CvScalar a, CvScalar b)
  {
    double dist = 0;
    for(int i = 0; i < 4; i++)
    {
      double diff = a.val[i] - b.val[i];
      dist += diff * diff;
    }
    return dist;
  }

  void ColorVisionNode::parseEncoding(CvScalar s, std::string encoding, unsigned char& red, unsigned char& green, unsigned char& blue)
  {
    if(!encoding.compare("rgb") || !encoding.compare("rgb8"))
    {
      red = s.val[0];
      green = s.val[1];
      blue = s.val[2];
    }
    else if(!encoding.compare("bgr") || !encoding.compare("bgr8"))
    {
      red = s.val[2];
      green = s.val[1];
      blue = s.val[0];
    }
    else if(!encoding.compare("mono") || !encoding.compare("mono8"))
    {
      red = s.val[0];
      green = s.val[0];
      blue = s.val[0];
    }
    else
    {
      ROS_ERROR("Unknown image encoding: %s", encoding.c_str());
    }
  }

  bool ColorVisionNode::transformGridToCameras(std::vector<sensor_msgs::Image>& images, pcl::PointCloud<pcl::PointXYZ> grid, std::vector<pcl::PointCloud<pcl::PointXYZ> >& clouds)
  {
    for(unsigned int i = 0; i < images.size(); i++)
    {
      grid.header.stamp = images[i].header.stamp;
      if(!m_tf_listener.waitForTransform(images[i].header.frame_id, grid.header.frame_id, grid.header.stamp, ros::Duration(0.05), ros::Duration(0.001)))
      {
        ROS_ERROR_THROTTLE(1.0, "Transform between %s and %s failed!", images[i].header.frame_id.c_str(), grid.header.frame_id.c_str());
        return false;
      }
      pcl::PointCloud<pcl::PointXYZ> camera_cloud;
      pcl_ros::transformPointCloud(images[i].header.frame_id, grid, camera_cloud, m_tf_listener);
      clouds.push_back(camera_cloud);
    }

    return true;
  }

  bool ColorVisionNode::colorsMatch(std::vector<CvScalar> colors, std::vector<sensor_msgs::Image>& images, CvScalar& matched_color)
  {
    if(colors.size() == 1)
    {
      return false; //can't match with only one point
    }

    for(unsigned int i = 0; i < colors.size(); i++)
    {
      double distance = colorDistance(colors[i], colors[colors.size() - 1]); //only need to compare everything to the last point, since we're doing this on every push
      if(distance < m_match_threshold)
      {
        unsigned char r1, b1, g1, r2, b2, g2;
        parseEncoding(colors[i], images[i].encoding, r1, g1, b1);
        parseEncoding(colors[colors.size() - 1], images[colors.size() - 1].encoding, r2, g2, b2);
        matched_color.val[0] = (r1 + r2) / 2;
        matched_color.val[1] = (g1 + g2) / 2;
        matched_color.val[2] = (b1 + b2) / 2;
        return true;
      }
    }

    return false;
  }

  void ColorVisionNode::filterCloud(pcl::PointCloud<pcl::PointXYZ> in, pcl::PointCloud<pcl::PointXYZ>& out)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr = in.makeShared();

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(in_cloud_ptr);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = m_resolution * 5.0;
    for(unsigned int i = 0; i < in.size(); i++)
    {
      int num_neighbors = kdtree.radiusSearch(in.points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(num_neighbors > 5)
      {
        out.points.push_back(in.points[i]);
      }
    }
    out.height = 1;
    out.width = out.points.size();

    //    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //    sor.setInputCloud(obstacle_cloud_ptr);
    //    sor.setMeanK(3);
    //    sor.setStddevMulThresh(1.0);
    //    sor.filter(obstacle_cloud_filtered);

    //    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    //    outrem.setInputCloud(obstacle_cloud_ptr);
    //    outrem.setRadiusSearch(3000.0);
    //    outrem.setMinNeighborsInRadius(1);
    //    outrem.filter(obstacle_cloud_filtered);

    if(out.size() / in_cloud_ptr->size() < 0.5)
    {
      ROS_WARN_THROTTLE(1.0, "Filtered all but %d/%d (%g%%) of obstacle points", (int) out.size(), (int) in_cloud_ptr->size(), 100.0 * out.size() / in_cloud_ptr->size());
    }
  }

  bool ColorVisionNode::sampleKnownGround(std::vector<sensor_msgs::Image>& images, std::vector<IplImage*> cv_images)
  {
    ROS_INFO("Sampling ground patch!");
    std::vector<pcl::PointCloud<pcl::PointXYZ> > camera_clouds;
    if(!transformGridToCameras(images, m_sample_grid, camera_clouds))
    {
      ROS_ERROR_THROTTLE(1.0, "Unable to transform cameras!");
      return false; //bad transform!
    }

    //gather color information from all cameras
    std::vector<pcl::PointCloud<pcl::PointXYZ> > colors; //x==r, y==g, z==b
    colors.resize(camera_clouds.size());
    for(unsigned int j = 0; j < camera_clouds.size(); j++)
    {
      for(unsigned int i = 0; i < m_sample_grid.size(); i++)
      {
        cv::Point3d cloud_point(camera_clouds[j].points[i].x, camera_clouds[j].points[i].y, camera_clouds[j].points[i].z);
        cv::Point2d image_point = m_cam_models[j].project3dToPixel(cloud_point);

        if(!(image_point.y < cv_images[j]->height && image_point.x < cv_images[j]->width && image_point.y >= 0 && image_point.x >= 0))
        {
          continue; //point is not on the image, so skip to the next one
        }

        CvScalar color = cvGet2D(cv_images[j], image_point.y, image_point.x);
        pcl::PointXYZ color_point(color.val[0], color.val[1], color.val[2]);
        colors[j].push_back(color_point);
      }
    }

    //cluster color data to generate color regions
    std::vector<int> pointIdxNKNSearch(m_num_color_regions);
    std::vector<float> pointNKNSquaredDistance(m_num_color_regions);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    for(unsigned int i = 0; i < camera_clouds.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr color_cloud_ptr = colors[i].makeShared();
      kdtree.setInputCloud(color_cloud_ptr);
      pcl::PointXYZ searchPoint;
      if(!kdtree.nearestKSearch(searchPoint, m_num_color_regions, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        return false;
      }
//      for(size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
//        std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x << " " << cloud->points[pointIdxNKNSearch[i]].y << " " << cloud->points[pointIdxNKNSearch[i]].z << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    m_last_sample_time = ros::Time::now();
    return true;
  }

  void ColorVisionNode::processImages(std::vector<sensor_msgs::Image>& images)
  {
//    ROS_DEBUG("Processing image set");
//    std::vector<pcl::PointCloud<pcl::PointXYZ> > camera_clouds;
//    if(!transformGridToCameras(images, m_ground_grid, camera_clouds))
//    {
//      ROS_ERROR_THROTTLE(1.0, "Unable to transform cameras!");
//      return; //bad transform!
//    }
//
//    std::vector<cv_bridge::CvImagePtr> imgs;
//    std::vector<IplImage*> cv_images;
//    for(unsigned int i = 0; i < images.size(); i++)
//    {
//      imgs.push_back(cv_bridge::toCvCopy(images[i], images[i].encoding));
//      cv_images.push_back(new IplImage(imgs[i]->image));
//    }
//
//    if((ros::Time::now() - m_last_sample_time) > ros::Duration(m_sample_period))
//    {
//      if(!sampleKnownGround(images, cv_images))
//      {
//        ROS_ERROR("Ground sampling failed!");
//        return;
//      }
//    }
//
//    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;
//    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_no_color;
//    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
//    for(unsigned int i = 0; i < m_ground_grid.size(); i++)
//    {
//      int num_matches = 0;
//      int num_image_hits = 0;
//      std::vector<CvScalar> colors;
//      for(unsigned int j = 0; j < camera_clouds.size(); j++)
//      {
//        //todo: match 9-cell (or param) instead of one pixel
//        cv::Point3d cloud_point(camera_clouds[j].points[i].x, camera_clouds[j].points[i].y, camera_clouds[j].points[i].z);
//        cv::Point2d image_point = m_cam_models[j].project3dToPixel(cloud_point);
//
//        if(!(image_point.y < cv_images[j]->height && image_point.x < cv_images[j]->width && image_point.y >= 0 && image_point.x >= 0))
//        {
//          continue; //point is not on the image, so skip to the next one
//        }
//        else
//        {
//          num_image_hits++;
//        }
//
//        colors.push_back(cvGet2D(cv_images[j], image_point.y, image_point.x));
//
//        CvScalar matching_color;
//        if(colorsMatch(colors, images, matching_color))
//        {
//          pcl::PointXYZRGB point = m_ground_grid[i];
//          point.r = matching_color.val[0];
//          point.g = matching_color.val[1];
//          point.b = matching_color.val[2];
//          ground_cloud.points.push_back(point);
//          break;
//        }
//        else if(j == camera_clouds.size() - 1 && num_image_hits > 2) //the point was on more than one camera, but no match was found for any pair of cameras (the point must not be on the ground)
//        {
//          obstacle_cloud_no_color.points.push_back(m_ground_grid[i]);
//        }
//      }
//    }
//
//    ground_cloud.header = m_ground_grid.header;
//    pcl::toROSMsg(ground_cloud, m_ground_cloud_msg);
//
//    //find white points on the ground and add them to the obstacle_cloud
//    for(unsigned int i = 0; i < ground_cloud.size(); i++)
//    {
//      double lightness = ((ground_cloud.points[i].r + ground_cloud.points[i].g + ground_cloud.points[i].b) / 3);
//      bool light_enough = (lightness > m_lightness_threshold);
//      bool r_close = abs(ground_cloud.points[i].r - lightness) < m_white_threshold;
//      bool g_close = abs(ground_cloud.points[i].g - lightness) < m_white_threshold;
//      bool b_close = abs(ground_cloud.points[i].b - lightness) < m_white_threshold;
//      if(light_enough && r_close && g_close && b_close)
//      {
//        obstacle_cloud.points.push_back(ground_cloud.points[i]);
//        obstacle_cloud_no_color.points.push_back(pcl::PointXYZ(ground_cloud.points[i].x, ground_cloud.points[i].y, ground_cloud.points[i].z));
//      }
//    }
//
//    //remove outliers from obstacle cloud
//    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_filtered;
//    filterCloud(obstacle_cloud_no_color, obstacle_cloud_filtered);
//
//    obstacle_cloud_filtered.header = m_ground_grid.header;
//    pcl::toROSMsg(obstacle_cloud_filtered, m_obstacle_cloud_msg);
//
//    m_have_clouds = true;
  }

  bool ColorVisionNode::allImagesValid()
  {
    for(unsigned int i = 0; i < m_images_valid.size(); i++)
    {
      if(!m_images_valid[i])
      {
        return false;
      }
    }
    return true;
  }

  bool ColorVisionNode::imagesSynced()
  {
    for(unsigned int i = 0; i < m_images.size(); i++)
    {
      for(unsigned int j = i + 1; j < m_images.size(); j++)
      {
        ros::Duration time_diff = m_images[i].header.stamp - m_images[j].header.stamp;
        if(time_diff.toSec() > m_max_image_time_lag)
        {
          ROS_WARN_THROTTLE(1.0, "Images not synchronized!");
          return false;
        }
      }
    }
    return true;
  }

  void drawRegion(cv::Mat mat, cv::vector<cv::Point> contour, CvScalar line_color)
  {
    for(unsigned int i = 0; i < contour.size(); i++)
    {
      cv::line(mat, contour[i], contour[(i + 1) % contour.size()], line_color, 2);
    }
  }

  void findColorHeuristic(cv::Mat mat, cv::vector<cv::Point> contour, std::vector<ColorRegion>& regions)
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
    std::vector<cv::Vec3b> color_palette;
    for(unsigned int y = min_y; y <= max_y; y++)
    {
      for(unsigned int x = min_x; x <= max_x; x++)
      {
        unsigned char val = mask.at<unsigned char>(y, x); //
        if(val > 0)
        {
          cv::Vec3b color = mat.at<cv::Vec3b>(y, x);
          color_palette.push_back(color);
        }
      }
    }

    //cluster into many smaller ranges to increase identification fidelity
    cv::Mat p = cv::Mat::zeros(color_palette.size(), 3, CV_32F);
    for(int i = 0; i < color_palette.size(); i++)
    {
      p.at<float>(i, 0) = (float) color_palette[i].val[0] / 255.0f;
      p.at<float>(i, 1) = (float) color_palette[i].val[1] / 255.0f;
      p.at<float>(i, 2) = (float) color_palette[i].val[2] / 255.0f;
    }

    cv::Mat best_labels, centers, clustered;
    int K = 20; //m_num_color_regions;
    cv::kmeans(p, K, best_labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

    regions.clear();
    regions.resize(K);
    for(int i = 0; i < color_palette.size(); i++)
    {
      int cluster_index = best_labels.at<int>(0, i);
//      std::cerr << "Adding color (" << (int)color_palette[i].val[0] << ", " << (int)color_palette[i].val[1] << ", " << (int)color_palette[i].val[2] << ") to cluster " << cluster_index << std::endl;
      regions[cluster_index].addColor(color_palette[i]);
    }

//    regions[0].print();
//    regions[1].print();
  }

  void thresholdImage(cv::Mat mat, std::vector<ColorRegion> regions, cv::Mat& thresholded)
  {
    thresholded = cv::Mat(mat.rows, mat.cols, CV_8U, cv::Scalar(0));
    for(unsigned int i = 0; i < mat.rows * mat.cols; i++)
    {
      cv::Vec3b color = mat.at<cv::Vec3b>(i);
      for(unsigned int j = 0; j < regions.size(); j++)
      {
        if(regions[j].contains(color))
        {
          thresholded.at<unsigned char>(i) = 255;
          break;
        }
      }
    }
  }

  void ColorVisionNode::imageCallback(const sensor_msgs::ImageConstPtr& image, int idx)
  {
    if(idx > 0)
    {
      return;
    }
//    if(m_images.size() < m_image_subs.size())
//    {
//      m_images.resize(m_image_subs.size());
//      m_images_valid.resize(m_image_subs.size(), false);
//    }
//
//    m_images[idx] = *image;
//    m_images_valid[idx] = true;
//
//    ROS_DEBUG("Image %d get", idx);
//
//    if(allImagesValid() && imagesSynced())
//    {
//      processImages(m_images);
//      m_images_valid.clear();
//      m_images_valid.resize(m_image_subs.size(), false);
//    }

    sensor_msgs::Image ros_img = *image;
    cv_bridge::CvImagePtr bridge_img = cv_bridge::toCvCopy(ros_img, ros_img.encoding);
    IplImage* cv_image = new IplImage(bridge_img->image);
    cv::Mat rgb_mat = cv::cvarrToMat(cv_image);

    cv::Mat bgr_mat, hsv_mat, rgb_mat_blur;
    cv::GaussianBlur(rgb_mat, rgb_mat_blur, cv::Size(5, 5), 0, 0);
//    cv::pyrDown(rgb_mat, rgb_mat_blur);
//    cv::pyrDown(rgb_mat_blur, rgb_mat_blur);
    cvtColor(rgb_mat_blur, hsv_mat, CV_RGB2HSV);
    cvtColor(rgb_mat_blur, bgr_mat, CV_RGB2BGR);

    cv::vector<cv::Point> contour; //TODO: generate the contour from the parameters
    contour.push_back(cv::Point(0.375 * hsv_mat.cols, hsv_mat.rows));
    contour.push_back(cv::Point(0.4375 * hsv_mat.cols, 0.75 * hsv_mat.rows));
    contour.push_back(cv::Point((1 - 0.4375) * hsv_mat.cols, 0.75 * hsv_mat.rows));
    contour.push_back(cv::Point((1 - 0.375) * hsv_mat.cols, hsv_mat.rows));

    CvScalar line_color;
    line_color.val[0] = 0;
    line_color.val[1] = 255;
    line_color.val[2] = 255;
    drawRegion(bgr_mat, contour, line_color);

    std::vector<ColorRegion> regions;
    findColorHeuristic(hsv_mat, contour, regions);

    cv::Mat hsv_mat_down;
//    cv::pyrDown(hsv_mat, hsv_mat_down);

    cv::Mat thresholded;
    thresholdImage(hsv_mat, regions, thresholded);
//    cv::pyrUp(thresholded, thresholded);
//    cv::pyrUp(thresholded, thresholded);

    cv::namedWindow("Raw", 0);
    cv::imshow("Raw", bgr_mat);
    cv::namedWindow("Thresholded", 0);
    cv::imshow("Thresholded", thresholded);
    cv::waitKey(0);

//    findRegionsKMeans(cv_image);

//    findColors(cv_image, region);
  }

  void ColorVisionNode::spin()
  {
    ROS_INFO("ColorVisionNode started.");
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      if(m_have_clouds)
      {
        m_ground_cloud_msg.header.stamp = ros::Time::now();
        m_obstacle_cloud_msg.header.stamp = m_ground_cloud_msg.header.stamp;
        m_ground_cloud_pub.publish(m_ground_cloud_msg);
        m_obstacle_cloud_pub.publish(m_obstacle_cloud_msg);
      }
      else
      {
        ROS_WARN_THROTTLE(5.0, "No clouds!");
      }

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
