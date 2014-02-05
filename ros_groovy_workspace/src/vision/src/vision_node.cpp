#include "vision/vision_node.h"

namespace vision_node
{
  VisionNode::VisionNode(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("white_threshold", m_white_threshold, 200);
    m_nh.param("base_frame_id", m_base_frame_id, std::string("/base_link"));
    m_nh.param("num_cameras", m_num_cameras, 2);

    if(m_num_cameras < 2 || m_num_cameras > 8)
    {
      ROS_FATAL("The number of cameras must be between 2 and 8.");
      ros::shutdown();
    }

    m_ground_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ground", 1, true);
    m_obstacle_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obstacles", 1, true);

    double resolution, x_min, x_max, y_min, y_max;
    m_nh.param("resolution", resolution, 0.05);
    m_nh.param("x_min", x_min, 0.0);
    m_nh.param("x_max", x_max, 2.0);
    m_nh.param("y_min", y_min, -0.75);
    m_nh.param("y_max", y_max, 0.75);

    generateGroundGrid(resolution, x_min, x_max, y_min, y_max);

    m_have_clouds = false;

    for(unsigned int i = 0; i < m_num_cameras; i++)
    {
      std::stringstream image_topic, info_topic, frame;
      image_topic << "/image_" << i;
      info_topic << "/camera_info_" << i;
      frame << "/camera_" << i << "_frame_id";

      std::string camera_frame_id;
      m_nh.param(frame.str(), camera_frame_id, frame.str());
      m_image_frame_ids.push_back(camera_frame_id);

      sensor_msgs::CameraInfoConstPtr camera_info_ptr;
      while(!camera_info_ptr)
      {
        camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic.str(), m_nh);
        ROS_WARN_THROTTLE(1.0, "Waiting for camera #%d info", i);
      }
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_ptr);
      m_cam_models.push_back(model);
    }

//todo: find a less terrible way to do this
    switch(m_num_cameras)
    {
    case 2:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      m_sync2 = new message_filters::Synchronizer<ImagePolicy2>(ImagePolicy2(10), image0_sub, image1_sub);
      m_sync2->registerCallback(boost::bind(&VisionNode::imageCallback2, this, _1, _2));
      break;
    }
    case 3:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      m_sync3 = new message_filters::Synchronizer<ImagePolicy3>(ImagePolicy3(10), image0_sub, image1_sub, image2_sub);
      m_sync3->registerCallback(boost::bind(&VisionNode::imageCallback3, this, _1, _2, _3));
      break;
    }
    case 4:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "/image3", 1);
      m_sync4 = new message_filters::Synchronizer<ImagePolicy4>(ImagePolicy4(10), image0_sub, image1_sub, image2_sub, image3_sub);
      m_sync4->registerCallback(boost::bind(&VisionNode::imageCallback4, this, _1, _2, _3, _4));
      break;
    }
    case 5:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "/image3", 1);
      message_filters::Subscriber<sensor_msgs::Image> image4_sub(m_nh, "/image4", 1);
      m_sync5 = new message_filters::Synchronizer<ImagePolicy5>(ImagePolicy5(10), image0_sub, image1_sub, image2_sub, image3_sub, image4_sub);
      m_sync5->registerCallback(boost::bind(&VisionNode::imageCallback5, this, _1, _2, _3, _4, _5));
      break;
    }
    case 6:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "/image3", 1);
      message_filters::Subscriber<sensor_msgs::Image> image4_sub(m_nh, "/image4", 1);
      message_filters::Subscriber<sensor_msgs::Image> image5_sub(m_nh, "/image5", 1);
      m_sync6 = new message_filters::Synchronizer<ImagePolicy6>(ImagePolicy6(10), image0_sub, image1_sub, image2_sub, image3_sub, image4_sub, image5_sub);
      m_sync6->registerCallback(boost::bind(&VisionNode::imageCallback6, this, _1, _2, _3, _4, _5, _6));
      break;
    }
    case 7:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "/image3", 1);
      message_filters::Subscriber<sensor_msgs::Image> image4_sub(m_nh, "/image4", 1);
      message_filters::Subscriber<sensor_msgs::Image> image5_sub(m_nh, "/image5", 1);
      message_filters::Subscriber<sensor_msgs::Image> image6_sub(m_nh, "/image6", 1);
      m_sync7 = new message_filters::Synchronizer<ImagePolicy7>(ImagePolicy7(10), image0_sub, image1_sub, image2_sub, image3_sub, image4_sub, image5_sub, image6_sub);
      m_sync7->registerCallback(boost::bind(&VisionNode::imageCallback7, this, _1, _2, _3, _4, _5, _6, _7));
      break;
    }
    case 8:
    {
      message_filters::Subscriber<sensor_msgs::Image> image0_sub(m_nh, "/image0", 1);
      message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "/image1", 1);
      message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "/image2", 1);
      message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "/image3", 1);
      message_filters::Subscriber<sensor_msgs::Image> image4_sub(m_nh, "/image4", 1);
      message_filters::Subscriber<sensor_msgs::Image> image5_sub(m_nh, "/image5", 1);
      message_filters::Subscriber<sensor_msgs::Image> image6_sub(m_nh, "/image6", 1);
      message_filters::Subscriber<sensor_msgs::Image> image7_sub(m_nh, "/image7", 1);
      m_sync8 = new message_filters::Synchronizer<ImagePolicy8>(ImagePolicy8(10), image0_sub, image1_sub, image2_sub, image3_sub, image4_sub, image5_sub, image6_sub, image7_sub);
      m_sync8->registerCallback(boost::bind(&VisionNode::imageCallback8, this, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    }
    default:
      break;
    }
  }

  VisionNode::~VisionNode()
  {
  }

  void VisionNode::generateGroundGrid(double resolution, double x_min, double x_max, double y_min, double y_max)
  {
    for(double x = x_min; x <= x_max; x += resolution)
    {
      for(double y = y_min; y <= y_max; y += resolution)
      {
        pcl::PointXYZRGB point;
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

  double VisionNode::colorDistance(CvScalar a, CvScalar b)
  {
    double dist = 0;
    for(int i = 0; i < 4; i++)
    {
      double diff = a.val[i] - b.val[i];
      dist += diff * diff;
    }
    return dist;
  }

  void VisionNode::parseEncoding(CvScalar s, std::string encoding, unsigned char& red, unsigned char& green, unsigned char& blue)
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

  bool VisionNode::transformGridToCameras(std::vector<sensor_msgs::ImageConstPtr>& images, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds)
  {
    for(unsigned int i = 0; i < images.size(); i++)
    {
      m_ground_grid.header.stamp = images[i]->header.stamp;
      if(!m_tf_listener.waitForTransform(m_image_frame_ids[i], m_ground_grid.header.frame_id, m_ground_grid.header.stamp, ros::Duration(0.05), ros::Duration(0.001)))
      {
        return false;
      }
      pcl::PointCloud<pcl::PointXYZRGB> camera_cloud;
      pcl_ros::transformPointCloud(m_image_frame_ids[i], m_ground_grid, camera_cloud, m_tf_listener);
      clouds.push_back(camera_cloud);
    }

    return true;
  }

  bool VisionNode::colorsMatch(std::vector<CvScalar> colors, std::vector<sensor_msgs::ImageConstPtr>& images, CvScalar matched_color)
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
        parseEncoding(colors[i], images[i]->encoding, r1, g1, b1);
        parseEncoding(colors[colors.size() - 1], images[colors.size() - 1]->encoding, r2, g2, b2);
        matched_color.val[0] = (r1 + r2) / 2;
        matched_color.val[1] = (g1 + g2) / 2;
        matched_color.val[2] = (b1 + b2) / 2;
        return true;
      }
    }

    return false;
  }

  void VisionNode::processImages(std::vector<sensor_msgs::ImageConstPtr>& images)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > camera_clouds;
    if(!transformGridToCameras(images, camera_clouds))
    {
      return; //bad transform!
    }

    std::vector<cv_bridge::CvImagePtr> imgs;
    std::vector<IplImage*> cv_images;
    for(unsigned int i = 0; i < images.size(); i++)
    {
      imgs.push_back(cv_bridge::toCvCopy(*images[i], images[i]->encoding));
      cv_images.push_back(new IplImage(imgs[i]->image));
    }

    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
    for(unsigned int i = 0; i < m_ground_grid.size(); i++)
    {
      int num_matches = 0;
      std::vector<CvScalar> colors;
      for(unsigned int j = 0; j < camera_clouds.size(); j++)
      {
        //todo: match 9-cell (or param) instead of one pixel
        cv::Point3d cloud_point(camera_clouds[j].points[i].x, camera_clouds[j].points[i].y, camera_clouds[j].points[i].z);
        cv::Point2d image_point = m_cam_models[j].project3dToPixel(cloud_point);

        if(!(image_point.y < cv_images[i]->height && image_point.x < cv_images[i]->width && image_point.y >= 0 && image_point.x >= 0))
        {
          continue; //point is not on the image, so skip to the next one
        }

        colors.push_back(cvGet2D(cv_images[i], image_point.y, image_point.x));
        CvScalar matching_color;
        if(colorsMatch(colors, images, matching_color))
        {
          pcl::PointXYZRGB point = m_ground_grid[i];
          point.r = matching_color.val[0];
          point.g = matching_color.val[1];
          point.b = matching_color.val[2];
          ground_cloud.points.push_back(point);
          break;
        }
        else if(j == camera_clouds.size() - 1) //no match was found for any pair of cameras (the point must not be on the ground)
        {
          obstacle_cloud.points.push_back(m_ground_grid[i]);
        }
      }
    }

    pcl::toROSMsg(ground_cloud, m_ground_cloud_msg);

    //find white points on the ground and add them to the obstacle_cloud
    for(unsigned int i = 0; i < ground_cloud.size(); i++)
    {
      if(((ground_cloud.points[i].r + ground_cloud.points[i].g + ground_cloud.points[i].b) / 3) > m_white_threshold)
      {
        obstacle_cloud.points.push_back(ground_cloud.points[i]);
      }
    }

    pcl::toROSMsg(obstacle_cloud, m_obstacle_cloud_msg);

    m_have_clouds = true;
  }

  //todo: find a way to do this better
  void VisionNode::imageCallback2(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    processImages(images);
  }

  void VisionNode::imageCallback3(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    processImages(images);
  }

  void VisionNode::imageCallback4(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    processImages(images);
  }

  void VisionNode::imageCallback5(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    images.push_back(image4);
    processImages(images);
  }

  void VisionNode::imageCallback6(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    images.push_back(image4);
    images.push_back(image5);
    processImages(images);
  }

  void VisionNode::imageCallback7(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    images.push_back(image4);
    images.push_back(image5);
    images.push_back(image6);
    processImages(images);
  }

  void VisionNode::imageCallback8(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6, const sensor_msgs::ImageConstPtr& image7)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    images.push_back(image4);
    images.push_back(image5);
    images.push_back(image6);
    images.push_back(image7);
    processImages(images);
  }

  void VisionNode::imageCallback9(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3, const sensor_msgs::ImageConstPtr& image4, const sensor_msgs::ImageConstPtr& image5, const sensor_msgs::ImageConstPtr& image6, const sensor_msgs::ImageConstPtr& image7, const sensor_msgs::ImageConstPtr& image8)
  {
    std::vector<sensor_msgs::ImageConstPtr> images;
    images.push_back(image0);
    images.push_back(image1);
    images.push_back(image2);
    images.push_back(image3);
    images.push_back(image4);
    images.push_back(image5);
    images.push_back(image6);
    images.push_back(image7);
    images.push_back(image8);
    processImages(images);
  }

  void VisionNode::spin()
  {
    ROS_INFO("VisionNode started.");
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

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh("~");

  vision_node::VisionNode node(nh);
  node.spin();

  return 0;
}
