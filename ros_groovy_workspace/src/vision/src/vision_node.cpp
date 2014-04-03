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
    m_nh.param("max_image_time_lag", m_max_image_time_lag, 0.1);

    if(m_num_cameras < 2 || m_num_cameras > 8)
    {
      ROS_FATAL("The number of cameras must be between 2 and 8.");
      ros::shutdown();
    }

    m_ground_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ground", 1, true);
    m_obstacle_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obstacles", 1, true);
    m_sensed_area_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/sensed_area", 1, true);

    double resolution, x_min, x_max, y_min, y_max;
    m_nh.param("resolution", resolution, 0.05);
    m_nh.param("x_min", x_min, 0.0);
    m_nh.param("x_max", x_max, 2.0);
    m_nh.param("y_min", y_min, -0.75);
    m_nh.param("y_max", y_max, 0.75);

    generateGroundGrid(resolution, x_min, x_max, y_min, y_max);

    m_have_clouds = false;

    m_image_subs.resize(m_num_cameras);
    for(unsigned int i = 0; i < m_num_cameras; i++)
    {
      std::stringstream image_topic, info_topic, frame;
      image_topic << "/image_" << i;
      info_topic << "/camera_info_" << i;
//      frame << "/camera_" << i << "_frame_id";
//
//      std::string camera_frame_id;
//      m_nh.param(frame.str(), camera_frame_id, frame.str());
//      m_image_frame_ids.push_back(camera_frame_id);

      sensor_msgs::CameraInfoConstPtr camera_info_ptr;
      while(!camera_info_ptr && ros::ok())
      {
        camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic.str(), m_nh, ros::Duration(0.1));
        ROS_WARN_THROTTLE(1.0, "Waiting for camera #%d's info", i);
      }
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_ptr);
      m_cam_models.push_back(model);

      ros::SubscribeOptions input_options = ros::SubscribeOptions::create<sensor_msgs::Image>(image_topic.str(), 1, boost::bind(&VisionNode::imageCallback, this, _1, i), ros::VoidPtr(), m_nh.getCallbackQueue());
      m_image_subs.at(i) = m_nh.subscribe(input_options);
      ROS_INFO_STREAM("Subscribed to " << m_image_subs.at(i).getTopic());
    }
  }

//  ros::SubscribeOptions::create(std::stringstream&, boost::_bi::bind_t<void, boost::_mfi::mf2<void, vision_node::VisionNode, const boost::shared_ptr<const sensor_msgs::Image_<std::allocator<void> > >&, int>, boost::_bi::list3<boost::_bi::value<vision_node::VisionNode*>, boost::arg<1>, boost::_bi::value<unsigned int> > >, ros::VoidPtr, ros::CallbackQueueInterface*)’
//  ros::SubscribeOptions::create(const string&, uint32_t, const boost::function<void(const boost::shared_ptr<const M>&)>&, const VoidConstPtr&, ros::CallbackQueueInterface*)

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
    pcl::toROSMsg(m_ground_grid, m_sensed_area_cloud_msg);
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

  bool VisionNode::transformGridToCameras(std::vector<sensor_msgs::Image>& images, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds)
  {
    for(unsigned int i = 0; i < images.size(); i++)
    {
      m_ground_grid.header.stamp = images[i].header.stamp;
      if(!m_tf_listener.waitForTransform(images[i].header.frame_id, m_ground_grid.header.frame_id, m_ground_grid.header.stamp, ros::Duration(0.05), ros::Duration(0.001)))
      {
        ROS_ERROR_THROTTLE(1.0, "Transform between %s and %s failed!", images[i].header.frame_id.c_str(), m_ground_grid.header.frame_id.c_str());
        return false;
      }
      pcl::PointCloud<pcl::PointXYZRGB> camera_cloud;
      pcl_ros::transformPointCloud(images[i].header.frame_id, m_ground_grid, camera_cloud, m_tf_listener);
      clouds.push_back(camera_cloud);
    }

    return true;
  }

  bool VisionNode::colorsMatch(std::vector<CvScalar> colors, std::vector<sensor_msgs::Image>& images, CvScalar matched_color)
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

  void VisionNode::processImages(std::vector<sensor_msgs::Image>& images)
  {
    ROS_DEBUG("Processing image set");
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > camera_clouds;
    if(!transformGridToCameras(images, camera_clouds))
    {
      ROS_ERROR_THROTTLE(1.0, "Unable to transform cameras!");
      return; //bad transform!
    }

    std::vector<cv_bridge::CvImagePtr> imgs;
    std::vector<IplImage*> cv_images;
    for(unsigned int i = 0; i < images.size(); i++)
    {
      imgs.push_back(cv_bridge::toCvCopy(images[i], images[i].encoding));
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

        if(!(image_point.y < cv_images[j]->height && image_point.x < cv_images[j]->width && image_point.y >= 0 && image_point.x >= 0))
        {
          obstacle_cloud.points.push_back(m_ground_grid[i]);//TODO: REMOVE!
          //ROS_INFO("Grid cell (%g,%g,%g) was not on the image!", cloud_point.x, cloud_point.y, cloud_point.z);
          continue; //point is not on the image, so skip to the next one
        }

//        colors.push_back(cvGet2D(cv_images[j], image_point.y, image_point.x));
        CvScalar matching_color;
//        if(colorsMatch(colors, images, matching_color))
//        {
          pcl::PointXYZRGB point = m_ground_grid[i];
          point.r = matching_color.val[0];
          point.g = matching_color.val[1];
          point.b = matching_color.val[2];
          ground_cloud.points.push_back(point);
//          break;
//        }
//        else if(j == camera_clouds.size() - 1) //no match was found for any pair of cameras (the point must not be on the ground)
//        {
//          obstacle_cloud.points.push_back(m_ground_grid[i]);
//        }
      }
    }

    ground_cloud.header = m_ground_grid.header;
    pcl::toROSMsg(ground_cloud, m_ground_cloud_msg);

    //find white points on the ground and add them to the obstacle_cloud
    for(unsigned int i = 0; i < ground_cloud.size(); i++)
    {
      if(((ground_cloud.points[i].r + ground_cloud.points[i].g + ground_cloud.points[i].b) / 3) > m_white_threshold)
      {
        obstacle_cloud.points.push_back(ground_cloud.points[i]);
      }
    }

    obstacle_cloud.header = m_ground_grid.header;
    pcl::toROSMsg(obstacle_cloud, m_obstacle_cloud_msg);

    m_have_clouds = true;
  }

  bool VisionNode::allImagesValid()
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

  bool VisionNode::imagesSynced()
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

  void VisionNode::imageCallback(const sensor_msgs::ImageConstPtr& image, int idx)
  {
    if(m_images.size() < m_image_subs.size())
    {
      m_images.resize(m_image_subs.size());
      m_images_valid.resize(m_image_subs.size(), false);
    }

    m_images[idx] = *image;
    m_images_valid[idx] = true;

    ROS_DEBUG("Image %d get", idx);

    if(allImagesValid() && imagesSynced())
    {
      processImages(m_images);
      m_images_valid.clear();
      m_images_valid.resize(m_image_subs.size(), false);
    }
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
        m_sensed_area_cloud_pub.publish(m_sensed_area_cloud_msg);
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
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh("~");

  vision_node::VisionNode node(nh);
  node.spin();

  return 0;
}
