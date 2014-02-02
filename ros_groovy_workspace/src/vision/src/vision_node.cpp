#include "vision/vision_node.h"

namespace vision_node
{
  VisionNode::VisionNode(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("white_threshold", m_white_threshold, 200);
    m_nh.param("base_frame_id", m_base_frame_id, std::string("/base_link"));

    m_ground_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ground", 1, true);
    m_obstacle_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obstacles", 1, true);

    sensor_msgs::CameraInfoConstPtr camera_1_info_ptr;
    while(!camera_1_info_ptr)
    {
      camera_1_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/image_1_info", m_nh);
      ROS_WARN_THROTTLE(1.0, "Waiting for camera 1 info");
    }

    sensor_msgs::CameraInfoConstPtr camera_2_info_ptr;
    while(!camera_2_info_ptr)
    {
      camera_2_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/image_2_info", m_nh);
      ROS_WARN_THROTTLE(1.0, "Waiting for camera 2 info");
    }

    sensor_msgs::CameraInfoConstPtr camera_3_info_ptr;
    while(!camera_3_info_ptr)
    {
      camera_3_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/image_3_info", m_nh);
      ROS_WARN_THROTTLE(1.0, "Waiting for camera 3 info");
    }

    double resolution, x_min, x_max, y_min, y_max;
    m_nh.param("resolution", resolution, 0.05);
    m_nh.param("x_min", x_min, 0.0);
    m_nh.param("x_max", x_max, 2.0);
    m_nh.param("y_min", y_min, -0.75);
    m_nh.param("y_max", y_max, 0.75);

    generateGroundGrid(resolution, x_min, x_max, y_min, y_max);

    m_have_clouds = false;

    message_filters::Subscriber<sensor_msgs::Image> image1_sub(m_nh, "image1", 1);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(m_nh, "image2", 1);
    message_filters::Subscriber<sensor_msgs::Image> image3_sub(m_nh, "image3", 1);

    m_sync = new message_filters::Synchronizer<TripleImagePolicy>(TripleImagePolicy(10), image1_sub, image2_sub, image3_sub);
    m_sync->registerCallback(boost::bind(&VisionNode::imageCallback, this, _1, _2, _3));
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

  void VisionNode::imageCallback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2, const sensor_msgs::ImageConstPtr& image3)
  {
    cv_bridge::CvImagePtr img1 = cv_bridge::toCvCopy(*image1, image1->encoding);
    cv_bridge::CvImagePtr img2 = cv_bridge::toCvCopy(*image2, image2->encoding);
    cv_bridge::CvImagePtr img3 = cv_bridge::toCvCopy(*image3, image3->encoding);
    IplImage *cv_image1 = new IplImage(img1->image);
    IplImage *cv_image2 = new IplImage(img2->image);
    IplImage *cv_image3 = new IplImage(img3->image);

    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
    for(unsigned int i = 0; i < m_ground_grid1.size(); i++)
    {
      cv::Point3d cloud_point1(m_ground_grid1.points[i].x, m_ground_grid1.points[i].y, m_ground_grid1.points[i].z);
      cv::Point2d image_point1 = m_cam_model_1.project3dToPixel(cloud_point1);
      CvScalar color1 = cvGet2D(cv_image1, image_point1.y, image_point1.x);

      cv::Point3d cloud_point2(m_ground_grid2.points[i].x, m_ground_grid2.points[i].y, m_ground_grid2.points[i].z);
      cv::Point2d image_point2 = m_cam_model_2.project3dToPixel(cloud_point2);
      CvScalar color2 = cvGet2D(cv_image2, image_point2.y, image_point2.x);

      cv::Point3d cloud_point3(m_ground_grid3.points[i].x, m_ground_grid3.points[i].y, m_ground_grid3.points[i].z);
      cv::Point2d image_point3 = m_cam_model_3.project3dToPixel(cloud_point3);
      CvScalar color3 = cvGet2D(cv_image3, image_point3.y, image_point3.x);

      double d12 = colorDistance(color1, color2);
      double d23 = colorDistance(color2, color3);
      double d31 = colorDistance(color3, color1);

      if(d12 < m_match_threshold || d23 < m_match_threshold || d31 < m_match_threshold)
      {
        //matched by at least one pair of cameras, so fill the color and add it to the ground
        pcl::PointXYZRGB point = m_ground_grid[i];
        if(d12)
        {
          unsigned char r1, b1, g1, r2, b2, g2;
          parseEncoding(color1, image1->encoding, r1, b1, g1);
          parseEncoding(color2, image2->encoding, r2, b2, g2);
          point.r = (r1 + r2)/2;
          point.g = (g1 + g2)/2;
          point.b = (b1 + b2)/2;
        }
        else if(d23)
        {
          unsigned char r2, b2, g2, r3, b3, g3;
          parseEncoding(color2, image2->encoding, r2, b2, g2);
          parseEncoding(color3, image3->encoding, r3, b3, g3);
          point.r = (r2 + r3)/2;
          point.g = (g2 + g3)/2;
          point.b = (b2 + b3)/2;
        }
        else
        {
          unsigned char r1, b1, g1, r2, b2, g2;
          parseEncoding(color1, image1->encoding, r1, b1, g1);
          parseEncoding(color2, image2->encoding, r2, b2, g2);
          point.r = (r1 + r2)/2;
          point.g = (g1 + g2)/2;
          point.b = (b1 + b2)/2;
        }

        ground_cloud.points.push_back(m_ground_grid[i]);
      }
      else
      {
        obstacle_cloud.points.push_back(m_ground_grid[i]);
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
