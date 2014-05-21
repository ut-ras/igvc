#include "vision/cloud_assimilator.h"

namespace vision
{
  CloudAssimilator::CloudAssimilator(const ros::NodeHandle& nh) :
      m_nh(nh), m_tf_listener(ros::Duration(30.0))
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("neighborhood_radius", m_neighborhood_radius, 0.25);
    m_nh.param("min_neighbors", m_min_neighbors, 5);
    m_nh.param("sensor_frame_id", m_sensor_frame_id, std::string("/base_link"));

    m_have_new_left_cloud = false;
    m_have_new_right_cloud = false;

    m_left_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/left_camera/obstacles", 10, boost::bind(&CloudAssimilator::leftCallback, this, _1));
    m_right_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/right_camera/obstacles", 10, boost::bind(&CloudAssimilator::rightCallback, this, _1));

    m_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/obstacles", 1);
  }

  CloudAssimilator::~CloudAssimilator()
  {
  }

  void CloudAssimilator::leftCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::fromROSMsg(*cloud, m_left_cloud);
    m_have_new_left_cloud = true;
  }

  void CloudAssimilator::rightCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::fromROSMsg(*cloud, m_right_cloud);
    m_have_new_right_cloud = true;
  }

  void CloudAssimilator::processClouds()
  {
    pcl::PointCloud<pcl::PointXYZ> combined_cloud = m_left_cloud;
    combined_cloud += m_right_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_ptr = combined_cloud.makeShared();

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(combined_cloud_ptr);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = m_neighborhood_radius;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    for(unsigned int i = 0; i < combined_cloud.size(); i++)
    {
      int num_neighbors = kdtree.radiusSearch(combined_cloud.points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(num_neighbors > m_min_neighbors)
      {
        filtered_cloud.points.push_back(combined_cloud.points[i]);
      }
    }
    filtered_cloud.height = 1;
    filtered_cloud.width = filtered_cloud.points.size();

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

    if(filtered_cloud.size() / combined_cloud_ptr->size() < 0.5)
    {
      ROS_WARN_THROTTLE(1.0, "%d/%d (%g%%) cloud points remaining after filtering", (int) filtered_cloud.size(), (int) combined_cloud_ptr->size(), 100.0 * filtered_cloud.size() / combined_cloud_ptr->size());
    }

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = m_left_cloud.header;
    m_cloud_pub.publish(filtered_cloud_msg);

    m_have_new_left_cloud = false;
    m_have_new_right_cloud = false; //TODO: actually look at timestamps for approximate sync
  }

  void CloudAssimilator::spin()
  {
    ROS_INFO("CloudAssimilator started.");
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      if(m_have_new_left_cloud && m_have_new_right_cloud)
      {
        processClouds();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_assimilator");
  ros::NodeHandle nh("~");

  vision::CloudAssimilator node(nh);
  node.spin();

  return 0;
}
