#include "vision/cloud_assimilator.h"

namespace vision
{
  CloudAssimilator::CloudAssimilator(const ros::NodeHandle& nh) :
      m_nh(nh), m_tf_listener(ros::Duration(30.0))
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("neighborhood_radius", m_neighborhood_radius, 0.2);
    m_nh.param("min_neighbors", m_min_neighbors, 3);
    m_nh.param("sensor_frame_id", m_sensor_frame_id, std::string("/base_link"));
    m_nh.param("sync_clouds", m_sync_clouds, true);
    m_nh.param("double_filter", m_double_filter, true);

    m_nh.param("output_laser_scan", m_output_laser_scan, true);
    m_nh.param("angle_min", m_angle_min, -1.75);
    m_nh.param("angle_max", m_angle_max, 1.75);
    m_nh.param("angle_increment", m_angle_increment, 0.005);
    m_nh.param("max_range", m_max_range, 10.0);

    m_have_new_left_cloud = false;
    m_have_new_right_cloud = false;

    m_left_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/left_camera/obstacles", 10, boost::bind(&CloudAssimilator::leftCallback, this, _1));
    m_right_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/right_camera/obstacles", 10, boost::bind(&CloudAssimilator::rightCallback, this, _1));

    m_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud>("/obstacles", 1);
    m_scan_pub = m_nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
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

  bool CloudAssimilator::timeSyncCloud(std_msgs::Header header, pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& synced_cloud)
  {
    if(!m_tf_listener.waitForTransform(header.frame_id, cloud.header.frame_id, header.stamp, ros::Duration(0.05), ros::Duration(0.001)))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Transform between " << header.frame_id << " and " << cloud.header.frame_id << " at time " << header.stamp << " failed!");
      return false;
    }
    pcl_ros::transformPointCloud(header.frame_id, header.stamp, cloud, cloud.header.frame_id, synced_cloud, m_tf_listener);

    return true;
  }

  struct sort_angle
  {
    inline bool operator()(const std::pair<double, double>& left, const std::pair<double, double>& right)
    {
      return (left.first < right.first);
    }
  };

  void CloudAssimilator::pointCloudToLaserScan(sensor_msgs::PointCloud cloud, sensor_msgs::LaserScan& scan)
  {
    std::vector<std::pair<double, double> > full_scan;
    for(unsigned int i = 0; i < cloud.points.size(); i++)
    {
      double angle = atan2(cloud.points[i].y, cloud.points[i].x);
      double distance = sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y);
      full_scan.push_back(std::pair<double, double>(angle, distance));
    }
    std::sort(full_scan.begin(), full_scan.end(), sort_angle());

    int num_bins = (m_angle_max - m_angle_min) / m_angle_increment;

    scan.header = cloud.header;
    scan.angle_min = m_angle_min;
    scan.angle_max = m_angle_max;
    scan.angle_increment = m_angle_increment;
    scan.ranges.resize(num_bins, m_max_range);
    scan.range_max = m_max_range;

    int current_bin = 0;
    for(unsigned int i = 0; i < full_scan.size(); i++)
    {
      double bin_change_threshold = m_angle_min + m_angle_increment * (current_bin + 0.5);
      while(full_scan[i].first > bin_change_threshold)
      {
        current_bin++;
        bin_change_threshold = m_angle_min + m_angle_increment * (current_bin + 1);
      }

      if(current_bin >= (int) scan.ranges.size())
      {
        break;
      }

      if(full_scan[i].second < scan.ranges[current_bin])
      {
        scan.ranges[current_bin] = full_scan[i].second;
      }
    }
  }

  void CloudAssimilator::filterCloud(pcl::PointCloud<pcl::PointXYZ>& combined_cloud, pcl::PointCloud<pcl::PointXYZ>& filtered_cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_ptr = combined_cloud.makeShared();

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(combined_cloud_ptr);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = m_neighborhood_radius;

//    ROS_INFO("Filtering based on %d neighbors in a %g radius", m_min_neighbors, m_neighborhood_radius);

    filtered_cloud.points.clear();
    for(unsigned int i = 0; i < combined_cloud.points.size(); i++)
    {
      int num_neighbors = kdtree.radiusSearch(combined_cloud.points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(num_neighbors > m_min_neighbors)
      {
        filtered_cloud.points.push_back(combined_cloud.points[i]);
      }
    }
    filtered_cloud.height = 1;
    filtered_cloud.width = filtered_cloud.points.size();

//    double dx = combined_cloud.points[0].x - combined_cloud.points[1].x;
//    double dy = combined_cloud.points[0].y - combined_cloud.points[1].y;
//    double dz = combined_cloud.points[0].z - combined_cloud.points[1].z;
//    double sanity_distance = sqrt(dx*dx+dy*dy+dz*dz);
//    std::cerr << sanity_distance << std::endl;

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
  }

  void CloudAssimilator::processClouds()
  {
    pcl::PointCloud<pcl::PointXYZ> combined_cloud = m_left_cloud;

    if(m_left_cloud.points.size() == 0 || m_right_cloud.points.size() == 0)
    {
      ROS_WARN("Clouds empty!");
      m_have_new_left_cloud = false;
      m_have_new_right_cloud = false;
      return;
    }

    if(m_sync_clouds)
    {
      pcl::PointCloud<pcl::PointXYZ> synced_right_cloud;
      if(!timeSyncCloud(pcl_conversions::fromPCL(m_left_cloud.header), m_right_cloud, synced_right_cloud))
      {
        m_have_new_left_cloud = false;
        m_have_new_right_cloud = false;
        return;
      }
      combined_cloud += synced_right_cloud;
    }
    else
    {
      combined_cloud += m_right_cloud;
    }

    if(combined_cloud.points.size() == 0)
    {
      ROS_WARN("Input clouds had no points!");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    filterCloud(combined_cloud, filtered_cloud);
//    if(m_double_filter && filtered_cloud.points.size() != 0)
//    {
//      pcl::PointCloud<pcl::PointXYZ> double_filtered_cloud;
//      filterCloud(filtered_cloud, double_filtered_cloud);
//      filtered_cloud = double_filtered_cloud;
//    }

    if((((double) filtered_cloud.size()) / ((double) combined_cloud.size())) < 0.5)
    {
      ROS_WARN_THROTTLE(1.0, "%d/%d (%g%%) cloud points remaining after filtering", (int) filtered_cloud.size(), (int) combined_cloud.size(), 100.0 * filtered_cloud.size() / combined_cloud.size());
    }

    sensor_msgs::PointCloud2 filtered_cloud_msg2;
    pcl::toROSMsg(filtered_cloud, filtered_cloud_msg2);
    filtered_cloud_msg2.header = pcl_conversions::fromPCL(m_left_cloud.header);
    sensor_msgs::PointCloud filtered_cloud_msg;
    convertPointCloud2ToPointCloud(filtered_cloud_msg2, filtered_cloud_msg);
    filtered_cloud_msg.header = filtered_cloud_msg2.header;
    m_cloud_pub.publish(filtered_cloud_msg);

    if(m_output_laser_scan)
    {
      sensor_msgs::LaserScan scan;
      pointCloudToLaserScan(filtered_cloud_msg, scan);
      m_scan_pub.publish(scan);
    }

    m_have_new_left_cloud = false;
    m_have_new_right_cloud = false;
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
