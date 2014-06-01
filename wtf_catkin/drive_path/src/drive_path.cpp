#include "drive_path/drive_path.h"

namespace drive_path
{
  DrivePath::DrivePath(const ros::NodeHandle &nh) :
    m_nh(nh), m_client(m_nh, "/move_base", true)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);

    std::string waypoints;
    m_nh.param("waypoints", waypoints, std::string(""));

    assert(waypoints.length() > 0);

    parseWaypoints(waypoints);

    while (!m_client.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    m_execute_path = false;
    m_go_service = m_nh.advertiseService("/go", &DrivePath::goCallback, this);
  }

  DrivePath::~DrivePath()
  {
  }

  bool DrivePath::goCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    m_execute_path = true;
    return true;
  }

  std::vector<double> stringVectorToDoubleVector(std::vector<std::string> input)
  {
    std::vector<double> output;

    for (unsigned int i = 0; i < input.size(); i++)
    {
      output.push_back(std::atof(input[i].c_str()));
    }

    return output;
  }

  void DrivePath::parseWaypoints(std::string waypoints)
  {
    boost::char_separator<char> sep(" ,()[]");
    boost::tokenizer<boost::char_separator<char> > tokens(waypoints, sep);
    std::vector<double> values = stringVectorToDoubleVector(std::vector<std::string>(tokens.begin(), tokens.end()));

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.z = 0;

    assert((values.size() % 3) == 0);
    std::cerr << "Parsed path: ";
    for (unsigned int i = 0; i < values.size(); i += 3)
    {
      pose.pose.position.x = values.at(i + 0);
      pose.pose.position.y = values.at(i + 1);
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(values.at(i + 2));

      std::cerr << "(" << pose.pose.position.x << "," << pose.pose.position.y << "," << values.at(i + 2) << ")";

      m_path.push_back(pose);
    }
    std::cerr << std::endl;
  }

  void DrivePath::execute()
  {
    for (unsigned int i = 0; i < m_path.size(); i++)
    {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = m_path[i];
      ROS_INFO("Sending goal #%d", i);
      m_client.sendGoal(goal);
      m_client.waitForResult();

      if (m_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Success!");
      }
    }
    m_execute_path = false;
  }

  void DrivePath::spin()
  {
    ROS_INFO("DrivePath started.");
    ros::Rate loop_rate(m_loop_rate);
    while (ros::ok())
    {
      if(m_execute_path)
      {
        execute();
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_path");
  ros::NodeHandle nh("~");

  drive_path::DrivePath node(nh);
  node.spin();

  return 0;
}
