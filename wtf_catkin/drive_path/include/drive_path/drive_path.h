#ifndef DRIVE_PATH_H
#define DRIVE_PATH_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

namespace drive_path
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  class DrivePath
  {
  public:
    DrivePath(const ros::NodeHandle& nh);
    ~DrivePath();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;

    MoveBaseClient m_client;
    ros::ServiceServer m_go_service;

    std::vector<geometry_msgs::PoseStamped> m_path;

    bool m_execute_path;

    void parseWaypoints(std::string waypoints);
    void execute();

    bool goCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  };
}
#endif //DRIVE_PATH_H
