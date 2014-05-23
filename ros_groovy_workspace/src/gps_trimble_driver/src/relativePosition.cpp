#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <gps_trimble_driver/Point.h>
#include <gps_trimble_driver/Waypoints.h>
#include <gps_trimble_driver/GetZero.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <gps_common/conversions.h>

#include <vector>
#include <iostream>
#include <fstream>

gps_trimble_driver::Point::Request firstFix;
bool TrustFix = false;
ros::Publisher correctedOdometry;
std::vector< geometry_msgs::Point > Waypoints;

bool zero_at_point ( gps_trimble_driver::Point::Request  &req,
		     gps_trimble_driver::Point::Response &res ) {
  firstFix.point.x = req.point.x;
  firstFix.point.y = req.point.y;
  firstFix.point.z = req.point.z;
  TrustFix = true;
  return true; }

bool zero ( std_srvs::Empty::Request  &req,
	    std_srvs::Empty::Response &res ) {
  TrustFix = false;
  return true; }

bool get_zero_point ( gps_trimble_driver::GetZero::Request &req,
		      gps_trimble_driver::GetZero::Response &res ) {
  res.point.x=firstFix.point.x ;
  res.point.y=firstFix.point.y ;
  res.point.z=firstFix.point.z ;
  return TrustFix; }

void relativeizeOdometry ( const nav_msgs::OdometryConstPtr &msg ) {
  if ( TrustFix ) {
    geometry_msgs::PoseWithCovariance newMsg = msg->pose;
    newMsg.pose.position.x -= firstFix.point.x;
    newMsg.pose.position.z -= firstFix.point.z;
    newMsg.pose.position.y -= firstFix.point.y;
    correctedOdometry.publish(newMsg); }
  else {
    firstFix.point.x = msg->pose.pose.position.x;
    firstFix.point.z = msg->pose.pose.position.z;
    firstFix.point.y = msg->pose.pose.position.y;
    TrustFix = true; } }

bool get_relative_waypoints ( gps_trimble_driver::Waypoints::Request  &req,
                              gps_trimble_driver::Waypoints::Response &res ) {
  if ( TrustFix ) {
    for (std::vector< geometry_msgs::Point >::iterator it = Waypoints.begin(); it != Waypoints.end(); ++it) {
      res.waypoints.push_back( *it );
      res.waypoints[res.waypoints.size() - 1].x -= firstFix.point.x;
      res.waypoints[res.waypoints.size() - 1].y -= firstFix.point.y;
      res.waypoints[res.waypoints.size() - 1].z -= firstFix.point.z; }
    return true; }
  else
    return false; }

double deg_min_sec_to_decimal( double deg, double mins, double sec ) {
  return ( deg / abs( deg ) ) * ( abs( deg ) + ( ( mins + ( sec / 60. ) ) / 60. ) ); }

int main(int argc, char**argv, char**envp) {
  
  ros::init( argc, argv, "odometry_relativeizer" );
  ros::NodeHandle node;
  ros::NodeHandle priv_node( "~" );
  std::string fileName;
  priv_node.param< std::string >( "waypoint_file", fileName, "gps_test_waypoints.txt" );

  std::ifstream waypointsFile;
  waypointsFile.open( fileName.c_str() );
  if ( not waypointsFile.is_open() ) { return 1; }

  double degN, minN, secN, degE, minE, secE;
  while ( waypointsFile >> degN >> minN >> secN >> degE >> minE >> secE ) {
    double decimalN = deg_min_sec_to_decimal( degN, minN, secN );
    double decimalE = deg_min_sec_to_decimal( degE, minE, secE );
    geometry_msgs::Point newPoint;
    std::string zone;
    gps_common::LLtoUTM( decimalN, decimalE, newPoint.y, newPoint.x, zone );
    newPoint.z = 0;
    Waypoints.push_back( newPoint ); }
    
  correctedOdometry = node.advertise< geometry_msgs::PoseWithCovariance >( "relative", 1000 );
  ros::Rate loop_rate( 100 );
  ros::Subscriber absoluteOdometry = node.subscribe( "absolute", 1000,
						     &relativeizeOdometry );
  ros::ServiceServer Zero = node.advertiseService( "reset_zero_zero", &zero );
  ros::ServiceServer ZeroAtPoint = node.advertiseService( "set_zero_zero", &zero_at_point );
  ros::ServiceServer getWaypoints = node.advertiseService( "get_relative_waypoints", &get_relative_waypoints );
  ros::ServiceServer getZeroZero = node.advertiseService( "get_zero_zero", &get_zero_point );

  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep(); }

  return 0; }
