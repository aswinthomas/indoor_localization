#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pubOdom, pubGPS, pubPositionEstimate;
visualization_msgs::Marker odomPoints, gpsPoints, positionDisplayPoints;

double prevOdomTime, prevGPSTime, prevEstimateTime;
double displayInterval=0.2;

void estimateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if(fabs(prevEstimateTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevEstimateTime = ros::Time::now().toSec();
  positionDisplayPoints.header.frame_id = "/my_frame";
  positionDisplayPoints.header.stamp = ros::Time::now();
  positionDisplayPoints.ns = "points";
  positionDisplayPoints.action = visualization_msgs::Marker::ADD;
  positionDisplayPoints.pose.orientation.w = 1.0;
  positionDisplayPoints.id = 0;
  positionDisplayPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  positionDisplayPoints.scale.x = 1.0;
  positionDisplayPoints.scale.y = 1.0;
  // Points are green
  positionDisplayPoints.color.g = 1.0f;
  positionDisplayPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  positionDisplayPoints.points.push_back(tempPoint);

  pubPositionEstimate.publish(positionDisplayPoints);
}

void encoderCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(fabs(prevOdomTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevOdomTime = ros::Time::now().toSec();
  odomPoints.header.frame_id = "/my_frame";
  odomPoints.header.stamp = ros::Time::now();
  odomPoints.ns = "points";
  odomPoints.action = visualization_msgs::Marker::ADD;
  odomPoints.pose.orientation.w = 1.0;
  odomPoints.id = 0;
  odomPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  odomPoints.scale.x = 1.0;
  odomPoints.scale.y = 1.0;
  // Points are yellow
  odomPoints.color.g = 1.0f;
  odomPoints.color.r = 1.0f;
  odomPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  odomPoints.points.push_back(tempPoint);

  pubOdom.publish(odomPoints);
}


void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(fabs(prevGPSTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevGPSTime = ros::Time::now().toSec();
  gpsPoints.header.frame_id = "/my_frame";
  gpsPoints.header.stamp = ros::Time::now();
  gpsPoints.ns = "points";
  gpsPoints.action = visualization_msgs::Marker::ADD;
  gpsPoints.pose.orientation.w = 1.0;
  gpsPoints.id = 0;
  gpsPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpsPoints.scale.x = 1.0;
  gpsPoints.scale.y = 1.0;
  // Points are yellow
  gpsPoints.color.r = 1.0f;
  gpsPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  gpsPoints.points.push_back(tempPoint);

  pubGPS.publish(gpsPoints);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_to_marker");

  ros::NodeHandle n("~");

  prevOdomTime= prevGPSTime=prevEstimateTime = ros::Time::now().toSec();

  //odom subscriber
  ros::Subscriber subOdom = n.subscribe("/encoder", 1000, encoderCallback);
  ros::Subscriber subEstimate = n.subscribe("/outdoor_waypoint_nav/odometry/filtered", 1000, estimateCallback);
  ros::Subscriber subGPS = n.subscribe("/navsat/enu", 1000, gpsCallback);

  pubPositionEstimate = n.advertise<visualization_msgs::Marker>("/estimate_position_marker", 10);
  pubOdom = n.advertise<visualization_msgs::Marker>("/odom_marker", 10);
  pubGPS = n.advertise<visualization_msgs::Marker>("/gps_marker", 10);

  ros::spin();

  return 0;
}

