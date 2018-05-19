#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pubOdom, pubGPS, pubPositionEstimate, pubPose, pubGt, pubViso;
visualization_msgs::Marker odomPoints, gpsPoints, positionDisplayPoints, posePoints, gtPoints, laserOdomPoints, visoPoints;

double prevOdomTime, prevGPSTime, prevEstimateTime, prevPoseTime, prevGtTime, prevLaserOdomTime, prevVisoTime;
double displayInterval=0.2;

void visoCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if(fabs(prevVisoTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevVisoTime = ros::Time::now().toSec();
  visoPoints.header.frame_id = "/my_frame";
  visoPoints.header.stamp = ros::Time::now();
  visoPoints.ns = "points";
  visoPoints.action = visualization_msgs::Marker::ADD;
  visoPoints.pose.orientation.w = 1.0;
  visoPoints.id = 0;
  visoPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  visoPoints.scale.x = 0.2;
  visoPoints.scale.y = 0.2;
  // Points are blue
  visoPoints.color.g = 1.0f;
  visoPoints.color.b = 1.0f;
  visoPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  visoPoints.points.push_back(tempPoint);

  pubViso.publish(visoPoints);
}

void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if(fabs(prevGtTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevGtTime = ros::Time::now().toSec();
  gtPoints.header.frame_id = "/my_frame";
  gtPoints.header.stamp = ros::Time::now();
  gtPoints.ns = "points";
  gtPoints.action = visualization_msgs::Marker::ADD;
  gtPoints.pose.orientation.w = 1.0;
  gtPoints.id = 0;
  gtPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gtPoints.scale.x = 0.2;
  gtPoints.scale.y = 0.2;
  // Points are red
  gtPoints.color.r = 1.0f;
  gtPoints.color.g = 1.0f;
  gtPoints.color.b = 1.0f;
  gtPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.position.x;
  tempPoint.y = msg->pose.position.y;
  tempPoint.z = msg->pose.position.z;
  gtPoints.points.push_back(tempPoint);

  pubGt.publish(gtPoints);
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if(fabs(prevPoseTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevPoseTime = ros::Time::now().toSec();
  posePoints.header.frame_id = "/my_frame";
  posePoints.header.stamp = ros::Time::now();
  posePoints.ns = "points";
  posePoints.action = visualization_msgs::Marker::ADD;
  posePoints.pose.orientation.w = 1.0;
  posePoints.id = 0;
  posePoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  posePoints.scale.x = 0.2;
  posePoints.scale.y = 0.2;
  // Points are red
  posePoints.color.r = 1.0f;
  posePoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  posePoints.points.push_back(tempPoint);

  pubPose.publish(posePoints);
}


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
  positionDisplayPoints.scale.x = 0.2;
  positionDisplayPoints.scale.y = 0.2;
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

void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(fabs(prevLaserOdomTime-ros::Time::now().toSec())<displayInterval) {
    return;
  }
  prevLaserOdomTime = ros::Time::now().toSec();
  laserOdomPoints.header.frame_id = "/my_frame";
  laserOdomPoints.header.stamp = ros::Time::now();
  laserOdomPoints.ns = "points";
  laserOdomPoints.action = visualization_msgs::Marker::ADD;
  laserOdomPoints.pose.orientation.w = 1.0;
  laserOdomPoints.id = 0;
  laserOdomPoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  laserOdomPoints.scale.x = 0.2;
  laserOdomPoints.scale.y = 0.2;
  // Points are red
  laserOdomPoints.color.r = 1.0f;
  laserOdomPoints.color.a = 1.0;

  geometry_msgs::Point tempPoint;
  tempPoint.x = msg->pose.pose.position.x;
  tempPoint.y = msg->pose.pose.position.y;
  tempPoint.z = msg->pose.pose.position.z;
  laserOdomPoints.points.push_back(tempPoint);

  pubPose.publish(laserOdomPoints);
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
  odomPoints.scale.x = 0.2;
  odomPoints.scale.y = 0.2;
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
  gpsPoints.scale.x = 0.2;
  gpsPoints.scale.y = 0.2;
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

  prevOdomTime= prevGPSTime=prevEstimateTime =prevPoseTime=prevGtTime=prevLaserOdomTime=prevVisoTime= ros::Time::now().toSec();

  //odom subscriber
  ros::Subscriber subOdom = n.subscribe("/base_odometry/odom", 1000, encoderCallback);
  ros::Subscriber subEstimate = n.subscribe("/odometry/filtered", 1000, estimateCallback);
  ros::Subscriber subGPS = n.subscribe("/navsat/enu", 1000, gpsCallback);
  ros::Subscriber subPose = n.subscribe("/poseupdate", 1000, poseCallback);
  ros::Subscriber subGt = n.subscribe("/groundtruth_pose", 1000, gtCallback);
  ros::Subscriber subLaserOdom = n.subscribe("/laser_odom_rf2o", 1000, laserOdomCallback);
  ros::Subscriber subViso = n.subscribe("/fovis_stereo_odometer/odometry", 1000, visoCallback);

  pubPositionEstimate = n.advertise<visualization_msgs::Marker>("/estimate_position_marker", 10);
  pubOdom = n.advertise<visualization_msgs::Marker>("/odom_marker", 10);
  pubGPS = n.advertise<visualization_msgs::Marker>("/gps_marker", 10);
  pubPose = n.advertise<visualization_msgs::Marker>("/laserodom_marker", 10);
  pubGt = n.advertise<visualization_msgs::Marker>("/groundtruth_marker", 10);
  pubViso = n.advertise<visualization_msgs::Marker>("/viso_odom_marker", 10);

  ros::spin();

  return 0;
}

