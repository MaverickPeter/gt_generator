#include <ros/ros.h>
#include "src/mcl.h"

using namespace std;

std::vector<Eigen::Matrix4f> vec_poses;
std::vector<double> vec_poses_time;
std::vector<Eigen::Matrix4Xf> vec_lasers;
std::vector<double>vec_lasers_time;
std::vector<cv::Mat> vec_images;
std::vector<double>vec_images_time;

mcl mclocalizer;
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_image(const sensor_msgs::Image::ConstPtr & msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);
void check_data_laser();
void check_data_image();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_mcl");
  ros::NodeHandle nh;
  ros::Subscriber subscribe_image = nh.subscribe<sensor_msgs::Image>("/elevation_mapping/orthomosaic", 100, callback_image);
  ros::Subscriber subscribe_laser = nh.subscribe<sensor_msgs::LaserScan>("/scan",100, callback_laser);
  ros::Subscriber subscribe_pose = nh.subscribe<nav_msgs::Odometry>("/odom", 100, callback_pose);
  ros::spin();

  return 0;
}


void check_data_laser()
{
  while((vec_poses.size()!=0 && vec_lasers.size()!=0))
  {
    if(fabs(vec_poses_time[0] - vec_lasers_time[0])>0.1)
    {
      if(vec_poses_time[0]>vec_lasers_time[0])
      {
        vec_lasers.erase(vec_lasers.begin());
        vec_lasers_time.erase(vec_lasers_time.begin());
      }
      else
      {
        vec_poses.erase(vec_poses.begin());
        vec_poses_time.erase(vec_poses_time.begin());
      }
    }
    else
    {
      mclocalizer.updateLaserData(vec_poses[0],vec_lasers[0]);
      vec_lasers.erase(vec_lasers.begin());
      vec_lasers_time.erase(vec_lasers_time.begin());
      vec_poses.erase(vec_poses.begin());
      vec_poses_time.erase(vec_poses_time.begin());
    }
  }
}

void check_data_image()
{
  while((vec_poses.size()!=0 && vec_images.size()!=0))
  {
    if(fabs(vec_poses_time[0] - vec_images_time[0])>0.1)
    {
      if(vec_poses_time[0]>vec_images_time[0])
      {
        vec_images.erase(vec_images.begin());
        vec_images_time.erase(vec_images_time.begin());
        cout << "erase image" << endl;
      }
      else
      {
        vec_poses.erase(vec_poses.begin());
        vec_poses_time.erase(vec_poses_time.begin());
        cout << "erase odom" << endl;
      }
    }
    else
    {
      mclocalizer.updateImageData(vec_poses[0],vec_images[0]);
      vec_images.erase(vec_images.begin());
      vec_images_time.erase(vec_images_time.begin());
      vec_poses.erase(vec_poses.begin());
      vec_poses_time.erase(vec_poses_time.begin());
      cout << "no time clip" << endl;
    }
    // cout << "posex: " << vec_poses[0](0,3) << " posey: " << vec_poses[0](1,3)<< endl;
  }
}

void callback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  int scanQuantity =((msg->angle_max)-(msg->angle_min))/(msg->angle_increment)+1;
  Eigen::Matrix4Xf eigenLaser = Eigen::Matrix4Xf::Ones(4, 1);
  int scanEffective = 0;
  for(int i=0;i<scanQuantity;i++){
    float dist = msg->ranges[i];
    if(dist > 1 && dist < 10)
    {
      scanEffective++;
      eigenLaser.conservativeResize(4,scanEffective);
      eigenLaser(0,scanEffective-1) =  dist * cos(msg->angle_min + ( msg->angle_increment * i));
      eigenLaser(1,scanEffective-1) =  dist * sin(msg->angle_min + ( msg->angle_increment * i));
      eigenLaser(2,scanEffective-1) =  0;
      eigenLaser(3,scanEffective-1) =  1;
    }
  }
  vec_lasers.push_back(eigenLaser);
  vec_lasers_time.push_back(msg->header.stamp.toSec());
  check_data_laser();
}

void callback_pose(const nav_msgs::Odometry::ConstPtr &msg)
{
  Eigen::Matrix4f eigenPose;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  eigenPose<< m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
              m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
              m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
              0,0,0,1;
  // cout << "eigenPose" << eigenPose << endl;
  Eigen::Matrix4f static_rot = tool::xyzrpy2eigen(0,0,0,0,0,93.2);
  eigenPose = static_rot * eigenPose ;
  //cout << "eigenPose" << msg->pose.pose.position.x << " y: " <<msg->pose.pose.position.y << endl;

  vec_poses.push_back(eigenPose);
  vec_poses_time.push_back(msg->header.stamp.toSec());
  check_data_image();
}

void callback_image(const sensor_msgs::Image::ConstPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr -> image;

  vec_images.push_back(img);
  vec_images_time.push_back(msg->header.stamp.toSec());
  check_data_image();
}