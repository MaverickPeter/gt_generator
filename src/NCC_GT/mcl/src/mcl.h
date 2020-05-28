#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <random>
#include "tool.h"
#include <cmath>
//#include <opencv2/core/cuda_devptrs.hpp>
//cuda headers
//#include "common.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_functions.h>

using namespace cv;
__global__ void cuda_calculateNCC(uchar3 *template_image, uchar3 *global_roi, float *gpu_sum_img, float *gpu_sum_temp, float * gpu_sum_2, int width, int height);
float NCC(cv::Mat template_image, cv::Mat global_roi);
class mcl
{
  struct particle{
    Eigen::Matrix4f pose;
    float score;
    float theta;
    cv::Mat local_measurement;
    Eigen::Matrix4Xf scan; // Only for maximum probability particle.
  };

private:
  int m_sync_count;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()

  float imageResolution;
  float init_angle;
  float angle_search_area;
  float mapCenterX;
  float mapCenterY;
  float odomCovariance[6];
  int numOfParticle;
  std::vector<particle> particles;
  particle maxProbParticle;
  cv::Mat gridMap; // Gridmap for showing
  cv::Mat gridMapCV; // Gridmap for use (gaussian-blurred)
  Eigen::Matrix4f tf_laser2robot;
  Eigen::Matrix4f odomBefore;
  float minOdomDistance;
  float minOdomAngle;
  int template_size;
  int repropagateCountNeeded;

  bool isOdomInitialized;
  int predictionCounter;

  void initializeParticles();
  void prediction(Eigen::Matrix4f diffPose);
  void weightning(Eigen::Matrix4Xf laser);
  void weightning_NCC(cv::Mat template_image);
  void resampling();
  void showInMap();


public:
  mcl();
  ~mcl();
  void updateLaserData(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser);
  void updateImageData(Eigen::Matrix4f pose, cv::Mat local_measurement);

};



#endif

