#ifndef TOOL_H
#define TOOL_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_functions.h>

namespace tool
{
  cv::Mat cvMaptoMCLMap(cv::Mat mat);
  cv::Mat cvRotateMat(cv::Mat src, double angle);
  cv::Mat cvResizeMat(cv::Mat src, double scale);
  Eigen::Matrix4f mat2eigen(cv::Mat mat);
  cv::Mat eigen2mat(Eigen::Matrix4f mat);
  cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);
  void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
  Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat);
  Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw);
  double GaussianRand();
}
#endif
