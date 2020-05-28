#include "mcl.h"

using namespace std;
using namespace cv;

mcl::mcl()
{
  m_sync_count = 0;
  gen.seed(rd()); //Set random seed for random engine
  Mat temp;
  
  // gridMap = cv::imread("/home/mav-lab/slam_ws/src/mcl_2d_lidar_ros/global_ortho.png",cv::IMREAD_GRAYSCALE); //Original gridmap (For show)
  gridMapCV = cv::imread("/home/mav-lab/slam_ws/src/mcl_2d_lidar_ros/pixel_1.png",cv::IMREAD_GRAYSCALE); //grdiamp for use.
  // gridMapCV = tool::cvResizeMat(temp, 1 / 3.54);
  // gridMapCV = tool::cvRotateMat(temp, -27.8);
  cout<< gridMapCV.cols << " " << gridMapCV.rows << endl;

  //--YOU CAN CHANGE THIS PARAMETERS BY YOURSELF--//
  numOfParticle = 1000; // Number of Particles.
  minOdomDistance = 0.1; // [m]
  minOdomAngle = 0; // [deg]
  repropagateCountNeeded = 1; // [num]
  odomCovariance[0] = 0.00; // Rotation to Rotation
  odomCovariance[1] = 0.00; // Translation to Rotation
  odomCovariance[2] = 0.00; // Translation to Translation
  odomCovariance[3] = 0.00; // Rotation to Translation
  odomCovariance[4] = 0.0; // X
  odomCovariance[5] = 0.0; // Y
  template_size = 128; // Template(square) size
  init_angle = -93.2; // Rotation init guess [degree]
  angle_search_area = 3; // Searching area [degree]

  //--DO NOT TOUCH THIS PARAMETERS--//
  imageResolution = 0.1; // [m] per [pixel]
  tf_laser2robot << 1,0,0,0.0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1; // TF (laser frame to robot frame)
  mapCenterX = round(gridMapCV.cols/2) * imageResolution; // [m]
  mapCenterY = round(gridMapCV.rows/2) * imageResolution; // [m]
  isOdomInitialized = false; //Will be true when first data incoming.
  predictionCounter = 0;

  initializeParticles(); // Initialize particles.
  showInMap();
}

mcl::~mcl()
{

}

/* INITIALIZE PARTICLES UNIFORMLY TO THE MAP
 */
void mcl::initializeParticles()
{
  particles.clear();
  std::uniform_real_distribution<float> x_pos(0, gridMapCV.cols * imageResolution / 4);
  std::uniform_real_distribution<float> y_pos(0, 1 * gridMapCV.rows * imageResolution / 5); //heuristic setting. (to put particles into the map)
  // std::uniform_real_distribution<float> theta_pos((init_angle - angle_search_area) / 180 * M_PI, (init_angle + angle_search_area) / 180 * M_PI); // -180 ~ 180 Deg
  //SET PARTICLES BY RANDOM DISTRIBUTION
  for(int i=0;i<numOfParticle;i++)
  {
    particle particle_temp;
    float randomX = x_pos(gen);
    float randomY = y_pos(gen);
    // float randomTheta = theta_pos(gen);
    particle_temp.pose = tool::xyzrpy2eigen(randomX,randomY,0,0,0,0);
    particle_temp.score = 1 / (double)numOfParticle;
    // particle_temp.theta = randomTheta / M_PI * 180;
    particles.push_back(particle_temp);
  }
  showInMap();
}

void mcl::prediction(Eigen::Matrix4f diffPose)
{
  std::cout<<"Predicting..."<<m_sync_count<<std::endl;
  Eigen::VectorXf diff_xyzrpy = tool::eigen2xyzrpy(diffPose); // {x,y,z,roll,pitch,yaw} (z,roll,pitch assume to 0)

  //------------  FROM HERE   ------------------//
  //// Using odometry model
  double delta_trans = sqrt(pow(std::round(diff_xyzrpy(0)), 2)+ pow(std::round(diff_xyzrpy(1)),2));
  double delta_rot1 = atan2(diff_xyzrpy(1),diff_xyzrpy(0));
  double delta_rot2 = diff_xyzrpy(5) - delta_rot1;

  std::default_random_engine generator;
  if(delta_rot1  > M_PI)
          delta_rot1 -= (2*M_PI);
  if(delta_rot1  < -M_PI)
          delta_rot1 += (2*M_PI);
  if(delta_rot2  > M_PI)
          delta_rot2 -= (2*M_PI);
  if(delta_rot2  < -M_PI)
          delta_rot2 += (2*M_PI);
  //// Add noises to trans/rot1/rot2
  double trans_noise_coeff = odomCovariance[2]*fabs(delta_trans) + odomCovariance[3]*fabs(delta_rot1+delta_rot2);
  double rot1_noise_coeff = odomCovariance[0]*fabs(delta_rot1) + odomCovariance[1]*fabs(delta_trans);
  double rot2_noise_coeff = odomCovariance[0]*fabs(delta_rot2) + odomCovariance[1]*fabs(delta_trans);

  float scoreSum = 0;
  for(int i=0;i<particles.size();i++)
  {
    std::normal_distribution<double> gaussian_distribution(0, 1);

    delta_trans = delta_trans + gaussian_distribution(gen) * trans_noise_coeff;
    delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff;
    // delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff;

    // double x = delta_trans * cos(delta_rot1) + gaussian_distribution(gen) * odomCovariance[4];
    // double y = delta_trans * sin(delta_rot1) + gaussian_distribution(gen) * odomCovariance[5];

    double x = diff_xyzrpy(0) + gaussian_distribution(gen) * odomCovariance[4];
    double y = diff_xyzrpy(1) + gaussian_distribution(gen) * odomCovariance[5];
    // double theta = delta_rot1 + delta_rot2 + gaussian_distribution(gen) * odomCovariance[0]*(M_PI/180.0);

    Eigen::Matrix4f diff_odom_w_noise = tool::xyzrpy2eigen(x, y, 0, 0, 0, 0);

    Eigen::Matrix4f pose_t_plus_1 = particles.at(i).pose * diff_odom_w_noise;

    scoreSum = scoreSum + particles.at(i).score; // For normalization
    
    particles.at(i).pose = pose_t_plus_1;
    // particles.at(i).theta = theta / M_PI * 180;
  }

  //------------  TO HERE   ------------------//

  for(int i=0;i<particles.size();i++)
  {
    particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
  }

  showInMap();

}

void mcl::weightning(Eigen::Matrix4Xf laser)
{
  float maxScore = 0;
  float scoreSum = 0;

  /* Your work.
   * Input : laser measurement data
   * To do : update particle's weight(score)
   */

  for(int i=0;i<particles.size();i++)
  {

    Eigen::Matrix4Xf transLaser = particles.at(i).pose* tf_laser2robot* laser; // now this is lidar sensor's frame.

    //--------------------------------------------------------//

    float calcedWeight = 0;

    for(int j=0;j<transLaser.cols();j++)
    {
      int ptX  = static_cast<int>((transLaser(0, j) - mapCenterX + (300.0*imageResolution)/2)/imageResolution);
      int ptY = static_cast<int>((transLaser(1, j) - mapCenterY + (300.0*imageResolution)/2)/imageResolution);

      if(ptX<0 || ptX>=gridMapCV.cols || ptY<0 ||  ptY>=gridMapCV.rows) continue; // dismiss if the laser point is at the outside of the map.
      else
      {
        double img_val =  gridMapCV.at<uchar>(ptY,ptX)/(double)255; //calculate the score.
        calcedWeight += img_val; //sum up the score.
      }


    }
    particles.at(i).score = particles.at(i).score + (calcedWeight / transLaser.cols()); //Adding score to particle.
    scoreSum += particles.at(i).score;
    if(maxScore < particles.at(i).score) // To check which particle has max score
    {
      maxProbParticle = particles.at(i);
      maxProbParticle.scan = laser;
      maxScore = particles.at(i).score;
    }
  }
  for(int i=0;i<particles.size();i++)
  {
    particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
  }
}

void mcl::resampling()
{
  std::cout<<"Resampling..."<<m_sync_count<<std::endl;

  //Make score line (roullette)
  std::vector<double> particleScores;
  std::vector<particle> particleSampled;
  double scoreBaseline = 0;
  for(int i=0;i<particles.size();i++)
  {
    scoreBaseline += particles.at(i).score;
    particleScores.push_back(scoreBaseline);
  }

  std::uniform_real_distribution<double> dart(scoreBaseline/2, scoreBaseline);
  for(int i=0;i<particles.size();i++)
  {
    double darted = dart(gen); //darted number. (0 to maximum scores)
    auto lowerBound = std::lower_bound(particleScores.begin(), particleScores.end(), darted);
    int particleIndex = lowerBound - particleScores.begin(); // Index of particle in particles.

    //TODO : put selected particle to array 'particleSampled' with score reset.

    particle selectedParticle = particles.at(particleIndex); // Which one you have to select?
    selectedParticle.score = 1 / (double)particles.size();
    particleSampled.push_back(selectedParticle);

  }
  particles = particleSampled;
}


//DRAWING FUNCTION.
void mcl::showInMap()
{
//  cv::Mat showMap(gridMap.cols, gridMap.rows, CV_8UC3);
  cv::Mat showMap;
  cv::cvtColor(gridMapCV, showMap, cv::COLOR_GRAY2BGR);

  for(int i=0;i<numOfParticle;i++)
  {
    float part_x = (particles.at(i).pose(0, 3)) / imageResolution;
    float part_y = (particles.at(i).pose(1, 3)) / imageResolution;

    int xPos  = static_cast<int>(std::round(part_x) + template_size / 2);
    int yPos = static_cast<int>(std::round(part_y) + template_size / 2);

    cv::circle(showMap,cv::Point(yPos,xPos),1,cv::Scalar(255,0,0),-1);
  }
  if(maxProbParticle.score > 0)
  {
    //// Original
  //  int xPos = static_cast<int>((maxProbParticle.pose(0, 3) - mapCenterX + (300.0*imageResolution)/2)/imageResolution);
  //  int yPos = static_cast<int>((maxProbParticle.pose(1, 3) - mapCenterY + (300.0*imageResolution)/2)/imageResolution);

    //// Estimate position using all particles
    float x_all = 0;
    float y_all = 0;
    for(int i=0;i<particles.size();i++)
    {
      x_all = x_all + particles.at(i).pose(0,3) * particles.at(i).score;
      y_all = y_all + particles.at(i).pose(1,3) * particles.at(i).score;
    }
    int xPos = static_cast<int>(std::round(x_all / imageResolution) + template_size / 2);
    int yPos = static_cast<int>(std::round(y_all / imageResolution) + template_size / 2);

    cv::circle(showMap,cv::Point(yPos,xPos),2,cv::Scalar(0,0,255),-1);

  }
  cv::imshow("MCL2", showMap);
  cv::waitKey(1);
}

void mcl::updateLaserData(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser)
{
  if(!isOdomInitialized)
  {
    odomBefore = pose; // Odom used at last prediction.
    isOdomInitialized = true;
  }
  //When difference of odom from last correction is over some threshold, Doing correction.

  //Calculate difference of distance and angle.

  Eigen::Matrix4f diffOdom = odomBefore.inverse()*pose; // odom after = odom New * diffOdom
  Eigen::VectorXf diffxyzrpy = tool::eigen2xyzrpy(diffOdom); // {x,y,z,roll,pitch,yaw}
  float diffDistance = sqrt(pow(diffxyzrpy[0],2)+pow(diffxyzrpy[1],2));
  float diffAngle = fabs(diffxyzrpy[5])*180.0/3.141592;

  if(diffDistance>minOdomDistance || diffAngle>minOdomAngle)
  {
    //Doing correction & prediction
    cout << "Predicting step" << endl;
    prediction(diffOdom);
        
    cout << "Weightning step" << endl;
    weightning(laser);

    predictionCounter++;
    if(predictionCounter == repropagateCountNeeded)
    {
      resampling();
      predictionCounter = 0;
    }

    m_sync_count = m_sync_count + 1;
    odomBefore = pose;
  }
}

float NCC(cv::Mat template_image, cv::Mat global_roi)
{
  uchar3 *gpu_template;
  uchar3 *gpu_roi;

  float sum_img;
  float sum_temp;
  float sum_2;
  float Weight;

  float *gpu_sum_img = NULL;
  float *gpu_sum_temp = NULL;
  float *gpu_sum_2 = NULL;

  cudaMalloc((void**)&gpu_template, template_image.cols * template_image.rows * sizeof(uchar3));
  cudaMalloc((void**)&gpu_roi, global_roi.cols * global_roi.rows * sizeof(uchar3));

  cudaMemcpy(gpu_template, template_image.data, template_image.cols * template_image.rows * sizeof(uchar3), cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_roi, global_roi.data, global_roi.cols * global_roi.rows * sizeof(uchar3), cudaMemcpyHostToDevice);

  //set threads block and grid

  //Specify a reasonable block size
  const dim3 block(32,32);

  //Calculate grid size to cover the whole image
  const dim3 grid((template_image.cols + block.x - 1)/block.x, (template_image.rows + block.y - 1)/block.y);
  //initiate some value and pass to cuda shared memory
  // const dim3 grid(256/block.x, 256/block.y);


  cudaMalloc((void**)&gpu_sum_img, sizeof(float));
  cudaMemcpy(gpu_sum_img, &sum_img, sizeof(float), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&gpu_sum_temp, sizeof(float));
  cudaMemcpy(gpu_sum_temp, &sum_temp, sizeof(float), cudaMemcpyHostToDevice);

  cudaMalloc((void**)&gpu_sum_2, sizeof(float));
  cudaMemcpy(gpu_sum_2, &sum_2, sizeof(float), cudaMemcpyHostToDevice);
  // call function cuda_calculate

  cuda_calculateNCC<<<grid, block>>>(gpu_template, gpu_roi, gpu_sum_img, gpu_sum_temp, gpu_sum_2, template_image.cols, template_image.rows);

  // restore the cuda value back to cpu 
  cudaMemcpy(&sum_img, gpu_sum_img, sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(&sum_temp, gpu_sum_temp, sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(&sum_2, gpu_sum_2, sizeof(float), cudaMemcpyDeviceToHost);
  // cout << "Weight: " << Weight << " sum_temp: " << sum_temp << " sum_img: " << sum_img << " sum_2: " << sum_2 << endl;

  Weight = (sum_2) / sqrt(sum_img * sum_temp) * 1000;
  // cout << "Weight: " << Weight << endl;
  
  cudaFree(gpu_template);
  cudaFree(gpu_roi);
  cudaFree(gpu_sum_img);
  cudaFree(gpu_sum_temp);
  cudaFree(gpu_sum_2);
  return Weight;
}

__global__ void cuda_calculateNCC(uchar3 *template_image, uchar3 *global_roi, float *gpu_sum_img, float *gpu_sum_temp, float * gpu_sum_2, int width, int height)
{
  //2D Index of current thread
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int k = blockIdx.y * blockDim.y + threadIdx.y;

  if(j <= width && k <= height){
    uchar3 template_rgb = template_image[k*width + j];
    uchar3 global_rgb = global_roi[k*width + j];
    // printf("gpu_sum_temp = %f \n", pow((template_rgb.x),2));
    if(template_rgb.x != 255 && global_rgb.x != 0){
      atomicAdd(gpu_sum_temp, pow((template_rgb.x),2));
      atomicAdd(gpu_sum_img, pow((global_rgb.x),2));
      atomicAdd(gpu_sum_2, (2 - abs((width / 2) - j) / (width / 2) * abs((height / 2) - j)/(height / 2)) * global_rgb.x * template_rgb.x);
    }
  }
}

void mcl::weightning_NCC(cv::Mat template_image)
{
  float maxScore = 0;
  float scoreSum = 0;
  cv::Mat contrast_image, tmp, image;
  contrast_image = Mat::zeros(template_image.size(), template_image.type());

  for (int y = 0; y < template_image.rows; y++) {
		for (int x = 0; x < template_image.cols; x++) {
			for (int c = 0; c < 3; c++) {
				contrast_image.at<Vec3b>(y, x)[c] = saturate_cast<uchar>((180*0.01)*(template_image.at<Vec3b>(y, x)[c]) - 10);
			}
		}
	}
  cvtColor(contrast_image, tmp, COLOR_BGR2GRAY);

  image = tool::cvRotateMat(tmp, -93.2);
  cv::imshow("template", image);
  cv::waitKey(1);

  for(int i=0;i<particles.size();i++)
  {

    Eigen::Matrix4Xf particle_pose = particles.at(i).pose;
    float particle_rot = particles.at(i).theta;

    cv::Mat global_imroi;
    if(particles.at(i).pose(0, 3) > 0 && particles.at(i).pose(1, 3) > 0 && particles.at(i).pose(0, 3) < (gridMapCV.cols - template_image.cols) * imageResolution 
      && particles.at(i).pose(1, 3) < (gridMapCV.rows - template_image.rows) * imageResolution){
      cv::Rect rect(int(particle_pose(0,3) / imageResolution), int(particle_pose(1,3) / imageResolution), image.cols, image.rows);
      // cout << "particle_pose(0,3) = " << particle_pose(0,3) / imageResolution << " particle_pose(1,3) = " << particle_pose(1,3) / imageResolution << " image.rows = " << image.rows << " image.cols = " << image.cols << endl;
      global_imroi = gridMapCV(rect);
          
      cv::imshow("particle view", global_imroi);
      cv::waitKey(1);
    }else{
      continue;
    }

    double sum_img = 0.0;
    double sum_temp = 0.0;
    double sum_2 = 0.0;
    float calcedWeight = 0.0;

    // NCC
    calcedWeight = NCC(image, global_imroi);

    // cout << " calcedWeight = " << calc << endl;
    particles.at(i).score = particles.at(i).score + 1000 * calcedWeight - 1000; //Adding score to particle.
    scoreSum += particles.at(i).score;
    if(maxScore < particles.at(i).score) // To check which particle has max score
    {
      maxProbParticle = particles.at(i);
      maxProbParticle.local_measurement = image;
      maxScore = particles.at(i).score;
    }
  }

  cout << " max score = "<< maxScore << endl;
  for(int i=0;i<particles.size();i++)
  {
    particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
  }
}

void mcl::updateImageData(Eigen::Matrix4f pose, cv::Mat local_measurement)
{
  if(!isOdomInitialized)
  {
    odomBefore = pose; // Odom used at last prediction.
    isOdomInitialized = true;
  }
  Eigen::Matrix4f diffOdom = odomBefore.inverse() * pose; // odom after = odom New * diffOdom
  Eigen::VectorXf diffxyzrpy = tool::eigen2xyzrpy(diffOdom); // {x,y,z,roll,pitch,yaw}
  float diffDistance = sqrt(pow(diffxyzrpy[0],2) + pow(diffxyzrpy[1],2));
  float diffAngle = fabs(diffxyzrpy[5]) * 180.0 / 3.141592;

  if(diffDistance>minOdomDistance || diffAngle>minOdomAngle)
  {
    //Doing correction & prediction
    cout << "Predicting step x = " << diffOdom(0,3) << " y = " << diffOdom(1,3) << endl;
    cout << "Pose step x = " << pose(0,3) << " y = " << pose(1,3) << endl;

    prediction(diffOdom);

    cout << "Weightning_NCC step" << endl;
    // cv::imshow("test", local_measurement);
    // cv::waitKey();
    weightning_NCC(local_measurement);

    predictionCounter++;
    if(predictionCounter == repropagateCountNeeded)
    {
      cout << "Resampling step" << endl;
      resampling();
      predictionCounter = 0;
    }

    m_sync_count = m_sync_count + 1;
    odomBefore = pose;
  }
}