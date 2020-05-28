#include <ros/ros.h>
#include "src/mcl.h"

#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <image_mcl/Pose.h>
#include <algorithm> 
#include "src/mcl.h"

#define PI 3.14159265

using namespace std;
using namespace cv;

float str2float(string& s)
{
  float num;
  stringstream ss(s);
  ss >> num;
  return num;
}

bool NumericGreaterThan(const std::string& lhs, const std::string& rhs)
{
  int lhsfileNamePos = lhs.find_last_of('/');
  string lhsfileName = lhs.substr(lhsfileNamePos + 1);
  int lhsindexPos = lhsfileName.find_first_of('_');
  string lhsindex = lhsfileName.substr(0, lhsindexPos);

  int rhsfileNamePos = rhs.find_last_of('/');
  string rhsfileName = rhs.substr(rhsfileNamePos + 1);
  int rhsindexPos = rhsfileName.find_first_of('_');
  string rhsindex = rhsfileName.substr(0, rhsindexPos);

  return atoi(lhsindex.c_str()) < atoi(rhsindex.c_str());
}

int readFileList(char *basePath, vector<string>& files)
{
  DIR *dir;
  struct dirent *dirptr;

  if((dir = opendir(basePath)) == NULL)
    {
      perror("Open dir error!!!");
      return 1;
    }

  int count = 0;
  while((dirptr = readdir(dir)) != NULL)
    {
      if(strcmp(dirptr->d_name, ".") == 0 || strcmp(dirptr->d_name, "..") == 0)
        continue;

      else if(dirptr->d_type == 8)       //file
        {
          files.push_back(dirptr->d_name);
          count++;
        }

      //else if(dirptr->d_type == 10)    //link file
      //else if(dirptr->d_type == 4)     //dir
    }

  //排序，按从小到大排序
  sort(files.begin(), files.end());

  closedir(dir);

  return count;
}

void GetFileNames(string path, vector<string>& filenames)
{
  DIR *pDir;
  struct dirent* ptr;
  if(!(pDir = opendir(path.c_str()))){
    return;
  }
  while((ptr = readdir(pDir))!=0) {
      if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
          filenames.push_back(path + "/" + ptr->d_name);
  }
  sort(filenames.begin(), filenames.end(), NumericGreaterThan);
  closedir(pDir);
}

void split(const std::string& str, vector<int>& info)
{
  int fileNamePos = str.find_last_of('/');
  string fileName = str.substr(fileNamePos + 1);

  int indexPos = fileName.find_first_of('_');
  string index = fileName.substr(0, indexPos);
  info.push_back(stoi(index));
  
  string remain = fileName.substr(indexPos + 1);
  int xPos = remain.find_first_of('_');
  string x = remain.substr(0, xPos);
  info.push_back(str2float(x)*10); // 1 pix equals 0.1m

  string remain_y = remain.substr(xPos + 1);
  int yEnd = remain_y.find_last_of('.');
  string y = remain_y.substr(0, yEnd);
  info.push_back(str2float(y)*10); // 1 pix equals 0.1m

}

class gt_generators
{
  public:

  gt_generators(const ros::NodeHandle& nh_);
  ~gt_generators();
  void gt_reader(const image_mcl::Pose::ConstPtr& pose);

  private:

  int imageIndex;
  ros::NodeHandle nh;
  cv::Mat aerialImage;
  vector<string> fileName;
  ros::Subscriber pose_sub_;
  ofstream outFile;
};
gt_generators::~gt_generators()
{
  outFile.close();
}

gt_generators::gt_generators(const ros::NodeHandle& nh_): nh(nh_)
{
  pose_sub_ = nh.subscribe("/turtle1/pose", 1, &gt_generators::gt_reader, this);
  string path = "/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/zjg_image_qsdjt";
  string aerialImagePath = "/home/mav-lab/Projects/SLAM/AG_gt/NCC_GT/mcl/test.png";
  GetFileNames(path, fileName);
  // for(int i = 0; i < fileName.size(); i++)
  //   std::cout << "filename = " << fileName[i] << std::endl;
  aerialImage = cv::imread(aerialImagePath);
  imageIndex = 0;
};

void gt_generators::gt_reader(const image_mcl::Pose::ConstPtr& Pose){

  cv::Mat image = imread(fileName[imageIndex]);

	outFile.open("/home/mav-lab/data_new.csv", ios::app);
	outFile << fileName[imageIndex] << ',' << Pose->x * 45 << ',' << Pose->y * 45 << endl;
  outFile.close();
  imshow("image",image);
  cv::waitKey(100);
  imageIndex++;
  
};



void visualGt()
{
  vector<string> fileName;
  string path = "/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/zjg_image_qsdjt";
  string aerialImagePath = "/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/test.png";
  GetFileNames(path, fileName);
  cv::Mat aerialImage = cv::imread(aerialImagePath, CV_LOAD_IMAGE_GRAYSCALE);
  cv::resize(aerialImage, aerialImage, cv::Size((aerialImage.cols-1)/2.0,(aerialImage.rows-1)/2.0));

  ifstream inFile("/home/mav-lab/NCC_out.csv", ios::in);
  if (!inFile)
  {
      cout << "打开文件失败！" << endl;
      exit(1);
  }

  int i = 0;
  string line;
  string field;
  
  while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
  {
    cv::Mat image = imread(fileName[i], CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(image, image, cv::Size((image.cols-1)/2.0,(image.rows-1)/2.0));
    float angle = 162.0-90.0;
    cv::Point2f center((image.cols-1)/2.0, (image.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - image.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - image.rows/2.0;

    cv::Mat rotImage;
    cv::warpAffine(image, rotImage, rot, cv::Size(100,100));
    string x, y, name;
    istringstream sin(line); //将整行字符串line读入到字符串流sin中

    getline(sin, name, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
    getline(sin, x, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
    getline(sin, y, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 

    i++;
    
    // imshow("image",croppedImage);
    // cv::waitKey(0);
    // imshow("image",rotImage);
    // cv::waitKey(0);
    cv::Rect myROI(atoi(x.c_str()), atoi(y.c_str()), 100, 100);
    cv::Mat croppedImage = aerialImage(myROI);
    imshow("aerial image",croppedImage);
    cv::waitKey(100);
    imshow("image",rotImage);
    cv::waitKey(100);
  }
  inFile.close();
}

void NCCGtGenerator() 
{
  vector<string> fileName;
  string path = "/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/zjg_image_qsdjt";
  string aerialImagePath = "/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/test.png";
  GetFileNames(path, fileName);
  cv::Mat aerialImage = cv::imread(aerialImagePath, CV_LOAD_IMAGE_GRAYSCALE);
  cv::resize(aerialImage, aerialImage, cv::Size((aerialImage.cols-1)/2.0,(aerialImage.rows-1)/2.0));
  ofstream outFile;
  outFile.open("/home/mav-lab/NCC_out.csv", ios::app);

  ifstream inFile("/home/mav-lab/data_new.csv", ios::in);
  if (!inFile)
  {
      cout << "打开文件失败！" << endl;
      exit(1);
  }

  int i = 0;
  string line;
  string field;
  
  while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
  {
    cv::Mat image = imread(fileName[i], CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(image, image, cv::Size((image.cols-1)/2.0,(image.rows-1)/2.0));
    float angle = 72.0;
    cv::Point2f center((image.cols-1)/2.0, (image.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - image.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - image.rows/2.0;

    cv::Mat rotImage;
    cv::warpAffine(image, rotImage, rot, cv::Size(100,100));
    string x, y, name;
    istringstream sin(line); //将整行字符串line读入到字符串流sin中

    getline(sin, name, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
    getline(sin, x, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
    getline(sin, y, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 

    i++;
    
    float max = -10.0;
    int xIndex = 0;
    int yIndex = 0;
    float score = 0.0;
    for(int rows = 0; rows < 80; rows++){
      for(int cols = 0; cols < 80; cols++){
        if(atoi(x.c_str())-30+rows < aerialImage.cols && atoi(y.c_str())-50+cols < aerialImage.rows &&
          atoi(x.c_str())-30+rows > 0 && atoi(y.c_str())-50+cols > 0){
          cv::Rect myROI(atoi(x.c_str())-30+rows, atoi(y.c_str())-50+cols, 100, 100);
          cv::Mat croppedImage = aerialImage(myROI);
          score = NCC(rotImage.clone(), croppedImage.clone());

          if(score > max){
            max = score;
            xIndex = rows;
            yIndex = cols;
          }
        }
      }
    }

    // imshow("image",croppedImage);
    // cv::waitKey(0);
    // imshow("image",rotImage);
    // cv::waitKey(0);
    // cv::Rect myROI(atoi(x.c_str())-30+xIndex, atoi(y.c_str())-50+yIndex, 100, 100);
    // cv::Mat croppedImage = aerialImage(myROI);
    // imshow("image",croppedImage);
    // cv::waitKey(1000);
    // imshow("image",rotImage);
    // cv::waitKey(1000);
    cout << "max score = " << max << " xIndex = " << xIndex << " yIndex = " << yIndex << endl;
    outFile << fileName[i] << ',' << atoi(x.c_str())-30+xIndex << ',' << atoi(y.c_str())-50+yIndex << endl;
  }
  inFile.close();
  outFile.close();
}

int main(int argc, char **argv)
{  
  // ros::init(argc, argv, "gt_generators");
  // ros::NodeHandle nh_;
  // gt_generators generator(nh_);
  // ros::spin();

  // NCCGtGenerator();

  visualGt();
}