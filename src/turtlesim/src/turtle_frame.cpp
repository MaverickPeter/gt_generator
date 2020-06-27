/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtlesim/turtle_frame.h"

#include <QPointF>
#include <QLabel>
#include <QMainWindow>
#include <QScrollArea>
#include <ros/package.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

using namespace std;

namespace turtlesim
{

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
// , path_image_("/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/test.png")
, path_painter_(&path_image_)
, image_id_(40)
, image_size(260)
, image_step(2)
, frame_count_(0)
, id_counter_(0)
, current_angle_(0.0)
, private_nh_("~")
{
  QImage image("/home/mav-lab/qsdjt_sat.png");
  image = image.scaled(2953, 1590);
  path_image_ = image;
  setFixedSize(2953, 1590);
  setWindowTitle("TurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(10);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  if (!private_nh_.hasParam("background_r"))
  {
    private_nh_.setParam("background_r", DEFAULT_BG_R);
  }
  if (!private_nh_.hasParam("background_g"))
  {
    private_nh_.setParam("background_g", DEFAULT_BG_G);
  }
  if (!private_nh_.hasParam("background_b"))
  {
    private_nh_.setParam("background_b", DEFAULT_BG_B);
  }

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");
  turtles.append("kinetic.png");
  turtles.append("lunar.png");
  turtles.append("melodic.png");
  turtles.append("noetic.png");

  QVector<QString> groundImageStr;

  QString images_path = (ros::package::getPath("turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    // img.load("/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/zjg_image_qsdjt/0.jpg");
    img.load("/media/mav-lab/1T/Datasets/zjg_AG/qiushidabuilding/zjg_image/40.jpg");
    img = img.scaled(image_size, image_size, Qt::KeepAspectRatio);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();
  ROS_INFO("width = %d,  height = %d", width(), height());
  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);
  save_signal_sub_ = nh_.subscribe("/turtle1/save", 1, &TurtleFrame::saveAndSpawnCallback, this);
  skip_signal_sub_ = nh_.subscribe("/turtle1/skip", 1, &TurtleFrame::skipAndSpawnCallback, this);

  pose_sub_ = nh_.subscribe("/turtle/pose", 1, &TurtleFrame::poseCallback, this);
  trans_sub_ = nh_.subscribe("/turtle1/trans", 1, &TurtleFrame::transCallback, this);
  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", 8.8, 1.45, 3.116700);
  current_x = 8.8;
  current_y = 1.45;
  current_angle_ = 3.116700;

  // spawn all available turtle types
  if(false)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, image_id_);
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }
  std::ostringstream cc;
  cc << index;
  currentName_ = ("/media/mav-lab/1T/Datasets/zjg_AG/qiushidabuilding/zjg_image/" + cc.str() + ".jpg").c_str();
  // currentName_ = "/home/mav-lab/r_-76.34_x_792_y_176.jpg";
  QImage groundImage;
  groundImage.load(currentName_);
  groundImage = groundImage.scaled(image_size, image_size, Qt::KeepAspectRatio);

  groundImage.convertToFormat(QImage::Format_ARGB32);

  groundImage = setOpacity(groundImage, 0.7);
  TurtlePtr t(new Turtle(ros::NodeHandle("turtle"), groundImage, QPointF(x, y), angle));
  turtles_[real_name] = t;
  update();

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  private_nh_.param("background_r", r, r);
  private_nh_.param("background_g", g, g);
  private_nh_.param("background_b", b, b);
  
  // QMainWindow mw;
  // QLabel *label=new QLabel();
  // QImage image("/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/test.png");
  // label->setPixmap(QPixmap::fromImage(image));

  // QScrollArea *sa=new QScrollArea(&mw);
  // sa->setWidget(label);
  // sa->resize(400,400);
  // mw.show();

  // QImage image("/home/mav-lab/Datasets/zjg_AG/qiushidabuilding/test.png");
  // path_image_= image;
  // path_image_.fill(qRgb(r, g, b));
  update();
}

void TurtleFrame::onUpdate()
{
  ros::spinOnce();

  updateTurtles();

  if (!ros::ok())
  {
    close();
  }
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.0002 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}

bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

void TurtleFrame::saveAndSpawnCallback(const std_msgs::Bool::ConstPtr& saveSignal)
{
  if(saveSignal->data == true)
  {
    cout << "Saving" << endl;
    M_Turtle::iterator it = turtles_.begin();
    turtles_.erase(it);
    spawnTurtle("", current_x, current_y, current_angle_, image_id_);
    image_id_+=image_step;

    std::ofstream outFile("/home/mav-lab/gt_I2S_qsdjt_manual.csv", ios::app);
    outFile << q2s(currentName_) << ',' << current_x * image_size << ',' << current_y * image_size << ',' << current_angle_ / PI * 180.0 - 90.0 << endl;
    outFile.close();
  }
}

void TurtleFrame::skipAndSpawnCallback(const std_msgs::Bool::ConstPtr& skipSignal)
{
  if(skipSignal->data == true)
  {
    cout << "Skip" << endl;
    M_Turtle::iterator it = turtles_.begin();
    turtles_.erase(it);
    spawnTurtle("", current_x, current_y, current_angle_, image_id_);
    image_id_+=image_step;

    // std::ofstream outFile("/home/mav-lab/gt_laser_qsdjt_manual.csv", ios::app);
    // outFile << q2s(currentName_) << ',' << current_x * image_size << ',' << current_y * image_size << ',' << current_angle_ / PI * 180.0 - 90.0 << endl;
    // outFile.close();
  }
}


void TurtleFrame::poseCallback(const turtlesim::Pose::ConstPtr& pose)
{
  current_x = pose->x;
  current_y = pose->y;
  current_angle_ = pose->theta;
}

void TurtleFrame::transCallback(const geometry_msgs::Twist::ConstPtr& trans)
{
  trans_dx = trans->angular.z;
  trans_dy = trans->linear.x;

  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->updateTrans(trans_dx, trans_dy, path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }
}

// opacity
QImage TurtleFrame::setOpacity(QImage& image, qreal opacity)
{
  QImage newImg(image.size(), QImage::Format_ARGB32);
  newImg.fill(Qt::transparent);

  QPainter painter(&newImg);
  painter.setOpacity(opacity);
  painter.drawImage(QRect(0, 0, image.width(), image.height()), image);

  return newImg;
}

// String to QString
QString TurtleFrame::s2q(const std::string &s)   
{   
return QString(QString::fromLocal8Bit(s.c_str()));   
}

// QString to string
std::string TurtleFrame::q2s(const QString &s)   
{   
return string((const char *)s.toLocal8Bit());   
}  

}
