#ifndef TORCS_IMAGE_PUBLISHER_NODE_H
#define TORCS_IMAGE_PUBLISHER_NODE_H

#include <iostream>
#include <unistd.h>
#include <sys/shm.h>
#include <stdlib.h>  
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define image_width 640
#define image_height 480

struct shared_use_st  
{  
  int written;
  unsigned char data[image_width*image_height*3];
  int pause;
  int zmq_flag; 
  int save_flag; 
};

class TORCSImgPublisherNode{
private:
  struct config_struct{
    int resize_width, resize_height;
    double loop_rate;
    int paused;
  };
  void *shm = NULL;
  struct shared_use_st *shared_;
  int shmid;

  // Setup opencv
  IplImage* screenRGB_;
  IplImage* resizeRGB_;

  std_msgs::Header header_;

  config_struct config_;

public:
  
  TORCSImgPublisherNode();

  ~TORCSImgPublisherNode();

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;

  image_transport::Publisher image_publisher_;

  void update();

  double getLoopRate();

  void getParams();

};

#endif