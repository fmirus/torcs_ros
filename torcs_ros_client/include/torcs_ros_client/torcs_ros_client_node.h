#ifndef TORCS_ROS_CLIENT_NODE_H
#define TORCS_ROS_CLIENT_NODE_H

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

/*** defines for UDP *****/
#define UDP_MSGLEN 1000
#define UDP_CLIENT_TIMEUOT 1000000
//#define __UDP_CLIENT_VERBOSE__
/************************/

#define PI 3.141592653589793

typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0

using namespace std;

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <torcs_msgs/TORCSCtrl.h>
#include <torcs_msgs/TORCSSensors.h>

#include <SimpleParser.h>

class TORCSROSClient{
private:
  struct config_struct{
    std::string host_name;
    int server_port;
    std::string id; 
    int max_episodes;
    int max_steps;
    std::string track_name;
    int stage;
    int num_opponents_ranges;
    int num_track_ranges;
    int num_focus_ranges;
    double loop_rate;
  };
  config_struct config_;

  sensor_msgs::LaserScan track_;
  sensor_msgs::LaserScan opponents_;
  sensor_msgs::LaserScan focus_;
  geometry_msgs::TwistStamped speed_;
  float wheelSpinVel_[4];
  float* track_array_;
  float* opponents_array_; 
  float* focus_array_;

  torcs_msgs::TORCSCtrl torcs_ctrl_;
  torcs_msgs::TORCSSensors torcs_sensors_;

  SOCKET socketDescriptor_;
  int numRead_;

  char hostName_[1000];
  unsigned int serverPort_;
  char id_[1000];
  unsigned int maxEpisodes_;
  char trackName_[1000];

  tSockAddrIn serverAddress_;
  struct hostent *hostInfo_;
  struct timeval timeVal_;
  fd_set readSet_;
  char buf_[UDP_MSGLEN];

  bool shutdownClient_;
  unsigned long curEpisode_;
  unsigned long currentStep_; 

  std_msgs::String debug_string_;

public:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber ctrl_sub_;
  ros::Publisher ctrl_pub_;
  ros::Publisher torcs_sensors_pub_;
  ros::Publisher track_pub_;
  ros::Publisher opponents_pub_;
  ros::Publisher focus_pub_;
  ros::Publisher speed_pub_;
  ros::Publisher debug_pub_;

  TORCSROSClient();
  ~TORCSROSClient();

  bool connect();

  void update();

  double getLoopRate();

  void getParams();

  void ctrlCallback(const torcs_msgs::TORCSCtrl::ConstPtr& msg);

  bool getShutdownClientStatus();

  void laserMsgToFloatArray(sensor_msgs::LaserScan scan, float* result);
  void laserMsgFromFloatArray(float* float_array, sensor_msgs::LaserScan &scan_result);

  std::string ctrlMsgToString();

  sensor_msgs::LaserScan initRangeFinder(std::string frame, double angle_min, double angle_max, double range_min, double range_max, int ranges_dim);

  std::string sensorsMsgToString();

  void sensorsMsgFromString(std::string torcs_string);

  virtual void init_angles(float *angles){
    for (int i = 0; i < 19; ++i)
      angles[i]=-90+i*10;
  };

};

#endif