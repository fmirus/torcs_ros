#ifndef TORCS_ROS_DRIVE_CTRL_NODE_H
#define TORCS_ROS_DRIVE_CTRL_NODE_H

#include <iostream>
#include <sstream>
#include <cstring>
#include <cassert>

#define PI 3.141592653589793
#define SIN5 0.08716
#define COS5 0.99619

using namespace std;

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <torcs_msgs/TORCSCtrl.h>
#include <torcs_msgs/TORCSSensors.h>

class TORCSROSDriveCtrl{
private:
  struct config_struct{
    /* Gear Changing Constants*/
    // RPM values to change gear 
    std::vector<int> gearUp;
    std::vector<int> gearDown;
    /* Stuck constants*/
    
    // How many time steps the controller wait before recovering from a stuck position
    int stuckTime;
    // When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
    double stuckAngle;

    /* Steering constants*/
    
    // Angle associated to a full steer command
    double steerLock; 
    // Min speed to reduce steering command 
    double steerSensitivityOffset;
    // Coefficient to reduce steering command at high speed (to avoid loosing the control)
    double wheelSensitivityCoeff;
    
    /* Accel and Brake Constants*/
    
    // max speed allowed
    double maxSpeed;
    // Min distance from track border to drive at  max speed
    double maxSpeedDist;
    
    /* ABS Filter Constants */
    
    // Radius of the 4 wheels of the car
    std::vector<double> wheelRadius;
    // min slip to prevent ABS
    double absSlip;           
    // range to normalize the ABS effect on the brake
    double absRange;
    // min speed to activate ABS
    double absMinSpeed;

    /* Clutch constants */
    double clutchMax;
    double clutchDelta;
    double clutchRange;
    double clutchDeltaTime;
    double clutchDeltaRaced;
    double clutchDec;
    double clutchMaxModifier;
    double clutchMaxTime;

    int stage;
    
    double loop_rate;
    
  };

  config_struct config_;

  sensor_msgs::LaserScan track_;
  sensor_msgs::LaserScan opponents_;
  sensor_msgs::LaserScan focus_;
  geometry_msgs::TwistStamped speed_;

  torcs_msgs::TORCSCtrl torcs_ctrl_out_;
  torcs_msgs::TORCSCtrl torcs_ctrl_in_;
  torcs_msgs::TORCSSensors torcs_sensors_;
  
public:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber ctrl_sub_;
  ros::Publisher ctrl_pub_;
  ros::Subscriber torcs_sensors_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber opponents_sub_;
  ros::Subscriber focus_sub_;
  ros::Subscriber speed_sub_;

  TORCSROSDriveCtrl();
  ~TORCSROSDriveCtrl();

  void getParams();

  // counter of stuck steps
  int stuck;
  
  // current clutch
  double clutch;

  // Solves the gear changing subproblems
  int getGear();

  // Solves the steering subproblems
  double getSteer();
  
  // Solves the acceleration subproblems
  double getAccel();
  
  // Apply an ABS filter to brake command
  double filterABS(double brake);

  // Solves the clucthing subproblems
  void clutching(double &clutch);

  void drive();

  void ctrlCallback(const torcs_msgs::TORCSCtrl::ConstPtr& msg);
  void torcsSensorsCallback(const torcs_msgs::TORCSSensors::ConstPtr& msg);
  void laserTrackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void laserFocusCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void laserOpponentsCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void twistSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  double getLoopRate();
};

#endif