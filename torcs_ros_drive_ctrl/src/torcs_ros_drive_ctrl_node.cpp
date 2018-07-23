#include <torcs_ros_drive_ctrl/torcs_ros_drive_ctrl_node.h>

TORCSROSDriveCtrl::TORCSROSDriveCtrl()
{
  ROS_DEBUG_STREAM("Constructor");
  // create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  getParams();

  torcs_ctrl_in_ = torcs_msgs::TORCSCtrl();
  torcs_ctrl_out_ = torcs_msgs::TORCSCtrl();
  torcs_sensors_ = torcs_msgs::TORCSSensors();

  torcs_sensors_.wheelSpinVel.resize(4, 0);


  focus_ = sensor_msgs::LaserScan();
  track_ = sensor_msgs::LaserScan();
  opponents_ = sensor_msgs::LaserScan();
  speed_= geometry_msgs::TwistStamped();


  ctrl_sub_ = nh_.subscribe("torcs_ctrl_in", 1000, &TORCSROSDriveCtrl::ctrlCallback, this);
  ctrl_pub_ = nh_.advertise<torcs_msgs::TORCSCtrl>("torcs_ctrl_out", 1000);
  torcs_sensors_sub_ = nh_.subscribe("torcs_sensors_in", 1000, &TORCSROSDriveCtrl::torcsSensorsCallback, this);
  track_sub_ = nh_.subscribe("torcs_track", 1000, &TORCSROSDriveCtrl::laserTrackCallback, this);
  opponents_sub_ = nh_.subscribe("torcs_opponents", 1000, &TORCSROSDriveCtrl::laserOpponentsCallback, this);
  focus_sub_ = nh_.subscribe("torcs_focus", 1000, &TORCSROSDriveCtrl::laserFocusCallback, this);
  speed_sub_ = nh_.subscribe("torcs_speed", 1000, &TORCSROSDriveCtrl::twistSpeedCallback, this);

  ROS_DEBUG_STREAM("Constructor end");
}

TORCSROSDriveCtrl::~TORCSROSDriveCtrl(){}

void TORCSROSDriveCtrl::drive()
{
  ROS_DEBUG_STREAM("drive");
  // check if car is currently stuck
  if ( fabs(torcs_sensors_.angle) > config_.stuckAngle )
  {
  // update stuck counter
      stuck++;
  }
  else
  {
    // if not stuck reset stuck counter
      stuck = 0;
  }

  // after car is stuck for a while apply recovering policy
  if (stuck > config_.stuckTime)
  {
    ROS_DEBUG_STREAM("car is stuck");
    /* set gear and sterring command assuming car is 
     * pointing in a direction out of track */
    
    // to bring car parallel to track axis
    double steer = - torcs_sensors_.angle / config_.steerLock; 
    int gear=-1; // gear R
    
    // if car is pointing in the correct direction revert gear and steer  
    if (torcs_sensors_.angle*torcs_sensors_.trackPos>0)
    {
        gear = 1;
        steer = -steer;
    }

    // Calculate clutching
    clutching(clutch);

    // build a torcs_ctrl message and publish it
    torcs_ctrl_out_.header.stamp = ros::Time::now();
    torcs_ctrl_out_.accel = 1.0;
    torcs_ctrl_out_.brake = 0.0;
    torcs_ctrl_out_.gear = gear;
    torcs_ctrl_out_.steering = steer;
    torcs_ctrl_out_.clutch = clutch;

    ctrl_pub_.publish(torcs_ctrl_out_);
  }
  else // car is not stuck
  {
    ROS_DEBUG_STREAM("car is not stuck");
    // compute accel/brake command
    double accel_and_brake;
    if(!track_.ranges.empty())
    {
      accel_and_brake = getAccel();
    }
    else
    {
      accel_and_brake = 0; 
    }
    // compute gear 
    int gear = getGear();
    // compute steering
    double steer = getSteer();
    
    // normalize steering
    if (steer < -1)
        steer = -1;
    if (steer > 1)
        steer = 1;
    
    // set accel and brake from the joint accel/brake command 
    double accel,brake;
    if (accel_and_brake>0)
    {
        accel = accel_and_brake;
        brake = 0;
    }
    else
    {
        accel = 0;
        // apply ABS to brake
        brake = filterABS(-accel_and_brake);
    }

    // Calculate clutching
    clutching(clutch);

    // build a torcs_ctrl message and publish it
    torcs_ctrl_out_.header.stamp = ros::Time::now();
    torcs_ctrl_out_.accel = accel;
    torcs_ctrl_out_.brake = brake;
    torcs_ctrl_out_.gear = gear;
    torcs_ctrl_out_.steering = steer;
    torcs_ctrl_out_.clutch = clutch;

    ctrl_pub_.publish(torcs_ctrl_out_);
  }
}

// Solves the gear changing subproblems
int TORCSROSDriveCtrl::getGear()
{
  int gear = torcs_ctrl_in_.gear;
  int rpm  = torcs_sensors_.rpm;

  // if gear is 0 (N) or -1 (R) just return 1 
  if (gear<1)
    return 1;
  // check if the RPM value of car is greater than the one suggested 
  // to shift up the gear from the current one     
  if (gear <6 && rpm >= config_.gearUp[gear-1])
    return gear + 1;
  else
  // check if the RPM value of car is lower than the one suggested 
  // to shift down the gear from the current one
    if (gear > 1 && rpm <= config_.gearDown[gear-1])
      return gear - 1;
    else // otherwhise keep current gear
      return gear;
}

// Solves the steering subproblems
double TORCSROSDriveCtrl::getSteer()
{
    // steering angle is compute by correcting the actual car angle w.r.t. to track 
    // axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    double targetAngle=(torcs_sensors_.angle - torcs_sensors_.trackPos*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (speed_.twist.linear.x > config_.steerSensitivityOffset)
      return targetAngle/(config_.steerLock*(speed_.twist.linear.x-config_.steerSensitivityOffset)*config_.wheelSensitivityCoeff);
    else
      return (targetAngle)/config_.steerLock;
}

// Solves the acceleration subproblems
double TORCSROSDriveCtrl::getAccel()
{
  // checks if car is out of track
  if (torcs_sensors_.trackPos < 1 && torcs_sensors_.trackPos > -1)
  {
    // reading of sensor at +5 degree w.r.t. car axis
    double rxSensor=track_.ranges[9];
    // double rxSensor=cs.getTrack(10);
    // reading of sensor parallel to car axis
    // double cSensor=cs.getTrack(9);
    double cSensor=track_.ranges[10];
    // reading of sensor at -5 degree w.r.t. car axis
    // double sxSensor=cs.getTrack(8);
    double sxSensor=track_.ranges[11];

    double targetSpeed;

    // track is straight and enough far from a turn so goes to max speed
    if (cSensor>config_.maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
      targetSpeed = config_.maxSpeed;
    else
    {
      // approaching a turn on right
      if(rxSensor>sxSensor)
      {
        // computing approximately the "angle" of turn
        double h = cSensor*SIN5;
        double b = rxSensor - cSensor*COS5;
        double sinAngle = b*b/(h*h+b*b);
        // estimate the target speed depending on turn and on how close it is
        targetSpeed = config_.maxSpeed*(cSensor*sinAngle/config_.maxSpeedDist);
        }
        // approaching a turn on left
        else
        {
            // computing approximately the "angle" of turn
          double h = cSensor*SIN5;
          double b = sxSensor - cSensor*COS5;
          double sinAngle = b*b/(h*h+b*b);
          // estimate the target speed depending on turn and on how close it is
          targetSpeed = config_.maxSpeed*(cSensor*sinAngle/config_.maxSpeedDist);
        }

    }

    // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
    return 2/(1+exp(speed_.twist.linear.x - targetSpeed)) - 1;
  }
  else
    return 0.3; // when out of track returns a moderate acceleration command

}

// Apply an ABS filter to brake command
double TORCSROSDriveCtrl::filterABS(double brake)
{
  // convert speed to m/s
  double speed = speed_.twist.linear.x / 3.6;
  // when spedd lower than min speed for abs do nothing
  if (speed < config_.absMinSpeed)
    return brake;
  
  // compute the speed of wheels in m/s
  double slip = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    slip += torcs_sensors_.wheelSpinVel[i] * config_.wheelRadius[i];
  }
  // slip is the difference between actual speed of car and average speed of wheels
  slip = speed - slip/4.0f;
  // when slip too high applu ABS
  if (slip > config_.absSlip)
  {
    brake = brake - (slip - config_.absSlip)/config_.absRange;
  }
  
  // check brake is not negative, otherwise set it to zero
  if (brake<0)
    return 0;
  else
    return brake;
}

// Solves the clucthing subproblems
void TORCSROSDriveCtrl::clutching(double &clutch)
{
  double maxClutch = config_.clutchMax;

  // Check if the current situation is the race start
  if (torcs_sensors_.currentLapTime < config_.clutchDeltaTime  && config_.stage==2 && torcs_sensors_.distRaced < config_.clutchDeltaRaced)
    clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0)
  {
    double delta = config_.clutchDelta;
    if (torcs_ctrl_in_.gear < 2)
    {
      // Apply a stronger clutch output when the gear is one and the race is just started
      delta /= 2;
      maxClutch *= config_.clutchMaxModifier;
      if (torcs_sensors_.currentLapTime < config_.clutchMaxTime)
        clutch = maxClutch;
    }

    // check clutch is not bigger than maximum values
    clutch = min(maxClutch,double(clutch));

    // if clutch is not at max value decrease it quite quickly
    if (clutch!=maxClutch)
    {
      clutch -= delta;
      clutch = max(0.0,double(clutch));
    }
    // if clutch is at max value decrease it very slowly
    else
      clutch -= config_.clutchDec;
  }
}

void TORCSROSDriveCtrl::ctrlCallback(const torcs_msgs::TORCSCtrl::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe ctrl");
  torcs_ctrl_in_ = *msg;
}

void TORCSROSDriveCtrl::torcsSensorsCallback(const torcs_msgs::TORCSSensors::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe sensors");
  torcs_sensors_ = *msg; 
}

void TORCSROSDriveCtrl::laserTrackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe track");
  track_ = *msg;
}

void TORCSROSDriveCtrl::laserFocusCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe focus");
  focus_ = *msg;
}

void TORCSROSDriveCtrl::laserOpponentsCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe opponents");
  opponents_ = *msg;
}

void TORCSROSDriveCtrl::twistSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("subscribe speed");
  speed_ = *msg;
}

void TORCSROSDriveCtrl::getParams()
{
  if(!pnh_.hasParam("gearUp"))
  {
    ROS_WARN("No parameter gearUp on parameter server. Using default value [{5000,6000,6000,6500,7000,0}].");
    int myints[] = {5000,6000,6000,6500,7000,0};
    std::vector<int> gearUp_default(myints, myints + sizeof(myints) / sizeof(int) );
    pnh_.setParam("gearUp", gearUp_default);
    config_.gearUp = gearUp_default;
  }
  else
  {
    pnh_.getParam("gearUp", config_.gearUp);
  }

  if(!pnh_.hasParam("gearDown"))
  {
    ROS_WARN("No parameter gearDown on parameter server. Using default value [{0,2500,3000,3000,3500,3500}].");
    int myints[] = {0,2500,3000,3000,3500,3500};
    std::vector<int> gearDown_default(myints, myints + sizeof(myints) / sizeof(int) );
    pnh_.setParam("gearDown", gearDown_default);
    config_.gearDown = gearDown_default;
  }
  else
  {
    pnh_.getParam("gearDown", config_.gearDown);
  }

  /* Stuck constants*/
  if(!pnh_.hasParam("stuckTime"))
  {
    ROS_WARN("No parameter stuckTime on parameter server. Using default value [25.0 s].");
  }
  pnh_.param("stuckTime", config_.stuckTime, (int)25);

  if(!pnh_.hasParam("stuckAngle"))
  {
    ROS_WARN("No parameter stuckAngle on parameter server. Using default value [PI/6].");
  }
  pnh_.param("stuckAngle", config_.stuckAngle, (double)0.523598775);

  /* Accel and Brake Constants*/
  
  if(!pnh_.hasParam("maxSpeedDist"))
  {
    ROS_WARN("No parameter maxSpeedDist on parameter server. Using default value [70].");
  }
  pnh_.param("maxSpeedDist", config_.maxSpeedDist, (double)70.0);

  if(!pnh_.hasParam("maxSpeed"))
  {
    ROS_WARN("No parameter maxSpeed on parameter server. Using default value [150].");
  }
  pnh_.param("maxSpeed", config_.maxSpeed, (double)150.0);

  /* Steering constants*/

  if(!pnh_.hasParam("steerLock"))
  {
    ROS_WARN("No parameter steerLock on parameter server. Using default value [0.366519].");
  }
  pnh_.param("steerLock", config_.steerLock, (double)0.366519);

  if(!pnh_.hasParam("steerSensitivityOffset"))
  {
    ROS_WARN("No parameter steerSensitivityOffset on parameter server. Using default value [80.0].");
  }
  pnh_.param("steerSensitivityOffset", config_.steerSensitivityOffset, (double)80.0);

  if(!pnh_.hasParam("wheelSensitivityCoeff"))
  {
    ROS_WARN("No parameter wheelSensitivityCoeff on parameter server. Using default value [1.0].");
  }
  pnh_.param("wheelSensitivityCoeff", config_.wheelSensitivityCoeff, (double)1.0);

  /* ABS Filter Constants */
  if(!pnh_.hasParam("wheelRadius"))
  {
    ROS_WARN("No parameter wheelRadius on parameter server. Using default value [{0.3306,0.3306,0.3276,0.3276}].");
    double mydoubles[] = {0.3306, 0.3306, 0.3276, 0.3276};
    std::vector<double> wheelRadius_default(mydoubles, mydoubles + sizeof(mydoubles) / sizeof(double) );
    pnh_.setParam("wheelRadius", wheelRadius_default);
    config_.wheelRadius = wheelRadius_default;
  }
  else
  {
    pnh_.getParam("wheelRadius", config_.wheelRadius);
  }

  if(!pnh_.hasParam("absSlip"))
  {
    ROS_WARN("No parameter absSlip on parameter server. Using default value [2.0].");
  }
  pnh_.param("absSlip", config_.absSlip, (double)2.0);

  if(!pnh_.hasParam("absRange"))
  {
    ROS_WARN("No parameter absRange on parameter server. Using default value [3.0].");
  }
  pnh_.param("absRange", config_.absRange, (double)3.0);

  if(!pnh_.hasParam("absMinSpeed"))
  {
    ROS_WARN("No parameter absMinSpeed on parameter server. Using default value [3.0].");
  }
  pnh_.param("absMinSpeed", config_.absMinSpeed, (double)3.0);

  /* Clutch constants */
  if(!pnh_.hasParam("clutchMax"))
  {
    ROS_WARN("No parameter clutchMax on parameter server. Using default value [0.5].");
  }
  pnh_.param("clutchMax", config_.clutchMax, (double)0.5);

  if(!pnh_.hasParam("clutchDelta"))
  {
    ROS_WARN("No parameter clutchDelta on parameter server. Using default value [0.05].");
  }
  pnh_.param("clutchDelta", config_.clutchDelta, (double)0.05);

  if(!pnh_.hasParam("clutchRange"))
  {
    ROS_WARN("No parameter clutchRange on parameter server. Using default value [0.85].");
  }
  pnh_.param("clutchRange", config_.clutchRange, (double)0.85);

  if(!pnh_.hasParam("clutchDeltaTime"))
  {
    ROS_WARN("No parameter clutchDeltaTime on parameter server. Using default value [0.02].");
  }
  pnh_.param("clutchDeltaTime", config_.clutchDeltaTime, (double)0.02);

  if(!pnh_.hasParam("clutchDeltaRaced"))
  {
    ROS_WARN("No parameter clutchDeltaRaced on parameter server. Using default value [10.0].");
  }
  pnh_.param("clutchDeltaRaced", config_.clutchDeltaRaced, (double)10.0);

  if(!pnh_.hasParam("clutchDec"))
  {
    ROS_WARN("No parameter clutchDec on parameter server. Using default value [0.01].");
  }
  pnh_.param("clutchDec", config_.clutchDec, (double)0.01);

  if(!pnh_.hasParam("clutchMaxModifier"))
  {
    ROS_WARN("No parameter clutchMaxModifier on parameter server. Using default value [1.3].");
  }
  pnh_.param("clutchMaxModifier", config_.clutchMaxModifier, (double)1.3);

  if(!pnh_.hasParam("clutchMaxTime"))
  {
    ROS_WARN("No parameter clutchMaxTime on parameter server. Using default value [1.5].");
  }
  pnh_.param("clutchMaxTime", config_.clutchMaxTime, (double)1.5);

  if(!pnh_.hasParam("stage"))
  {
    ROS_WARN("No parameter stage on parameter server. Using default value [3 = UNKNOWN].");
  }
  pnh_.param("stage", config_.stage, (int)3);

  if(!pnh_.hasParam("loop_rate"))
  {
    ROS_WARN("No parameter loop_rate on parameter server. Using default value [100.0 Hz].");
  }
  pnh_.param("loop_rate", config_.loop_rate, (double)100.0);

}

double TORCSROSDriveCtrl::getLoopRate()
{
  return config_.loop_rate;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torcs_ros_drive_ctrl_node");
  TORCSROSDriveCtrl torcs_ros_drive_ctrl;

  ros::Rate loop_rate(50); // Hz

  while(torcs_ros_drive_ctrl.nh_.ok()){
    torcs_ros_drive_ctrl.drive();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
