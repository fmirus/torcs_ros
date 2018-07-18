#include <torcs_ros_client/torcs_ros_client_node.h>

TORCSROSClient::TORCSROSClient(){
  // create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  getParams();

  hostInfo_ = gethostbyname(config_.host_name.c_str());
    if (hostInfo_ == NULL)
    {
      ROS_ERROR_STREAM("problem interpreting host: " << config_.host_name.c_str());
      exit(1);
    }

  socketDescriptor_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (INVALID(socketDescriptor_))
  {
    ROS_ERROR("can not create socket");
    exit(1);
  }

  // Set some fields in the serverAddress structure.
  serverAddress_.sin_family = hostInfo_->h_addrtype;
  memcpy((char *) &serverAddress_.sin_addr.s_addr,
         hostInfo_->h_addr_list[0], hostInfo_->h_length);
  serverAddress_.sin_port = htons(config_.server_port);

  shutdownClient_ = false;
  curEpisode_ = 0;
  currentStep_ = 0;
  numRead_ = 0;

  torcs_ctrl_ = torcs_msgs::TORCSCtrl();
  torcs_sensors_ = torcs_msgs::TORCSSensors();
  torcs_global_ = torcs_msgs::TORCSGlobal(); //x
  speed_ = geometry_msgs::TwistStamped();

  torcs_sensors_.wheelSpinVel.resize(4, 0);

  focus_array_ = new float[config_.num_focus_ranges];
  focus_ = initRangeFinder("base_link", 0-2*PI/360, 0+2*PI/360, 0, 200, 5);
  track_array_ = new float[config_.num_track_ranges];
  track_ = initRangeFinder("base_link", -PI/2, PI/2, 0, 200, 19);
  opponents_array_ = new float[config_.num_opponents_ranges];
  opponents_ = initRangeFinder("base_link", -PI, 0.99*PI, 0, 200, 36);

  debug_string_ = std_msgs::String();

  ctrl_sub_ = nh_.subscribe("torcs_ctrl_in", 1000, &TORCSROSClient::ctrlCallback, this);
  ctrl_pub_ = nh_.advertise<torcs_msgs::TORCSCtrl>("torcs_ctrl_out", 1000);
  torcs_sensors_pub_ = nh_.advertise<torcs_msgs::TORCSSensors>("torcs_sensors_out", 1000);
  torcs_global_pub_ = nh_.advertise<torcs_msgs::TORCSGlobal>("torcs_global_out", 1000); //x
  track_pub_ = nh_.advertise<sensor_msgs::LaserScan>("torcs_track", 1000);
  opponents_pub_ = nh_.advertise<sensor_msgs::LaserScan>("torcs_opponents", 1000);
  focus_pub_ = nh_.advertise<sensor_msgs::LaserScan>("torcs_focus", 1000);
  speed_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("torcs_speed", 1000);
  debug_pub_ = nh_.advertise<std_msgs::String>("udp_string", 1000);

  bool connected = false;
  while(connected == false)
  {
    ROS_WARN_STREAM("Not connected to server yet!!");
    connected = connect();
  }
}

TORCSROSClient::~TORCSROSClient(){}

bool TORCSROSClient::connect(){
  /***********************************************************************************
  ************************* UDP client identification ********************************
  ***********************************************************************************/
  // Initialize the angles of rangefinders
  float angles[19];
  bool result = false;
  init_angles(angles);
  string initString = SimpleParser::stringify(string("init"),angles,19);
  ROS_DEBUG_STREAM("Sending id to server: " << config_.id);
  initString.insert(0,config_.id);
  ROS_DEBUG_STREAM("Sending init string to the server: " << initString);
  if (sendto(socketDescriptor_, initString.c_str(), initString.length(), 0,
             (struct sockaddr *) &serverAddress_,
             sizeof(serverAddress_)) < 0)
  {

    ROS_ERROR("cannot send data");
    CLOSE(socketDescriptor_);
    exit(1);
  }

  // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
  FD_ZERO(&readSet_);
  FD_SET(socketDescriptor_, &readSet_);
  timeVal_.tv_sec = 0;
  timeVal_.tv_usec = UDP_CLIENT_TIMEUOT;

  if (select(socketDescriptor_+1, &readSet_, NULL, NULL, &timeVal_))
  {
    // Read data sent by the solorace server
    memset(buf_, 0x0, UDP_MSGLEN);  // Zero out the buffer.
    numRead_ = recv(socketDescriptor_, buf_, UDP_MSGLEN, 0);
    if (numRead_ < 0)
    {
      ROS_ERROR("didn't get response from server...");
    }
    else
    {
      ROS_DEBUG_STREAM("Received: " << buf_);
      if (strcmp(buf_,"***identified***")==0)
      {
        ROS_INFO_STREAM("Server " << buf_);
        result = true;
      }
    }
  }

  return result;
}

double TORCSROSClient::getLoopRate()
{
  return config_.loop_rate;
}

bool TORCSROSClient::getShutdownClientStatus()
{
  return shutdownClient_;
}

void TORCSROSClient::ctrlCallback(const torcs_msgs::TORCSCtrl::ConstPtr& msg)
{
  torcs_ctrl_ = *msg;
}

std::string TORCSROSClient::ctrlMsgToString(){
  std::string result;

  result  = SimpleParser::stringify("accel", (float) torcs_ctrl_.accel);
  result += SimpleParser::stringify("brake", (float) torcs_ctrl_.brake);
  result += SimpleParser::stringify("gear",  (int) torcs_ctrl_.gear);
  result += SimpleParser::stringify("steer", (float) torcs_ctrl_.steering);
  result += SimpleParser::stringify("clutch", (float) torcs_ctrl_.clutch);
  result += SimpleParser::stringify("focus",  (float) torcs_ctrl_.focus);
  result += SimpleParser::stringify("meta", (int) torcs_ctrl_.meta);

  return result;
}

std::string TORCSROSClient::sensorsMsgToString(){

  std::string result;

  laserMsgToFloatArray(focus_, focus_array_);
  laserMsgToFloatArray(opponents_, opponents_array_);
  laserMsgToFloatArray(track_, track_array_);
  for (int i=0; i<4; i++)
  {
    wheelSpinVel_[i] = torcs_sensors_.wheelSpinVel[i];
  }

  result  = SimpleParser::stringify("angle", (float)torcs_sensors_.angle);
  result += SimpleParser::stringify("curLapTime", (float)torcs_sensors_.currentLapTime);
  result += SimpleParser::stringify("damage", (float)torcs_sensors_.damage);
  result += SimpleParser::stringify("distFromStart", (float)torcs_sensors_.distFromStart);
  result += SimpleParser::stringify("distRaced", (float)torcs_sensors_.distRaced);
  result += SimpleParser::stringify("focus", focus_array_, config_.num_focus_ranges);
  result += SimpleParser::stringify("fuel", (float)torcs_sensors_.fuel);
  result += SimpleParser::stringify("gear", (int)torcs_sensors_.gear);
  result += SimpleParser::stringify("lastLapTime", (float)torcs_sensors_.lastLapTime);
  result += SimpleParser::stringify("opponents", opponents_array_, config_.num_opponents_ranges);
  result += SimpleParser::stringify("racePos", (int)torcs_sensors_.racePos);
  result += SimpleParser::stringify("rpm", (float)torcs_sensors_.rpm);
  result += SimpleParser::stringify("speedX", (float)speed_.twist.linear.x);
  result += SimpleParser::stringify("speedY", (float)speed_.twist.linear.y);
  result += SimpleParser::stringify("speedZ", (float)speed_.twist.linear.z);
  result += SimpleParser::stringify("track", track_array_, config_.num_track_ranges);
  result += SimpleParser::stringify("trackPos", (float) torcs_sensors_.trackPos);
  result += SimpleParser::stringify("wheelSpinVel", wheelSpinVel_, 4);
  result += SimpleParser::stringify("z", (float)torcs_sensors_.z);
  result += SimpleParser::stringify("x", (float)torcs_global_.x); 
  result += SimpleParser::stringify("y", (float)torcs_global_.y); 
  result += SimpleParser::stringify("roll", (float)torcs_global_.roll);
  result += SimpleParser::stringify("pitch", (float)torcs_global_.pitch); 
  result += SimpleParser::stringify("yaw", (float)torcs_global_.yaw); 
  result += SimpleParser::stringify("speedGlobalX", (float)torcs_global_.speedGX); 
  result += SimpleParser::stringify("speedGlobalY", (float)torcs_global_.speedGY); 



  return result;
}
void TORCSROSClient::sensorsMsgFromString(std::string torcs_string){
  torcs_sensors_.header.stamp = ros::Time::now();
  torcs_global_.header.stamp = ros::Time::now();

  float angle;
  SimpleParser::parse(torcs_string, "angle", angle);
  torcs_sensors_.angle = angle;

  float curLapTime;
  SimpleParser::parse(torcs_string, "curLapTime", curLapTime);
  torcs_sensors_.currentLapTime = curLapTime;

  float damage;
  SimpleParser::parse(torcs_string, "damage", damage);
  torcs_sensors_.damage = damage;

  float distFromStart;
  SimpleParser::parse(torcs_string, "distFromStart", distFromStart);
  torcs_sensors_.distFromStart = distFromStart;

  float distRaced;
  SimpleParser::parse(torcs_string, "distRaced", distRaced);
  torcs_sensors_.distRaced = distRaced;

  float fuel;
  SimpleParser::parse(torcs_string, "fuel", fuel);
  torcs_sensors_.fuel = fuel;

  int gear;
  SimpleParser::parse(torcs_string, "gear", gear);
  torcs_sensors_.gear = gear;

  float lastLapTime;
  SimpleParser::parse(torcs_string, "lastLapTime", lastLapTime);
  torcs_sensors_.lastLapTime = lastLapTime;

  int racePos;
  SimpleParser::parse(torcs_string, "racePos", racePos);
  torcs_sensors_.racePos = racePos;

  float rpm;
  SimpleParser::parse(torcs_string, "rpm", rpm);
  torcs_sensors_.rpm = rpm;

  float trackPos;
  SimpleParser::parse(torcs_string, "trackPos", trackPos);
  torcs_sensors_.trackPos = trackPos;

  SimpleParser::parse(torcs_string, "wheelSpinVel", wheelSpinVel_, 4);
  for (int i=0; i<4; i++)
  {
    torcs_sensors_.wheelSpinVel[i] = wheelSpinVel_[i];
  }

  float z;
  SimpleParser::parse(torcs_string, "z", z);
  torcs_sensors_.z = z; //depreceated
  torcs_global_.z = z;
  
  float x;
  SimpleParser::parse(torcs_string, "x", x);
  torcs_global_.x = x;

  float y;
  SimpleParser::parse(torcs_string, "y", y);
  torcs_global_.y = y;

  float roll;
  SimpleParser::parse(torcs_string, "roll", roll);
  torcs_global_.roll = roll;

  float pitch;
  SimpleParser::parse(torcs_string, "pitch", pitch);
  torcs_global_.pitch = pitch;

  float yaw;
  SimpleParser::parse(torcs_string, "yaw", yaw);
  torcs_global_.yaw = yaw;

  float speedGX;
  SimpleParser::parse(torcs_string, "speedGlobalX", speedGX);
  torcs_global_.speedGX = speedGX;

  float speedGY;
  SimpleParser::parse(torcs_string, "speedGlobalY", speedGY);
  torcs_global_.speedGY = speedGY;

  SimpleParser::parse(torcs_string, "focus", focus_array_, config_.num_focus_ranges);
  laserMsgFromFloatArray(focus_array_, focus_);

  SimpleParser::parse(torcs_string, "opponents", opponents_array_, config_.num_opponents_ranges);
  laserMsgFromFloatArray(opponents_array_, opponents_);

  SimpleParser::parse(torcs_string, "track", track_array_, config_.num_track_ranges);
  laserMsgFromFloatArray(track_array_, track_);

  speed_.header.stamp = ros::Time::now();
  float speedX, speedY, speedZ;
  SimpleParser::parse(torcs_string, "speedX", speedX);
  SimpleParser::parse(torcs_string, "speedY", speedY);
  SimpleParser::parse(torcs_string, "speedZ", speedZ);
  speed_.twist.linear.x = speedX;
  speed_.twist.linear.y = speedY;
  speed_.twist.linear.z = speedZ;

}

sensor_msgs::LaserScan TORCSROSClient::initRangeFinder(std::string frame, double angle_min, double angle_max,
                                                       double range_min, double range_max, int ranges_dim)
{
  sensor_msgs::LaserScan result = sensor_msgs::LaserScan();
  result.header = std_msgs::Header();

  result.header.frame_id = frame;
  result.header.stamp = ros::Time::now();

  result.angle_min = angle_min;
  result.angle_max = angle_max;
  result.angle_increment = (angle_max - angle_min)/ranges_dim;
  result.range_min = range_min;
  result.range_max = range_max;
  result.ranges.resize(ranges_dim, 0);

  return result;
}
void TORCSROSClient::laserMsgToFloatArray(sensor_msgs::LaserScan scan, float* result)
{
  int size = scan.ranges.size();
  for (int i=0; i<size; i++)
  {
    result[i] = scan.ranges[size-i];
  }
}

void TORCSROSClient::laserMsgFromFloatArray(float* float_array, sensor_msgs::LaserScan &scan_result)
{
  int size = scan_result.ranges.size();
  scan_result.header.stamp = ros::Time::now();
  for (int i=0; i<size; i++)
  {
     scan_result.ranges[size-i] = float_array[i];
  }
}

void TORCSROSClient::update()
{
  ROS_DEBUG("Start update");
  // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
  FD_ZERO(&readSet_);
  FD_SET(socketDescriptor_, &readSet_);
  timeVal_.tv_sec = 0;
  timeVal_.tv_usec = UDP_CLIENT_TIMEUOT;

  if (select(socketDescriptor_+1, &readSet_, NULL, NULL, &timeVal_))
  {
    // Read data sent by the solorace server
    // receive TORCS information from UDP socket
    memset(buf_, 0x0, UDP_MSGLEN);  // Zero out the buffer.
    numRead_ = recv(socketDescriptor_, buf_, UDP_MSGLEN, 0);
    if (numRead_ < 0)
    {
      ROS_ERROR("didn't get response from server...");
      CLOSE(socketDescriptor_);
      exit(1);
    }

    ROS_DEBUG_STREAM("Received: " << buf_);

    if (strcmp(buf_,"***shutdown***")==0)
    {
      // d.onShutdown();
      shutdownClient_ = true;
      ROS_INFO_STREAM("Client Shutdown");
    }

    if (strcmp(buf_,"***restart***")==0)
    {
      // d.onRestart();
      ROS_INFO_STREAM("Client Restart");
    }
    /**************************************************
     * Compute The Action to send to the solorace sever
     **************************************************/

    if ( (++currentStep_) != config_.max_steps)
    {
      // string action = d.drive(string(buf));
      // store sensor and ctrl data in ROS messages
      std::string udp_str(buf_);
      debug_string_.data = udp_str;
      debug_pub_.publish(debug_string_);
      sensorsMsgFromString((std::string) buf_);
      // now publish the created ROS messages
      ctrl_pub_.publish(torcs_ctrl_);
      torcs_sensors_pub_.publish(torcs_sensors_);
      torcs_global_pub_.publish(torcs_global_); //x
      track_pub_.publish(track_);
      opponents_pub_.publish(opponents_);
      focus_pub_.publish(focus_);
      speed_pub_.publish(speed_);
      // create string from subscribed ctrl msg
      std::string action = ctrlMsgToString();
      memset(buf_, 0x0, UDP_MSGLEN);
      sprintf(buf_,"%s",action.c_str());
    }
    else
    {
      sprintf (buf_, "(meta 1)");
    }

    // send action string back to TORCS
    if (sendto(socketDescriptor_, buf_, strlen(buf_)+1, 0,
               (struct sockaddr *) &serverAddress_,
               sizeof(serverAddress_)) < 0)
    {
      ROS_ERROR("cannot send data");
      CLOSE(socketDescriptor_);
      exit(1);
    }
    else
    {
      ROS_DEBUG_STREAM("Sending: " << buf_);
    }
  }
  else
  {
    ROS_WARN_STREAM("** Server did not respond in 1 second");
  }
  ROS_DEBUG("End update");
}

void TORCSROSClient::getParams()
{
  if(!pnh_.hasParam("host_name"))
  {
    ROS_WARN("No parameter host_name on parameter server. Using default value ['localhost'].");
  }
  pnh_.param("host_name", config_.host_name, (std::string)"localhost");

  if(!pnh_.hasParam("server_port"))
  {
    ROS_WARN("No parameter server_port on parameter server. Using default value [3001].");
  }
  pnh_.param("server_port", config_.server_port, (int)3001);

  if(!pnh_.hasParam("id"))
  {
    ROS_WARN("No parameter id on parameter server. Using default value ['SCR'].");
  }
  pnh_.param("id", config_.id, (std::string)"SCR");

  if(!pnh_.hasParam("max_episodes"))
  {
    ROS_WARN("No parameter max_episodes on parameter server. Using default value [0].");
  }
  pnh_.param("max_episodes", config_.max_episodes, (int)0);

  if(!pnh_.hasParam("max_steps"))
  {
    ROS_WARN("No parameter max_steps on parameter server. Using default value [0].");
  }
  pnh_.param("max_steps", config_.max_steps, (int)0);

  if(!pnh_.hasParam("track_name"))
  {
    ROS_WARN("No parameter track_name on parameter server. Using default value ['unknown'].");
  }
  pnh_.param("track_name", config_.track_name, (std::string)"unknown");

  if(!pnh_.hasParam("stage"))
  {
    ROS_WARN("No parameter stage on parameter server. Using default value [3 = UNKNOWN].");
  }
  pnh_.param("stage", config_.stage, (int)3);

  if(!pnh_.hasParam("num_opponents_ranges"))
  {
    ROS_WARN("No parameter num_opponents_ranges on parameter server. Using default value [36].");
  }
  pnh_.param("num_opponents_ranges", config_.num_opponents_ranges, (int)36);

  if(!pnh_.hasParam("num_track_ranges"))
  {
    ROS_WARN("No parameter num_track_ranges on parameter server. Using default value [19].");
  }
  pnh_.param("num_track_ranges", config_.num_track_ranges, (int)19);

  if(!pnh_.hasParam("num_focus_ranges"))
  {
    ROS_WARN("No parameter num_focus_ranges on parameter server. Using default value [5].");
  }
  pnh_.param("num_focus_ranges", config_.num_focus_ranges, (int)5);

  if(!pnh_.hasParam("loop_rate"))
  {
    ROS_WARN("No parameter loop_rate on parameter server. Using default value [100.0 Hz].");
  }
  pnh_.param("loop_rate", config_.loop_rate, (double)100.0);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torcs_ros_client_node");
  TORCSROSClient torcs_client;

  ros::Rate loop_rate(torcs_client.getLoopRate()); // Hz

  while(torcs_client.nh_.ok() && torcs_client.getShutdownClientStatus() == false){
    torcs_client.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
