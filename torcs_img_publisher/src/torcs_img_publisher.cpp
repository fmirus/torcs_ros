#include <torcs_img_publisher/torcs_img_publisher.h>

// constructor
TORCSImgPublisherNode::TORCSImgPublisherNode()
 : it_(nh_)
{
  ROS_DEBUG("init torcs image publisher node");
  // create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  ROS_DEBUG("Start Memory sharing");
  shmid = shmget((key_t)1234, sizeof(struct shared_use_st), 0666|IPC_CREAT);
  if(shmid == -1)  
  {  
    fprintf(stderr, "shmget failed\n");  
    exit(EXIT_FAILURE);  
  }
  shm = shmat(shmid, 0, 0);  
  if(shm == (void*)-1)  
  {  
    fprintf(stderr, "shmat failed\n");  
    exit(EXIT_FAILURE);  
  }  
  ROS_DEBUG_STREAM("Memroy sharing started at " << shm);
  // printf("\n********** Memory sharing started, attached at blablabal %X **********\n", shm);

  getParams();

  shared_ = (struct shared_use_st*)shm; 
  shared_->written = 0;
  shared_->pause = config_.paused;
  shared_->zmq_flag = 0;  
  shared_->save_flag = 0;

  // Setup opencv
  screenRGB_ = cvCreateImage(cvSize(image_width,image_height),IPL_DEPTH_8U,3);
  resizeRGB_ = cvCreateImage(cvSize(config_.resize_width,config_.resize_height),IPL_DEPTH_8U,3);

  header_ = std_msgs::Header();
  // publisher
  image_publisher_ = it_.advertise("pov_image", 1);

  // cv::namedWindow("TORCS Image");
}

TORCSImgPublisherNode::~TORCSImgPublisherNode(){
  // cv::destroyWindow("TORCS Image");
}

void TORCSImgPublisherNode::getParams()
{
  if(!pnh_.hasParam("loop_rate"))
  {
    ROS_WARN("No parameter loop_rate on parameter server. Using default value [10.0 Hz].");
  }
  pnh_.param("loop_rate", config_.loop_rate, (double)10.0);

  if(!pnh_.hasParam("resize_width"))
  {
    ROS_WARN("No parameter resize_width on parameter server. Using default value [640 px].");
  }
  pnh_.param("resize_width", config_.resize_width, (int)640);

  if(!pnh_.hasParam("resize_height"))
  {
    ROS_WARN("No parameter resize_height on parameter server. Using default value [480 px].");
  }
  pnh_.param("resize_height", config_.resize_height, (int)480);

  if(!pnh_.hasParam("paused"))
  {
    ROS_WARN("No parameter paused on parameter server. Using default value [1].");
  }
  pnh_.param("paused", config_.paused, (int)1);
}

void TORCSImgPublisherNode::update()
{
  if (shared_->written == 1) {

    for (int h = 0; h < image_height; h++) {
      for (int w = 0; w < image_width; w++) {
       screenRGB_->imageData[(h*image_width+w)*3+2]=shared_->data[((image_height-h-1)*image_width+w)*3+0];
       screenRGB_->imageData[(h*image_width+w)*3+1]=shared_->data[((image_height-h-1)*image_width+w)*3+1];
       screenRGB_->imageData[(h*image_width+w)*3+0]=shared_->data[((image_height-h-1)*image_width+w)*3+2];
      }
    }
    
    cvResize(screenRGB_, resizeRGB_);
    
    Mat img = cvarrToMat(resizeRGB_, true);
    // Update GUI Window
    // cv::imshow("TORCS Image", img);
    // cv::waitKey(3);

    header_.stamp = ros::Time::now();
  
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header_, "bgr8", img).toImageMsg();
    
    image_publisher_.publish(msg);

    shared_->written=0;
  }
}

double TORCSImgPublisherNode::getLoopRate()
{
  return config_.loop_rate;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "torcs_img_publisher_node");
  TORCSImgPublisherNode my_publisher;
  ros::Rate loop_rate(my_publisher.getLoopRate()); // Hz
  while(my_publisher.nh_.ok()){
    my_publisher.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
