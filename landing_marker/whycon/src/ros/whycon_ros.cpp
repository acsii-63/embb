#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"
#include <vector>
#include <Eigen/Dense>


whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
	transformation_loaded = true;
	similarity.setIdentity();
  current_find = 0;
  if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

  n.param("name", frame_id, std::string("whycon"));
	n.param("world_frame", world_frame_id, std::string("camera_color_optical_frame"));
  n.param("max_attempts", max_attempts, 1);
  n.param("max_refine", max_refine, 1);

	n.getParam("outer_diameter", parameters.outer_diameter);
  n.getParam("max_distance", max_distance);
	n.getParam("inner_diameter", parameters.inner_diameter);
	n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
	n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
	n.getParam("roundness_tolerance", parameters.roundness_tolerance);
	n.getParam("circularity_tolerance", parameters.circularity_tolerance);
	n.getParam("max_size", parameters.max_size);
	n.getParam("min_size", parameters.min_size);
  n.getParam("offset_x", offsetx_);
	n.getParam("offset_y", offsety_);
	n.getParam("ratio_tolerance", parameters.ratio_tolerance);
	n.getParam("max_eccentricity", parameters.max_eccentricity);

	transform_broadcaster = boost::make_shared<tf::TransformBroadcaster>();

  /* initialize ros */
  int input_queue_size = 1;
  n.param("input_queue_size", input_queue_size, input_queue_size);
  cam_sub = it.subscribeCamera("/camera/image_raw", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  should_shutdown = false;
  stop = true;
  image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
  poses_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/landing/marker_pose/whycon", 1);
  context_pub = n.advertise<sensor_msgs::Image>("context", 1);
	projection_pub = n.advertise<whycon::Projection>("projection", 1);
  terminateServer = n.advertiseService("/landing/marker_terminate/whycon",&WhyConROS::terminateCallback,this);
  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{ 
  if(should_shutdown)
    ros::shutdown();
  if(stop)
    return;
  camera_model.fromCameraInfo(info_msg);
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
  const cv::Mat& image = cv_ptr->image;

  cv::Mat dis_coeff_temp = cv::Mat(camera_model.distortionCoeffs());

  if (dis_coeff_temp.empty())
  {
      dis_coeff_temp = cv::Mat::zeros(1, 5, CV_64F);
  }

  if (!system_check()){
    systemList.clear();
    ROS_INFO_STREAM("system clear");
    for(int t = 1; t <= targets ; t++){
      systemList.push_back(boost::make_shared<whycon::LocalizationSystem>(t, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), dis_coeff_temp, parameters));
    }
  }
  current_find = std::min(targets,current_find+1);
  bool finnish_search = false;
  while (!finnish_search)
  {
    bool found = systemList[current_find-1]->localize(image, should_reset, max_attempts, max_refine);
    if(found){
      finnish_search = true;
    }
    else{
      current_find --;
      if(current_find == 0) {
        finnish_search = true;
        break;
      }
    }
  }
  if(current_find == 0) 
  {
    is_tracking = false;
  }
  else {
    is_tracking = true;
    publish_results(image_msg->header, cv_ptr);
    should_reset = false;

    if (image_pub.getNumSubscribers() != 0)
    image_pub.publish(cv_ptr);
    if (context_pub.getNumSubscribers() != 0) {
      cv_bridge::CvImage cv_img_context;
      cv_img_context.encoding = cv_ptr->encoding;
      cv_img_context.header.stamp = cv_ptr->header.stamp;
      system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
      context_pub.publish(cv_img_context.toImageMsg());
    }
  }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  should_reset = true;
  return true;
}

bool whycon::WhyConROS::terminateCallback(marker_planner::terminate::Request& request, marker_planner::terminate::Response& response)
{
  response.success = 1;
      switch(request.action){
    
        case request.start :{
            ROS_INFO_STREAM("WHYCON_ACTIVATE");
            stop =false;
            break;
        }
        case request.stop :{
            ROS_INFO_STREAM("WHYCON_STOP");
            stop = true;
            break;
        }
        case request.terminate :{
            ROS_INFO_STREAM("WHYCON_TERMINATE");
            should_shutdown = true;
            break;
        }
      }
      
      return true;
}
bool whycon::WhyConROS::system_check(){
  if(systemList.size()<targets) return false;
  else{
    for (auto s : systemList){
      if(!s) return false;
    }
  }
  return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
  geometry_msgs::PoseWithCovariance PWC;
  int tartget_found = current_find;
  PWC.covariance[0]= 0;
  std::vector<Eigen::Vector3d> marker_pose;

  for (int i = 0; i < tartget_found ; i++) {
    const whycon::CircleDetector::Circle& circle = systemList[tartget_found-1]->get_circle(i);
    whycon::LocalizationSystem::Pose pose = systemList[tartget_found-1]->get_pose(circle);
      marker_pose.push_back(Eigen::Vector3d(pose.pos(0),pose.pos(1),pose.pos(2)));
  }
  Eigen::Vector3d mean(0,0,0);
  for(auto p : marker_pose){
    mean+= p;
  }
  mean/= tartget_found;
  for(auto p : marker_pose){
    if((p-mean).norm()>max_distance*1.5){
      ROS_INFO_STREAM("high deviation");
      return;
    }
  }
  
  
  if(tartget_found!=3 && tartget_found!=0){
  PWC.covariance[0]=0.2+tartget_found/5.0;
  PWC.pose.position.x = mean(0);
  PWC.pose.position.y = mean(1);
  PWC.pose.position.z = mean(2);
  poses_pub.publish(PWC);
  } 
  else if(tartget_found==3){
    Eigen::Vector3d a = marker_pose[0],b = marker_pose[1],c = marker_pose[2] ;// are the 3 pts of the tri
    Eigen::Vector3d ac = c - a ;
    Eigen::Vector3d ab = b - a ;
    Eigen::Vector3d abXac = ab.cross(ac) ;

    // this is the vector from a TO the circumsphere center
    Eigen::Vector3d toCircumsphereCenter = (abXac.cross(ab)*ac.squaredNorm() + ac.cross( abXac )*ab.squaredNorm()) / (2.f*abXac.squaredNorm()) ;
    float circumsphereRadius = toCircumsphereCenter.norm();

    // The 3 space coords of the circumsphere center then:
    Eigen::Vector3d ccs = a  +  toCircumsphereCenter ;
    PWC.pose.position.x = ccs(0);
    PWC.pose.position.y = ccs(1);
    PWC.pose.position.z = ccs(2);
    PWC.covariance[0] = 0.8;
    poses_pub.publish(PWC);
  }
}
