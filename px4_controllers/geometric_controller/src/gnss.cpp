#include "ros/ros.h"
#include <cstdlib>
#include <fstream>
#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/triangle_form.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <controller_msgs/PositionCommand.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <controller_msgs/start.h>

double maxj = 5.0 , maxa = 4.0 , maxv = 2.5 , maxav = 1.2 , maxaa = 2.0;
int sample_idx=0,waypoint_idx=0;
int sample_size=0;
int seqID = 0;
double yaw_offset = 0.0;
TriangleForm plan;
std::vector<int> Indexwp;
bool gps_home_init = false, odom_init = false ,target_init = false;
Eigen::Quaterniond mav_att ;
double mav_yaw = 0,mav_yawvel = 0;
Eigen::Vector3d gps_home,gpsraw,local_start,mav_pos,mav_vel,offset;
std::vector<Eigen::Vector3d> gps_target,local_setpoint;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;
enum State {WAITING_GPS_TARGET,
            SETMODE_INIT,
            PLAN_INIT,
            PLAN_EXECUTE,
            TERMINATE
                    } state;
enum Error {Success,
            FileError
            } error;

void operator>>(const YAML::Node& node, Eigen::Vector3d& v)
{
    v.x() = node[0].as<double>();
    v.y() = node[1].as<double>();
    v.z() = node[2].as<double>();
}

double ToEulerYaw(const Eigen::Quaterniond& q){
    Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }
bool targetCallback(controller_msgs::start::Request& req, controller_msgs::start::Response& res){
  // ROS_INFO_STREAM(start);
  if(!target_init){
  gps_target.clear();
  seqID = req.seq;
  for( auto p : req.target.poses){
  Eigen::Vector3d gps_element ;
  gps_element << p.position.x, p.position.y,p.position.z;
  gps_target.push_back(gps_element);
  }
  if( gps_target.size()> 0 )
  target_init = true ;
  res.success = true;
  res.result = 1;
  return true;
  }
  return false;
}

void odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_att.w()=odomMsg.pose.pose.orientation.w;
  mav_att.x()=odomMsg.pose.pose.orientation.x;
  mav_att.y()=odomMsg.pose.pose.orientation.y;
  mav_att.z()=odomMsg.pose.pose.orientation.z;
  mav_yaw = ToEulerYaw(mav_att);
  mav_yawvel = (mav_att * toEigen(odomMsg.twist.twist.angular))(2);
  mav_pos = toEigen(odomMsg.pose.pose.position);
  mav_vel = mav_att * toEigen(odomMsg.twist.twist.linear);
  odom_init = true;
}
double distance( Eigen::Vector3d cur , Eigen::Vector3d fin) {
  Eigen::Vector3d rm = cur-fin;
  return (cur-fin).norm();
}
void gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 
 if(!gps_home_init && odom_init && !target_init){
 local_start = mav_pos;
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y);//32 zurich 48 VietNam
 gps_home_init = true; 
 }
}
void getLocalSetpoint(){
  for(auto target : gps_target){
    LatLonToUTMXY(target(0),target(1),48,UTM_SP_X,UTM_SP_Y);
    Eigen::Vector3d setpoint;
    setpoint(0) = mav_pos(0) - UTM_X + UTM_SP_X;
    setpoint(1) = mav_pos(1) - UTM_Y + UTM_SP_Y;
    setpoint(2) = target(2);
    setpoint += offset;
    local_setpoint.push_back(setpoint);
    }
  for(auto lsp: local_setpoint){
  ROS_INFO_STREAM("local_setpoint" << lsp(0) << " " << lsp(1));
  }
}

bool getYawOffset(){
   std::string llh_Path = ros::package::getPath("geometric_controller") + "/cfg/gps_longlat.yaml";
  std::ifstream llh_file(llh_Path);
  if (!llh_file.is_open())
  {
      std::cerr << "Failed to open file: " << llh_Path << std::endl;
      return false;
  }
  // Load YAML data from the file
  YAML::Node yamlNode = YAML::Load(llh_file);
  
  if (yamlNode["YawOffset"].IsDefined())
    {
        yaw_offset = yamlNode["YawOffset"].as<double>();
    }
  std::cout << "yaw_offset = : " << yaw_offset << "\n";
  return true;
}

bool getXYOffset(){
  std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
   std::ifstream file(yamlPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << yamlPath << std::endl;
        return false;
    }
    // Load YAML data from the file
    YAML::Node yamlData = YAML::Load(file);
    // Access the loaded data
    if (yamlData["offsetX"].IsDefined())
    {
        offset(0) = -yamlData["offsetX"].as<double>();
    }
    if (yamlData["offsetX"].IsDefined())
    {
        offset(1) = -yamlData["offsetY"].as<double>();
    }
  file.close();
  return true;
}


int main(int argc, char **argv) {
 
  int argc_ = 0;
  char ** argv_;
  ros::init(argc_, argv_, "controller_gps");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Rate delay_rate(5);
  if(!getYawOffset()||!getXYOffset()) return FileError;

  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/global_position/global", 1, &gpsrawCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher pos_cmd = n.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd",1);
  ros::Publisher acc_cmd = n.advertise<geometry_msgs::Point>("/controller/acc_cmd",1);
  ros::ServiceClient setModeClient = n.serviceClient<controller_msgs::setmode>("/controller/set_mode");
  ros::ServiceServer startServer = n.advertiseService("/sequence/start", &targetCallback);
  controller_msgs::setmode setModeCall;
  state = WAITING_GPS_TARGET;
  
  while(ros::ok()){
    switch (state)
    {
    case WAITING_GPS_TARGET:{
      // ROS_INFO("Waiting for target");
      if (target_init && gps_home_init){
      getLocalSetpoint();
      state = SETMODE_INIT;
      }
      break;
    }
    case SETMODE_INIT:{
      setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      setModeCall.request.timeout = 50;
      setModeCall.request.seq = seqID;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success)
         state = PLAN_INIT;
      else
        delay_rate.sleep();
      break;
    }
    case PLAN_INIT:{
      plan.setStart(mav_pos,mav_yaw);
      for(auto lsp : local_setpoint){
      plan.appendSetpoint(lsp);
      }
      plan.execute();
      plan.getWpIndex(Indexwp);
      sample_size = plan.getSize();
      std::cout << sample_size;
      state = PLAN_EXECUTE;
      break;
    }
    case PLAN_EXECUTE:{
        sample_idx ++;
        if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
        controller_msgs::PositionCommand cmd;
        cmd.position = toGeometry_msgs(plan.getPos(sample_idx));
        cmd.velocity = toVector3(plan.getVel(sample_idx));
        cmd.acceleration = toVector3(plan.getAcc(sample_idx));
        cmd.yaw = plan.getYaw(sample_idx) + yaw_offset;
        pos_cmd.publish(cmd);
        geometry_msgs::Point acc_msg;
        acc_msg = toGeometry_msgs(plan.getAcc(sample_idx));
        acc_cmd.publish(acc_msg);
        // ROS_INFO_STREAM("Sec2nxtWP" << (Indexwp[waypoint_idx] - sample_idx)*0.01 
        // << " Sec2END " << (sample_size - sample_idx) *0.01);
        if(sample_idx == sample_size - 1)
          state = TERMINATE;
      break;
    }
    case TERMINATE:{
        setModeCall.request.mode = setModeCall.request.HOLD;
        setModeCall.request.timeout = 50;
        setModeCall.request.seq = seqID;
        setModeClient.call(setModeCall);
        if(setModeCall.response.success)
        { ros::shutdown();
          return 0;}
        else delay_rate.sleep();
    }
    default:
      break;
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
   return 0;
 }