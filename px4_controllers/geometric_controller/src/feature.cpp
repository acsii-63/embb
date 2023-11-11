#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/feature_extract.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <controller_msgs/PositionCommand.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define latView1 21.0062703
#define longView1 105.8420085

Eigen::Vector3d View1(latView1, longView1 , 0.0);

#define latView2 21.0066709
#define longView2 105.8420836

Eigen::Vector3d View2(latView2, longView2 , 0.0);

#define latView3 21.0070603
#define longView3 105.8421051

Eigen::Vector3d View3(latView3, longView3 , 0.0);

#define latS1 21.0063531
#define longS1 105.8427953

Eigen::Vector3d S1(latS1,longS1,3.0);

#define latS2 21.0067215
#define longS2 105.8427934

Eigen::Vector3d S2(latS2,longS2,3.0);

#define latS3 21.0063433
#define longS3 105.8429603

Eigen::Vector3d S3(latS3,longS3,3.0);

#define latS4 21.0066926
#define longS4 105.8429589

Eigen::Vector3d S4(latS4,longS4,3.0);

typedef std::pair<Eigen::Vector3d,Eigen::Vector3d> PairVector;

int sample_idx=0,waypoint_idx=0;
int sample_size=0;
FeatureExtract plan;
std::vector<int> Indexwp;
bool gps_home_init = false, odom_init = false , setmode_init =false ,plan_init = false ,plan_fin =false;
Eigen::Quaterniond mav_att ;
double mav_yaw = 0,mav_yawvel = 0;
Eigen::Vector3d gps_home,gpsraw,local_start,mav_pos,mav_vel,offset;
std::vector<PairVector> gps_target,local_setpoint;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;
double UTM_V_X,UTM_V_Y;

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
 
 if(!gps_home_init && odom_init){
 local_start = mav_pos;
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y);//32 zurich 48 VietNam
 for(auto target : gps_target){
    LatLonToUTMXY(target.first(0),target.first(1),48,UTM_SP_X,UTM_SP_Y);
    LatLonToUTMXY(target.second(0),target.second(1),48,UTM_V_X,UTM_V_Y);
    Eigen::Vector3d setpoint;
    setpoint(0) = mav_pos(0) - UTM_X + UTM_SP_X;
    setpoint(1) = mav_pos(1) - UTM_Y + UTM_SP_Y;
    setpoint(2) = target.first(2);
    setpoint += offset;
    Eigen::Vector3d view;
    view(0) = mav_pos(0) - UTM_X + UTM_V_X;
    view(1) = mav_pos(1) - UTM_Y + UTM_V_Y;
    view(2) = target.second(2);
    view += offset;
    local_setpoint.push_back(PairVector(setpoint,view));
 }
 gps_home_init = true; 
 for(auto lsp: local_setpoint){
  ROS_INFO_STREAM("local_setpoint" << lsp.first(0) << " " << lsp.first(1)<<" " << lsp.first(2));
  ROS_INFO_STREAM("local_view" << lsp.second(0) << " " << lsp.second(1)<<" " << lsp.second(2));
  }
 }
}

int main(int argc, char **argv) {
  gps_target.push_back(PairVector(S1,View1));
  gps_target.push_back(PairVector(S2,View2));
  gps_target.push_back(PairVector(S1,View3));
  gps_target.push_back(PairVector(S3,View3));
  gps_target.push_back(PairVector(S4,View2));
  gps_target.push_back(PairVector(S3,View1));

  ros::init(argc, argv, "controller_gps");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  offset << 0.0,0.0,0.0;
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/global_position/global", 1, &gpsrawCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher pos_cmd = n.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd",1);
  ros::Publisher acc_cmd = n.advertise<geometry_msgs::Point>("/controller/acc_cmd",1);
  ros::ServiceClient setModeClient = n.serviceClient<controller_msgs::setmode>("/controller/set_mode");
  controller_msgs::setmode setModeCall;
  Eigen::Vector3d x_B, y_B,z_B;
  x_B << 0.0 , 1.0, -0.0;
  y_B << -1.0, 0.0, 0.0;
  z_B << 0.0, 0.0, 1.0;
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  Eigen::Quaterniond desired_attitude(R_W_B);
  ROS_INFO_STREAM("desq "<<desired_attitude.w() << " "<<desired_attitude.x() << " "<<desired_attitude.y()<<" "<<desired_attitude.z());
  
  while(ros::ok()){
    loop_rate.sleep();
    if(!setmode_init && gps_home_init){
      setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      setModeCall.request.timeout = 50;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success)
      setmode_init = true;
    }
    if(!plan_init && setmode_init){
       plan.setStart(mav_pos,mav_yaw);
       for(auto lsp : local_setpoint)
       plan.appendSetpoint(lsp);
       plan.execute();
       plan.getWpIndex(Indexwp);
       sample_size = plan.getSize();
       plan_init = true;
    }
    if(plan_init){
      sample_idx ++;
      if(sample_idx < sample_size)
      { 
        if(sample_idx > Indexwp[waypoint_idx]) waypoint_idx++;
        controller_msgs::PositionCommand cmd;
        cmd.position = toGeometry_msgs(plan.getPos(sample_idx));
        cmd.velocity = toVector3(plan.getVel(sample_idx));
        cmd.acceleration = toVector3(plan.getAcc(sample_idx));
        cmd.yaw = plan.getYaw(sample_idx);
        pos_cmd.publish(cmd);
        geometry_msgs::Point acc_msg;
        acc_msg = toGeometry_msgs(plan.getAcc(sample_idx));
        acc_cmd.publish(acc_msg);
        ROS_INFO_STREAM("Sec2 next waypoint " << (Indexwp[waypoint_idx] - sample_idx)*0.01 
        << " Sec to end " << (sample_size - sample_idx) *0.01);
      }
      else{
        setModeCall.request.mode = setModeCall.request.HOLD;
        setModeCall.request.timeout = 50;
        setModeClient.call(setModeCall);
        if(setModeCall.response.success)
        ros::shutdown();
        return 0;
      }
    }
    ros::spinOnce();
  }
   
   return 0;
 }