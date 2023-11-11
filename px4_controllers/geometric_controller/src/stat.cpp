#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
string CtrState = " ";
string poseCommand= " ";
string accelCommand= " ";
string mavPos = " ";
string mavVel = " ";
string mavAtt = " ";
string Thrust = " 0 0 0 ";
string state = " ";
string mavAcc = " ";
string battery = " ";
string gpsStatus =" ";
string green   ="\033[;32m";
string red     ="\033[1;31m";
string yellow  ="\033[1;33m";
string blue    ="\033[;34m";
string normal  ="\033[0m";
string purple= "\033[0;35m" ;    
string cyan=   "\033[0;36m"  ;  
string prefix = " ";
string suffix = " ";
string prex = "", prey ="" , prez= "";
float voltage = 0;
int pose_status = 0;
void pyschoPass(int status){
if (status == 0) {prefix = ""; suffix ="";}
else if(status == 1) {prefix = green ; suffix = normal;}
else if(status == 2) {prefix = blue ; suffix = normal;}
else if(status == 3) {prefix = cyan ; suffix = normal;}
else if(status == 4) {prefix = yellow ; suffix = normal;}
else if(status == 5) {prefix = red ; suffix = normal;}
else if(status == 6) {prefix = purple; suffix = normal;}
}
void odomCallback(const nav_msgs::Odometry &odomMsg){
if(odomMsg.pose.pose.position.z > 20.0 || fabs(odomMsg.pose.pose.position.x)> 15.0 || fabs(odomMsg.pose.pose.position.y)> 15.0) pose_status = 5 ;
else if(odomMsg.pose.pose.position.z > 15.0 || fabs(odomMsg.pose.pose.position.x)> 12.0 || fabs(odomMsg.pose.pose.position.y)> 12.0) pose_status = 4;
else if(odomMsg.pose.pose.position.z > 10.0 || fabs(odomMsg.pose.pose.position.x)> 10.0 || fabs(odomMsg.pose.pose.position.y)> 10.0) pose_status = 3;
else if(odomMsg.pose.pose.position.z > 5.0 || fabs(odomMsg.pose.pose.position.x)> 7.0 || fabs(odomMsg.pose.pose.position.y)> 7.0) pose_status = 2;
else pose_status = 1; 
pyschoPass(pose_status);
 mavPos =prefix +" x: "+ to_string(odomMsg.pose.pose.position.x) + " y: "+ to_string(odomMsg.pose.pose.position.y) + " z: "+ to_string(odomMsg.pose.pose.position.z) +suffix;
 mavAtt =" x: "+ to_string(odomMsg.pose.pose.orientation.x) + " y: "+ to_string(odomMsg.pose.pose.orientation.y) + " z: "+ to_string(odomMsg.pose.pose.orientation.z) + " w: "+ to_string(odomMsg.pose.pose.orientation.w);
}

void stateCallback(const mavros_msgs::State &msg) { 
  if (msg.armed == 1 && msg.mode == "OFFBOARD") pose_status = 1;
  else pose_status = 4;
  pyschoPass(pose_status);
  state = prefix+ "Arm status "+ to_string(msg.armed) + " Flightmode " + msg.mode + suffix;
 }
void batteryCallback(const sensor_msgs::BatteryState &msg){
  voltage = msg.voltage;
  if(voltage > 16.0) pose_status = 1;
  else if(voltage > 15.5) pose_status = 2;
  else if(voltage > 15.3) pose_status = 3;
  else if(voltage > 15.18) pose_status = 4;
  else pose_status = 5;
  pyschoPass(pose_status);
  battery = prefix + "Voltage " +to_string(voltage) +" less than "+ to_string( 100 - 5 * int((16.80 - msg.voltage)/0.15)) +" % " + suffix ;
}
void gpsStatusCallback(const mavros_msgs::GPSRAW &msg){
  if(msg.h_acc > 750 && msg.v_acc > 750 && msg.vel_acc > 200 && msg.satellites_visible > 9 ) pose_status = 1;
  else if(msg.h_acc > 500 && msg.v_acc > 500 && msg.vel_acc > 150 && msg.satellites_visible > 9 ) pose_status = 2;
  else if(msg.h_acc > 250 && msg.v_acc > 250 && msg.vel_acc > 125 && msg.satellites_visible > 9 ) pose_status = 3;
  else if(msg.h_acc > 100 && msg.v_acc > 100 && msg.vel_acc > 100 && msg.satellites_visible > 9 ) pose_status = 4;
  else pose_status = 5;
  gpsStatus= prefix + " H_acc " +to_string(int(msg.h_acc)) +" V_acc "+ to_string(int(msg.v_acc)) +" Vel_acc " + to_string(int(msg.vel_acc))+" Sat_num" +to_string(msg.satellites_visible) + suffix ;
}
void accelCallback(const geometry_msgs::Point &msg){
  accelCommand = " x " + to_string(msg.x) + " y " + to_string(msg.y) + " z " + to_string(msg.z);
}
void poscommandCallback(const geometry_msgs::Point &msg){
  poseCommand = " x " + to_string(msg.x) + " y " + to_string(msg.y) + " z " + to_string(msg.z);
}
void controlStateCallback (const std_msgs::Int16 & msg){
  switch(msg.data){
    case 0 : 
    CtrState = " WAITING_FOR_HOME_POSE";
                     
    break;

    case 1 : 
    CtrState = " TAKE_OFF_AND_HOLD";
                     
    break;

    case 2 : 

    CtrState = " HOLD";
                    
    break;

    case 3 : 

    CtrState = " MISSION_EXECUTION";
                     
    break;

    case 4 : 

    CtrState = " AUTO_LAND";
    
    break;
    
    case 5 : 

    CtrState = " ONGROUND";
    
    break;
  }
}
void imuCallback(const sensor_msgs::Imu &msg){
Eigen::Vector3d Imu_base,Imu_accel ;
Eigen::Quaterniond quat_imu;
Imu_base(0) = msg.linear_acceleration.x;
Imu_base(1) = msg.linear_acceleration.y;
Imu_base(2) = msg.linear_acceleration.z;
quat_imu.w()=msg.orientation.w;
quat_imu.x()=msg.orientation.x;
quat_imu.y()=msg.orientation.y;
quat_imu.z()=msg.orientation.z;
Imu_accel = quat_imu * Imu_base ;  
prex = "", prey ="" , prez= "";
if(Imu_accel(0)>0) prex =" ";
if(Imu_accel(1)>0) prey =" ";
if(Imu_accel(2)>0) prez =" ";
mavAcc = "  x:" + prex + to_string(Imu_accel(0)) + "  y:" + prey + to_string(Imu_accel(1)) + "  z:" + prez + to_string(Imu_accel(2));

}
void mavtwistCallback(const geometry_msgs::TwistStamped &odomMsg) {
  if (fabs(odomMsg.twist.linear.z )> 3.5 || fabs(odomMsg.twist.linear.x)> 3.5 || fabs(odomMsg.twist.linear.y)> 4.0) pose_status = 5;
else if(fabs(odomMsg.twist.linear.z )> 3.0 || fabs(odomMsg.twist.linear.x)> 3.0 || fabs(odomMsg.twist.linear.y)> 3.0) pose_status = 4;
else if(fabs(odomMsg.twist.linear.z) > 2.0 || fabs(odomMsg.twist.linear.x)> 2.0 || fabs(odomMsg.twist.linear.y)> 2.0) pose_status = 3;
else if(fabs(odomMsg.twist.linear.z )> 1.0 || fabs(odomMsg.twist.linear.x)> 1.0 || fabs(odomMsg.twist.linear.y)> 1.0) pose_status = 2;
else pose_status = 1; 
prex = "", prey ="" , prez= "";
if(odomMsg.twist.linear.x>0) prex =" ";
if(odomMsg.twist.linear.y>0) prey =" ";
if(odomMsg.twist.linear.z>0) prez =" ";
pyschoPass(pose_status);
 mavVel =prefix +" x: "+ prex + to_string(odomMsg.twist.linear.x) + " y: "+ prey + to_string(odomMsg.twist.linear.y) + " z: "+ prez +to_string(odomMsg.twist.linear.z) +suffix;
 
}
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "controller_stat");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber  mavtwistSub_ = n.subscribe("mavros/local_position/velocity_local", 1, &mavtwistCallback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber mav_stateSub_ =n.subscribe("mavros/state", 1, &stateCallback, ros::TransportHints().tcpNoDelay());  
  ros::Subscriber batterySub_ = n.subscribe("mavros/battery", 1, &batteryCallback,  ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/gpsstatus/gps1/raw", 1, &gpsStatusCallback,  ros::TransportHints().tcpNoDelay());
  ros::Subscriber imuSub_ = n.subscribe("/mavros/imu/data",1,&imuCallback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber accelCommandSub_ = n.subscribe("/debug/accel_command",1,&accelCallback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber poseCommandSub_ = n.subscribe("/debug/pose_command",1,&poscommandCallback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber ctr_stateSub_ = n.subscribe("/debug/control_state",1,&controlStateCallback,ros::TransportHints().tcpNoDelay());
  
   while(ros::ok()){
   loop_rate.sleep();
   cout << "Position " << mavPos << endl;
   cout << "Velocity " << mavVel << endl;
   cout << "Acceleration" << mavAcc << endl;
   cout << "Orientation " << mavAtt << endl;
   cout << "Pose_command" << poseCommand << endl;
   cout << "Accel_command" << accelCommand << endl;
   cout << "Ctr_State" << CtrState << endl;
   cout << "Mav_State " << state << endl;
   cout << "Battery " << battery << endl;
   cout << "GPS "<< gpsStatus << endl;
   cout << "\033[2J\033[1;1H";
   ros::spinOnce();
   }
   
   return 0;
 }