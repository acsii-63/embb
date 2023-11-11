
#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <Eigen/Dense>


#include <visualization_msgs/Marker.h>
#include <controller_msgs/FlatTarget.h>
#include <controller_msgs/PositionCommand.h>
#include <controller_msgs/Sequence.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <dynamic_reconfigure/server.h>

#include <geometric_controller/GeometricControllerConfig.h>
#include <geometric_controller/common.h>
#include <geometric_controller/UTM.h>
#include <controller_msgs/setmode.h>
#include <controller_msgs/getmode.h>
#include <ros/package.h>

// #include <geometric_controller/logging_lib.h>
#include <std_srvs/SetBool.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#define MathPI 3.14159

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};


class geometricCtrl {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber referenceSub_,optimalPointsSub_;
  ros::Subscriber flatreferenceSub_,quad_msgsSub_;
  ros::Subscriber multiDOFJointSub_;
  ros::Subscriber mavstateSub_;
  ros::Subscriber gpsrawSub_;
  ros::Subscriber global_poseSub_;
  ros::Subscriber marker_relative_Sub_;
  ros::Subscriber mavposeSub_, mavrcSubs_ , gzmavposeSub_;
  ros::Subscriber mavtwistSub_,batterySub_;
  ros::Subscriber imuSub_;
  ros::Subscriber yawreferenceSub_,yawObstacleSub_;
  ros::Publisher setpoint_raw_Pub, debugGpsLocalPub_ , debugAccelPub_, debugVelPub_ , debugPosePub_ , debugStatePub_;
  ros::Publisher rviz1Pub_,rviz2Pub_,rvizPosePub_,rvizPathPub_,rvizAcelDesPub_,rvizAcelPub_,rvizQuatDesPub_,rvizQuatPub_,debugDragPub_;
  ros::Publisher systemstatusPub_;
  ros::Publisher sequencestatusPub_;
  ros::ServiceClient arming_client_,land_client_;
  ros::ServiceClient set_mode_client_;

  ros::ServiceServer setModeServer_,getModeServer_;
  
  ros::Timer cmdloop_timer_, statusloop_timer_ ;
  ros::Time Flight_start,last_request_;
  string mav_name_;
  bool failsafe;
  bool take_off_request_,hold_request_;
  int ctrl_mode_,control_seq;
  bool landing_detected = false;
  bool sim_enable_, rcHold , Automatic_ ;
  double reference_request_dt_;
  double UTM_X,UTM_Y,UTM_HOME_X,UTM_HOME_Y;
  double max_fb_acc_,max_ft_vel_;
  double dx_, dy_, dz_;

  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;
  mavros_msgs::CommandTOL land_cmd_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
  
  double initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_;
  Eigen::Vector3d TakeOffTargetPos_,HoldTargetPos_;
  double TakeOffYaw_,HoldYaw_;

  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  double yaw_velocity;
  double mavYaw_,mavVelYaw_;
  Eigen::Quaterniond mavAtt_;
  Eigen::Vector3d Kpos_, Kacc_ ,Kvel_, D_, Kint_;
  Eigen::Vector3d globalPos_,gpsraw,gps_pos;
  Eigen::Vector3d Imu_base;
  Eigen::Vector3d Imu_accel;
  Eigen::Vector3d desired_acc,drag_acc,gravity_force;
  Eigen::Vector3d Imu_ang_vel;
  Eigen::Vector3d integral_error,last_integral_error;
  Eigen::Vector4d globalAtt_;
  Eigen::Quaterniond q_des;

  float battery_voltage;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_, Kint_x_ ,Kint_y_ ,Kint_z_ , Kacc_x_,  Kacc_y_ ,  Kacc_z_;
  int posehistory_window_;
  void imuCallback(const sensor_msgs::Imu &msg);
  void imuloadCallback(const sensor_msgs::Imu &msg);
  void imuphysicalCallback(const sensor_msgs::Imu &msg);
  void gpsrawCallback(const sensor_msgs::NavSatFix &msg);

  void pubDebugInfo(const Eigen::Vector3d &cmd , const Eigen::Vector3d &pos , const Eigen::Vector3d &vel , const int &state);
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  void pubPoseHistory();
  void pubSystemStatus();
  void rvisualize();

  void rawsetpointPubAcc(const Eigen::Vector3d &accelration ,const double yawvel);
  void rawsetpointPubVel(const Eigen::Vector3d &velocity ,const double yawvel);
  void rawsetpointTakeOff(const Eigen::Vector3d &position , const double &yaw);
  void rawsetpointHold(const Eigen::Vector3d &position , const double &yaw);

  bool setModeCallback( controller_msgs::setmodeRequest &req , controller_msgs::setmodeResponse &res);
  bool getModeCallback( controller_msgs::getmodeRequest &req , controller_msgs::getmodeResponse &res);
  
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  void quad_msgsCallback(const controller_msgs::PositionCommand &msg);
  void yawtargetCallback(const std_msgs::Float32 &msg);
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);

  void cmdloopCallback(const ros::TimerEvent &event);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void statusloopCallback(const ros::TimerEvent &event);
  void batteryCallback(const sensor_msgs::BatteryState &state);
  void rcCallback(const mavros_msgs::RCIn &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void sequenceCallback();
  bool almostZero(double value);
  double controlyawvel();
  void checkingHoldSwitch();

  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
  double ToEulerYaw(const Eigen::Quaterniond& q);
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                  const Eigen::Vector3d &target_acc);
  
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error , int control_mask_);

  Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
  const Eigen::Vector3d& y_C,
  const Eigen::Quaterniond& attitude_estimate);
  Eigen::Quaterniond computeDesiredQuat(const Eigen::Vector3d &a_des);
  Eigen::Vector3d computeDragAcc(const Eigen::Vector3d& v_des , const Eigen::Quaterniond& q_des ,Eigen::Vector3d& drag_acc);
  enum FlightState { WAITING_FOR_HOME_POSE,
                     TAKE_OFF,
                     HOLD,
                     MISSION_EXECUTION,
                     AUTO_LAND,
                     ONGROUND,
                    } node_state;
  enum MavStatus {MAV_STATE_UNINIT,
                  MAV_STATE_BOOT,
                  MAV_STATE_CALIBRATING ,
                  MAV_STATE_STANDBY ,
                  MAV_STATE_ACTIVE ,
                  MAV_STATE_CRITICAL,
                  MAV_STATE_EMERGENCY	,
                  MAV_STATE_FLIGHT_TERMINATION} mav_status;
  enum Controller_mask { ACCELERATION_FEEDBACK , ACCELERATION_FEEDBFORWARD, VELOCITY_CONTROL } control_mask ;
  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };
  geometry_msgs::Point home_pose_;
  bool received_home_pose;
  bool gps_home_init = false;
  Eigen::Vector3d gps_home = Eigen::Vector3d::Zero();
 public:
  void dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t level);
  geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~geometricCtrl();
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif
