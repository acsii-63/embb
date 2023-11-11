#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      failsafe(false),
      node_state(WAITING_FOR_HOME_POSE) {
  
  // 4 controller input standard

  flatreferenceSub_ = nh_.subscribe("/controller/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  
  quad_msgsSub_ = nh_.subscribe("/controller/pos_cmd", 1, &geometricCtrl::quad_msgsCallback, this, ros::TransportHints().tcpNoDelay());
  
  yawreferenceSub_ = nh_.subscribe("/controller/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());

  multiDOFJointSub_ = nh_.subscribe("/controller/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this, ros::TransportHints().tcpNoDelay());
 
  //  Main loop for controller output

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this);
  
  // Main FCU output

  setpoint_raw_Pub = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

  // Status loop , need update
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this);

  // FCU basic status subscription 
  mavstateSub_ = nh_.subscribe("/mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
  
  mavrcSubs_ = nh_.subscribe("/mavros/rc/in", 1, &geometricCtrl::rcCallback, this, ros::TransportHints().tcpNoDelay());
  
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());

  gpsrawSub_= nh_.subscribe("/mavros/global_position/global", 1, &geometricCtrl::gpsrawCallback, this, ros::TransportHints().tcpNoDelay());
                              
  imuSub_ = nh_.subscribe("/mavros/imu/data",1,&geometricCtrl::imuCallback, this, ros::TransportHints().tcpNoDelay()); 
  
  batterySub_= nh_.subscribe("/mavros/battery", 1, &geometricCtrl::batteryCallback, this, ros::TransportHints().tcpNoDelay());  

  // FCU system call
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

  // Set state services
  setModeServer_ =  nh_.advertiseService("/controller/set_mode",&geometricCtrl::setModeCallback,this);
  getModeServer_ =  nh_.advertiseService("/controller/get_mode",&geometricCtrl::getModeCallback,this);
  // Sequence topics
  sequencestatusPub_ = nh_.advertise<controller_msgs::Sequence>("/sequence/status", 1);
  // Debug topics
  debugGpsLocalPub_ = nh_.advertise<geometry_msgs::Point>("/debug/localgps_pos", 1);
  debugAccelPub_ = nh_.advertise<geometry_msgs::Point>("/debug/accel_command", 1);
  debugPosePub_ = nh_.advertise<geometry_msgs::Point>("/debug/pose_command", 1);
  debugVelPub_ = nh_.advertise<geometry_msgs::Point>("/debug/vel_command", 1);
  debugDragPub_ = nh_.advertise<geometry_msgs::Point>("/debug/drag_coeff", 1);
  debugStatePub_ = nh_.advertise<std_msgs::Int16>("/debug/control_state", 1);
  // Visualize topics
  rvizPosePub_= nh_.advertise<visualization_msgs::Marker>( "/debug/rvisualize", 0 );
  rvizAcelDesPub_= nh_.advertise<visualization_msgs::Marker>( "/debug/desired_acc", 0 );
  rvizPathPub_ = nh_.advertise<nav_msgs::Path>("/debug/path", 10);
  rvizAcelPub_= nh_.advertise<visualization_msgs::Marker>( "/debug/acceleration", 0 );
  rviz1Pub_= nh_.advertise<visualization_msgs::Marker>( "/debug/1", 0 );
  rviz2Pub_= nh_.advertise<visualization_msgs::Marker>( "/debug/2", 0 );
  // Parameters loader                                
  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, 2);
  nh_private_.param<double>("max_acc", max_fb_acc_, 2.0);
  nh_private_.param<double>("max_ft_vel", max_ft_vel_, 2.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("Kp_x", Kpos_x_, 1.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 1.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 1.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 1.5);
  nh_private_.param<double>("Ka_x", Kacc_x_, 0.2);
  nh_private_.param<double>("Ka_y", Kacc_y_, 0.2);
  nh_private_.param<double>("Ka_z", Kacc_z_, 0.2);
  nh_private_.param<double>("Ki_x", Kint_x_, 0.1);
  nh_private_.param<double>("Ki_y", Kint_y_, 0.1);
  nh_private_.param<double>("Ki_z", Kint_z_, 0.1);
  nh_private_.param<double>("D_x", dx_, 0.1);
  nh_private_.param<double>("D_y", dy_, 0.1);
  nh_private_.param<double>("D_z", dz_, 0.1);
  nh_private_.param<bool>("Automatic", Automatic_, false);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  control_seq = 0;

  control_mask == ACCELERATION_FEEDBACK;
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavYaw_ = 0.0 ;
  mavVel_ << 0.0, 0.0, 0.0;
  gravity_force << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Kacc_ <<  Kacc_x_,  Kacc_y_ ,  Kacc_z_;
  Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
  rcHold = false;
  D_ << dx_, dy_, dz_;
  last_integral_error = Eigen::Vector3d::Zero();
  // creates();
  Flight_start = ros::Time::now();
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}
void geometricCtrl::batteryCallback(const sensor_msgs::BatteryState &msg){
  battery_voltage = msg.voltage;
}
void geometricCtrl::gpsrawCallback(const sensor_msgs::NavSatFix &msg){
  if(!gps_home_init){
    gps_home_init = true;
     gps_home(0) = msg.latitude;
     gps_home(1) = msg.longitude;
     gps_home(2) = msg.altitude ;
     string zone;
    LatLonToUTMXY(gps_home(0),gps_home(1),48,UTM_HOME_X,UTM_HOME_Y);//32 zurich 48 VietNam
  }
 gpsraw(0) = msg.latitude; 
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),32,UTM_X,UTM_Y); //32 zurich 48 VietNam
 gps_pos(0) = UTM_X-UTM_HOME_X;
 gps_pos(1) = UTM_Y-UTM_HOME_Y;
 gps_pos(2) = mavPos_(2);
 geometry_msgs::PoseStamped tx = toGeometry_msgs(gps_pos,mavAtt_);
 debugGpsLocalPub_.publish(tx);
}
void geometricCtrl::rcCallback(const mavros_msgs::RCIn &msg){
  if ( msg.channels.at(7) > 1500 )
  rcHold = true ;
  else 
  rcHold = false;
}
void geometricCtrl::imuCallback(const sensor_msgs::Imu &msg){
Imu_base(0) = msg.linear_acceleration.x;
Imu_base(1) = msg.linear_acceleration.y;
Imu_base(2) = msg.linear_acceleration.z;
Imu_ang_vel(0)= msg.angular_velocity.x;
Imu_ang_vel(1)= msg.angular_velocity.y;
Imu_ang_vel(2)= msg.angular_velocity.z;
Eigen::Quaterniond quat_imu ;
quat_imu.w()=msg.orientation.w;
quat_imu.x()=msg.orientation.x;
quat_imu.y()=msg.orientation.y;
quat_imu.z()=msg.orientation.z;
Imu_accel = quat_imu * Imu_base ;  
}

void geometricCtrl::quad_msgsCallback(const controller_msgs::PositionCommand &msg) {
  control_mask == ACCELERATION_FEEDBFORWARD;
  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);
  targetAcc_ = toEigen(msg.acceleration);
  mavYaw_ = double(msg.yaw);
  mavVelYaw_ = double(msg.yaw_dot);
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {

  control_mask =  Controller_mask(msg.type_mask);

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);
  
  if (control_mask == ACCELERATION_FEEDBACK) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    
  } else if (control_mask == ACCELERATION_FEEDBFORWARD) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  
  }else if (control_mask == VELOCITY_CONTROL) {
    targetPos_ = Eigen::Vector3d::Zero();
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  mavYaw_ = double(msg.data);
}


void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  control_mask == ACCELERATION_FEEDBFORWARD;

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose.position;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position); //- Eigen::Vector3d::UnitX() * 0.4 ;
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}
bool geometricCtrl::getModeCallback( controller_msgs::getmodeRequest &req , controller_msgs::getmodeResponse &res){
   switch (node_state) {
    case WAITING_FOR_HOME_POSE : {
      res.result = req.WAITING_FOR_HOME_POSE;
      break;
    }
    case TAKE_OFF : {
      res.result = req.TAKE_OFF;
      break;
    }
    case HOLD : {
      res.result = req.HOLD;
      break;
    }
    case MISSION_EXECUTION : {
      res.result = req.MISSION_EXECUTION;
      break;
    }
    case AUTO_LAND : {
      res.result = req.AUTO_LAND;
      break;
    }
    case ONGROUND : {
      res.result = req.ONGROUND;
      break;
    }
   }
   return true;
}
bool geometricCtrl::setModeCallback( controller_msgs::setmodeRequest &req , controller_msgs::setmodeResponse &res){
  switch(req.mode){
    case req.TAKE_OFF_AND_HOLD :{
      if(node_state == ONGROUND && req.sub < 5.1 && mav_status == MAV_STATE_STANDBY)
        {
         desired_acc = Eigen::Vector3d::Zero();
         TakeOffTargetPos_ << mavPos_(0) , mavPos_(1) , req.sub;
         TakeOffYaw_ = ToEulerYaw(mavAtt_);
         ROS_INFO_STREAM("TakeOffAndHold at : " << mavPos_(0) << ", " << mavPos_(1) <<", " << req.sub << " yaw " << TakeOffYaw_ << " accepted");
         node_state = TAKE_OFF;
         res.success = true;
         take_off_request_ = true;
         control_seq = req.seq;
         failsafe = req.failsafe;
        }
        else res.success = false;
    }
    break;

    case req.HOLD :{
      if(node_state != ONGROUND)
        {
         desired_acc = Eigen::Vector3d::Zero();
         HoldTargetPos_ << mavPos_(0) , mavPos_(1) , mavPos_(2);
         HoldYaw_ = ToEulerYaw(mavAtt_);
         ROS_INFO_STREAM("Hold at : " << mavPos_(0) << ", " << mavPos_(1) <<", " << mavPos_(2) <<  " yaw " << HoldYaw_ << " accepted");
         node_state = HOLD;
         hold_request_ = true;
         res.success = true;
         control_seq = req.seq;
         failsafe = req.failsafe;
        }
        else res.success = false;
    break;
    }
    case req.MISSION_EXECUTION :{
      if(node_state == HOLD)
        {
         targetPos_ << mavPos_(0) , mavPos_(1) , mavPos_(2);
         ROS_INFO_STREAM("Execute start at : " << mavPos_(0) << ", " << mavPos_(1) <<", " << mavPos_(2) << " accepted");
         node_state = MISSION_EXECUTION;
         res.success = true;
         control_seq = req.seq;
         failsafe = req.failsafe;
        }
        else res.success = false;
    break;
    }
    case req.AUTO_LAND :{
       if(node_state != ONGROUND)
        {
         desired_acc = Eigen::Vector3d::Zero();
         ROS_INFO_STREAM("Start  landing at : " << mavPos_(0) << ", " << mavPos_(1) <<", " << mavPos_(2) << " accepted");
         node_state = AUTO_LAND;
         res.success = true;
         control_seq = req.seq;
         failsafe = req.failsafe;
        }
        else res.success = false;
    break;
   
    default :
        res.success = false;
    }
    break;
    
  }
  return true;
}

void geometricCtrl::checkingHoldSwitch(){
      double TakeOffPosErr = (mavPos_ - TakeOffTargetPos_).norm();
      double TakeOffYawErr = fabs(ToEulerYaw(mavAtt_) - TakeOffYaw_);
      double TakeOffVelErr = mavVel_.norm();
      if(TakeOffPosErr < 0.2 && TakeOffYawErr < 0.1 && TakeOffVelErr < 0.2){
      HoldTargetPos_ = TakeOffTargetPos_;
      HoldYaw_ = TakeOffYaw_;
      node_state = HOLD;
      }
}

void geometricCtrl::rawsetpointPubAcc(const Eigen::Vector3d &accelration,double yawvel){
      mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_PX|a.IGNORE_PY|a.IGNORE_PZ|a.IGNORE_VX|a.IGNORE_VY|a.IGNORE_VZ|a.IGNORE_YAW);
      a.acceleration_or_force = toVector3(desired_acc);
      a.yaw_rate = yawvel;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
}
void geometricCtrl::rawsetpointPubVel(const Eigen::Vector3d &velocity,double yawvel){
      mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_PX|a.IGNORE_PY|a.IGNORE_PZ|a.IGNORE_AFX|a.IGNORE_AFY|a.IGNORE_AFZ|a.IGNORE_YAW);
      a.velocity = toVector3(velocity);
      a.yaw_rate = yawvel;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
}

void geometricCtrl::rawsetpointTakeOff(const Eigen::Vector3d &position , const double &yaw ){
      mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_AFX|a.IGNORE_AFY|a.IGNORE_AFZ|a.IGNORE_VX|a.IGNORE_VY|a.IGNORE_VZ|a.IGNORE_YAW_RATE);
      a.position = toGeometry_msgs(position);
      a.yaw = yaw;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
}

void geometricCtrl::rawsetpointHold(const Eigen::Vector3d &position , const double &yaw ){
      mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_AFX|a.IGNORE_AFY|a.IGNORE_AFZ|a.IGNORE_VX|a.IGNORE_VY|a.IGNORE_VZ|a.IGNORE_YAW_RATE);
      a.position = toGeometry_msgs(position);
      a.yaw = yaw;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
}

void geometricCtrl::sequenceCallback(){
  controller_msgs::Sequence seq_msg;
  if(node_state==ONGROUND||node_state==HOLD)
    if(failsafe)
      seq_msg.status = seq_msg.FINNISH_FAILSAFE;
    else
      seq_msg.status = seq_msg.FINNISH;
  else
    if(failsafe)
      seq_msg.status = seq_msg.ACTIVE_FAILSAFE;
    else
      seq_msg.status = seq_msg.ACTIVE;
  
  seq_msg.seq = control_seq;
  seq_msg.progress = 0.0;
  sequencestatusPub_.publish(seq_msg);
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE: {
      waitForPredicate(&received_home_pose , "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = ONGROUND;
      break;
    }
    case TAKE_OFF: {
      rawsetpointTakeOff(TakeOffTargetPos_ , TakeOffYaw_);
      checkingHoldSwitch();
      break;
    }
    case HOLD: {
      rawsetpointHold(HoldTargetPos_ , HoldYaw_);
      break;
    }
    case MISSION_EXECUTION: {
      switch(control_mask){
        case ACCELERATION_FEEDBACK: case ACCELERATION_FEEDBFORWARD: {
          desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
          yaw_velocity = controlyawvel();
          rawsetpointPubAcc(desired_acc,yaw_velocity);
          break;
        }
        case VELOCITY_CONTROL: {
          yaw_velocity = controlyawvel();
          rawsetpointPubVel(targetVel_,yaw_velocity);
          break;
        }
      }
      // updates((ros::Time::now()-Flight_start).toSec(),mavPos_(0),mavPos_(1),mavPos_(2),0,desired_acc(0),desired_acc(1),desired_acc(2),integral_error(0),integral_error(1),integral_error(2),Imu_accel(0),Imu_accel(1),Imu_accel(2),mavVel_(0),mavVel_(1),mavVel_(2), 0,0,0);
      break;
    }
    case ONGROUND:{
      break;
    }
    case AUTO_LAND:{
      rawsetpointHold(mavPos_,ToEulerYaw(mavAtt_));
      if(current_state_.mode!="AUTO.LAND"){
      land_cmd_.request.yaw = mavYaw_;
      land_client_.call(land_cmd_);
      if(land_cmd_.response.success == false)
      ROS_INFO("land_request failed");
      }
      if(mav_status == MAV_STATE_STANDBY)
      node_state = ONGROUND;
    }
  }
  sequenceCallback();
  rvisualize();
  pubDebugInfo(desired_acc,Kpos_.asDiagonal()*(mavPos_-targetPos_),Kvel_.asDiagonal()*(mavVel_-targetVel_),int(node_state));
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { 
  current_state_ = *msg;
  mav_status = MavStatus(int(current_state_.system_status));
  }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
if(Automatic_){
  if (take_off_request_ ) {
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
        arming_client_.call(arm_cmd_);
        if (arm_cmd_.response.success == true) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    else if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
        take_off_request_ = false;
      }
      last_request_ = ros::Time::now();
    } 
   }
  }
  pubSystemStatus();
}
void geometricCtrl::pubDebugInfo(const Eigen::Vector3d &acc_, const Eigen::Vector3d &pos_ , const Eigen::Vector3d &vel_ , const int &state){
  geometry_msgs::Point msg = toGeometry_msgs(acc_);
  debugAccelPub_.publish(msg);
  msg = toGeometry_msgs(pos_);
  debugPosePub_.publish(msg);
  msg = toGeometry_msgs(vel_);
  debugVelPub_.publish(msg);
  std_msgs::Int16 state_msg ;
  state_msg.data = state;
  debugStatePub_.publish(state_msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

Eigen::Vector3d geometricCtrl::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate){
  Eigen::Vector3d x_B = x_B_prototype;
  if (x_B.norm()<0.01) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (x_B_projected.norm()<0.01) {
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }
  return x_B;
}

Eigen::Quaterniond geometricCtrl::computeDesiredQuat(const Eigen::Vector3d &a_des) {
  // Reference attitude
  Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(mavYaw_, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  Eigen::Vector3d z_B;
  if(a_des.norm()<0.01){
    z_B = mavAtt_ * Eigen::Vector3d::UnitZ();
  }
  else z_B = a_des.normalized();
  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, mavAtt_);
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  Eigen::Quaterniond desired_attitude(R_W_B);
  return desired_attitude;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel ;
  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error,control_mask);
  // Desired orientation
  q_des = computeDesiredQuat(a_fb - gravity_force + drag_acc );//  
  // Rotor Drag compensation
  const Eigen::Vector3d a_dc =  computeDragAcc(target_vel,q_des,drag_acc);
  // Reference acceleration
  Eigen::Vector3d a_des = a_fb + a_ref.cwiseProduct(Kacc_) ; //+ a_dc ;
  // ROS_INFO_STREAM("ades"<<a_des(0)<<" "<<a_des(1)<<" "<<a_des(2));
  geometry_msgs::Point drag = toGeometry_msgs(a_fb);
  debugDragPub_.publish(drag);
  return a_des;
}
Eigen::Vector3d geometricCtrl::computeDragAcc(const Eigen::Vector3d& v_des , const Eigen::Quaterniond& q_des ,Eigen::Vector3d& drag_acc){
  
  Eigen::Vector3d drag_B =   (q_des.inverse() * v_des).cwiseProduct(D_);
  Eigen::Vector3d drag_W = q_des * drag_B;
  // Eigen::Vector3d drag_B = (q_des.inverse()*drag_acc);
  // ROS_INFO_STREAM("Drag"<<drag_B(0)<<" "<<drag_B(1)<<" "<<drag_B(2));
  // Eigen::Vector3d drag_B_norm = (q_des.inverse() * v_des);
  // ROS_INFO_STREAM("Vel"<<drag_B_norm(0)<<" "<<drag_B_norm(1)<<" "<<drag_B_norm(2));
  // D_ = drag_B.cwiseProduct(drag_B_norm.cwiseInverse());
  // ROS_INFO_STREAM(D_(0)<<" "<<D_(1)<<" " <<D_(2));
  drag_acc = drag_W;
  return drag_W;
}
double geometricCtrl::controlyawvel(){
  double yaw_err = mavYaw_ - ToEulerYaw(mavAtt_);
      
      while (fabs(yaw_err)> fabs(yaw_err-2*MathPI)){
       yaw_err = (yaw_err-2*MathPI);
      }
      while (fabs(yaw_err)> fabs(yaw_err+2*MathPI)){
       yaw_err = (yaw_err+2*MathPI);
      }
   return yaw_err * 0.6 + mavVelYaw_ * 0.4;
}
double geometricCtrl::ToEulerYaw(const Eigen::Quaterniond& q){
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


Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error , int control_mask_) {
  Eigen::Vector3d a_fb;
  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error ;
  last_integral_error = integral_error;
  if(current_state_.mode == "OFFBOARD" && current_state_.armed)
  integral_handle(integral_error,a_fb,-0.01,Kint_,1.5);
  // ROS_INFO_STREAM("integral_error"<<integral_error(0)<<" "<<integral_error(1)<<" "<<integral_error(2));
  // a_fb += integral_error;   
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large
  
  return a_fb;
}

bool geometricCtrl::almostZero(double value) {
  if (fabs(value) < 0.01) return true;
  else return false;
}

void geometricCtrl::rvisualize(){
  visualization_msgs::Marker drone = drone_marker();
  drone.pose.position = toGeometry_msgs(mavPos_);
  drone.pose.orientation = toGeometry_msgs(mavAtt_);
  rvizPosePub_.publish(drone);

  visualization_msgs::Marker arrow = arrow_marker();
  arrow.points.push_back(toGeometry_msgs(mavPos_));
  arrow.points.push_back(toGeometry_msgs(desired_acc - gravity_force + mavPos_));
  rvizAcelDesPub_.publish(arrow);

  arrow.color.g = 1;
  arrow.points.pop_back();
  arrow.points.push_back(toGeometry_msgs(Imu_accel + mavPos_));//gravity_force
  rvizAcelPub_.publish(arrow);

  arrow.color.r = 1;
  arrow.color.g = 0;
  arrow.color.b = 0;
  arrow.points.pop_back();
  arrow.points.push_back(toGeometry_msgs(q_des * Eigen::Vector3d::UnitX() + mavPos_));//gravity_force
  rviz1Pub_.publish(arrow);

  arrow.color.r = 0;
  arrow.color.g = 0;
  arrow.color.b = 1;
  arrow.points.pop_back();
  arrow.points.push_back(toGeometry_msgs(q_des * Eigen::Vector3d::UnitZ() + mavPos_));//gravity_force
  rviz2Pub_.publish(arrow);
}
void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.4f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.4f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.4f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.4f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.4f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.4f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.4f  ", config.Kv_z);
  } else if (Kint_x_ != config.Ki_x) {
    Kint_x_ = config.Ki_x;
    ROS_INFO("Reconfigure request : Ki_x  = %.4f  ", config.Ki_x);
  } else if (Kint_y_ != config.Ki_y) {
    Kint_y_ = config.Ki_y;
    ROS_INFO("Reconfigure request : Ki_y =%.4f  ", config.Ki_y);
  } else if (Kint_z_ != config.Ki_z) {
    Kint_z_ = config.Ki_z;
    ROS_INFO("Reconfigure request : Ki_z  = %.4f  ", config.Ki_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
}

