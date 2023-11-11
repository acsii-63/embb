#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h> // Include the YAML-CPP library
#include <fstream>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "geometric_controller/common.h"
#include "geometric_controller/UTM.h"
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/NavSatFix.h>


bool gps_home_init = false, odom_init = false, setpoint_init = false;
Eigen::Vector3d gps_home, gpsraw, local_start, mav_pos, mav_vel, offset;
Eigen::Vector3d gps_target, local_setpoint;
double UTM_X, UTM_Y;
double UTM_SP_X, UTM_SP_Y;

void operator>>(const YAML::Node& node, Eigen::Vector3d& v)
{
    v.x() = node[0].as<double>();
    v.y() = node[1].as<double>();
    v.z() = node[2].as<double>();
}

void odomCallback(const nav_msgs::Odometry& odomMsg)
{
  mav_pos = toEigen(odomMsg.pose.pose.position);
  odom_init = true;
}

void gpsrawCallback(const sensor_msgs::NavSatFix& msg)
{
  if (setpoint_init && !gps_home_init && odom_init) {
    local_start = mav_pos;
    gpsraw(0) = msg.latitude;
    gpsraw(1) = msg.longitude;
    gpsraw(2) = msg.altitude;
    LatLonToUTMXY(gpsraw(0), gpsraw(1), 48, UTM_X, UTM_Y);           // 32 zurich 48 VietNam
    LatLonToUTMXY(gps_target(0), gps_target(1), 48, UTM_SP_X, UTM_SP_Y);
    Eigen::Vector3d setpoint;
    offset(0) = -UTM_X + UTM_SP_X;
    offset(1) = -UTM_Y + UTM_SP_Y;
    std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "offsetX" << YAML::Value << offset(0);
    emitter << YAML::Key << "offsetY" << YAML::Value << offset(1);
    emitter << YAML::EndMap;
    std::ofstream file(yamlPath, std::ofstream::out | std::ofstream::trunc);
    file << emitter.c_str();
    file.close();
    std::cout << "offset setted to " << offset(0) << " " << offset(1) << std::endl;
    gps_home_init = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calib_node");
  ros::NodeHandle nh("~"); // Use a private node handle for parameters

  std::string llh_Path = ros::package::getPath("geometric_controller") + "/cfg/gps_longlat.yaml";
  std::ifstream llh_file(llh_Path);
  if (!llh_file.is_open())
  {
      std::cerr << "Failed to open file: " << llh_Path << std::endl;
      return 1;
  }
  // Load YAML data from the file
  YAML::Node yamlNode = YAML::Load(llh_file);

  char charTarget = argv[1][0];
  try
    {       YAML::Node targetNode = yamlNode[charTarget];
            if (targetNode.IsNull())
            {
                std::cout << "Invalid target: " << charTarget << "\n";
                return -1;
            }
            targetNode >> gps_target;
    }
  catch (const YAML::Exception& e)
  {
      std::cout << "Error while parsing YAML file: " << e.what() << "\n";
  }
  setpoint_init = true;

  ros::Rate loop_rate(20);
  ros::Subscriber odomSub_ = nh.subscribe("/mavros/local_position/odom", 1, &odomCallback,
                                          ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = nh.subscribe("/mavros/global_position/global", 1, &gpsrawCallback,
                                          ros::TransportHints().tcpNoDelay());

  while (ros::ok()) {
    std::cout << "OK";
    ros::spinOnce();
    if (gps_home_init)
      break;
    loop_rate.sleep();
  }

  return 0;
}
