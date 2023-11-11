#include "ros/ros.h"
#include <cstdlib>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseArray.h>
#include <controller_msgs/setmode.h>
#include <controller_msgs/start.h>
#include <controller_msgs/stop.h>
#include <controller_msgs/Sequence.h>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <sys/types.h>

int count,goal,index_ = 0 ;
bool script_init = true;
controller_msgs::setmode setModeCall;
ros::ServiceClient setModeClient,startClient;
ros::ServiceServer stopServer;
ros::Publisher targetPub;
ros::Subscriber seqStatusSub;
enum Type { TAKEOFF , HOLD , LAND , MOVE , MARKER , INVALID };
enum State { INIT , ACTIVE , END } seqState;
enum Script { sMARKER , sEGO , sMOVE } ;
class Sequence {
  public:
  int count;
  int failsafe;
  bool finnish_s;
  bool failsafe_s;
  double timeout;
  double height;
  int type;
  geometry_msgs::PoseArray pose_array;
  Sequence(int failsafe_, double timeout_){
    failsafe = failsafe_;
    timeout = timeout_;
    finnish_s = false;
    failsafe_s = false;
  }
};
std::vector<Sequence> seqList;
void executeMoveScript( int scriptN )
{
    pid_t pid = fork();

    if (pid == 0) {
        // Child process
        ROS_INFO("Executing move script");
        std::string ScriptName;
        switch (scriptN)
        {
        case sMARKER:
          ScriptName = "marker.sh";
          break;
        case sMOVE:
          ScriptName = "move.sh";
          break;
        case sEGO:
          ScriptName = "ego.sh";
          break;
        
        default:
          break;
        }
        std::string bashScriptPath = ros::package::getPath("sequence_controller") + "/cfg/script/" + ScriptName;
        execl("/bin/bash", "bash", bashScriptPath.c_str(), (char*)NULL);
        _exit(0);
    } else if (pid < 0) {
        std::cerr << "Fork failed" << std::endl;
    }
}

void StartSeq(Sequence &seq,int &number){
  switch (seq.type){
    case TAKEOFF: {
      setModeCall.request.mode = setModeCall.request.TAKE_OFF_AND_HOLD;
      setModeCall.request.sub = seq.height;
      setModeCall.request.seq = number+1;
      setModeCall.request.timeout = seq.timeout;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success) seqState = ACTIVE;
      break;
    }
    case LAND: {
      setModeCall.request.mode = setModeCall.request.AUTO_LAND;
      setModeCall.request.sub = 0;
      setModeCall.request.seq = number+1;
      setModeCall.request.timeout = seq.timeout;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success) seqState = ACTIVE;
      break;
    }
    case MOVE: {
      if(script_init){
      executeMoveScript(sMOVE);
      ros::Duration(1.0).sleep();
      }
      ros::Duration(0.25).sleep();
      script_init = false;
      controller_msgs::start Start;
      Start.request.seq = number + 1;
      Start.request.sub =0.0;
      Start.request.target = seq.pose_array;
      startClient.call(Start);
      if(Start.response.success){
        seqState = ACTIVE;
        script_init = true;
      }
      break;
    }
    case MARKER: {
      if(script_init){
      executeMoveScript(sMARKER);
      ros::Duration(1.0).sleep();
      }
      ros::Duration(0.25).sleep();
      script_init = false;
      controller_msgs::start Start;
      Start.request.seq = number + 1;
      Start.request.sub = seq.height;
      Start.request.target = seq.pose_array;
      startClient.call(Start);
      if(Start.response.success){
        seqState = ACTIVE;
        script_init = true;
      }
      break;
    }
  }
 
  
}
geometry_msgs::PoseArray node2parr(YAML::Node& node){
  int t_count = node["count"].as<int>();
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "move";
        pose_array.header.seq = 0;
        for (int j = 1; j <= t_count; ++j)
        {
          std::string t_name = "t" + std::to_string(j);
          std::vector<double> t_values = node[t_name].as<std::vector<double>>();

          if (t_values.size() != 3)
          {
            ROS_WARN_STREAM("Invalid t value size for " << t_name);
            continue;
          }
          geometry_msgs::Pose pose;
          pose.position.x = t_values[0];
          pose.position.y = t_values[1];
          pose.position.z = t_values[2];
          pose_array.poses.push_back(pose);
        }
        return pose_array;
}

void Parse(std::vector<Sequence> &List , YAML::Node &yaml_node , int &count , int &goal){
  count = yaml_node["count"].as<int>();  // Access the count value
  goal = yaml_node["goal"].as<int>();

  for (int i = 1; i <= count; ++i)
  {
    std::string section_name = "s" + std::to_string(i);
    YAML::Node section_node = yaml_node[section_name];
    ROS_INFO_STREAM(i);
    std::string type = section_node["type"].as<std::string>();
    double timeout = section_node["timeout"].as<double>();
    int failsafe = section_node["failsafe"].as<int>();
    Sequence seq_(failsafe,timeout);
    switch (type[0])
    { 
      
      case 't':
      { ROS_INFO_STREAM("t");
        double height = section_node["height"].as<double>();
        seq_.height = height;
        seq_.type = TAKEOFF;
        break;
      }
      case 'm':
      { ROS_INFO_STREAM("m");
        seq_.type = MOVE;
        seq_.pose_array = node2parr(section_node);
        break;
      }
      case 'k':
      { 
        ROS_INFO_STREAM("k");
        seq_.type = MARKER;
        seq_.height = section_node["height"].as<double>();
        seq_.pose_array = node2parr(section_node);
        break;
      }
      case 'l':
      { 
        ROS_INFO_STREAM("l");
        seq_.type = LAND;
        break;
      }
      default:
        seq_.type = INVALID;
        break;
    }
    List.push_back(seq_);
  }  

}
// void seqStatusCallback(const controller_msgs::Sequence& msg){
//   ROS_INFO_STREAM(" _1_ "<< msg.seq <<" _2_ " << index_+1 <<" _3_ " << msg.status);
//   ROS_INFO_STREAM(msg);

void seqStatusCallback(const controller_msgs::Sequence& msg){
  uint se = msg.seq;
  uint st = msg.status;
  // ROS_INFO_STREAM(se<<" "<<st<<" "<<index_+1);
  if(se ==index_+1){
    switch(st){
      case msg.ACTIVE:{
        seqList[index_].finnish_s = false;
        seqList[index_].failsafe_s = false;
        break;
      }
      case msg.FINNISH:{
        seqList[index_].finnish_s = true;
        seqList[index_].failsafe_s = false;
        break;
      }
      case msg.ACTIVE_FAILSAFE:{
        seqList[index_].finnish_s = false;
        seqList[index_].failsafe_s = true;
        break;
      }
      case msg.FINNISH_FAILSAFE:{
        seqList[index_].finnish_s = true;
        seqList[index_].failsafe_s = true;
        break;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yaml_parser_node");
  ros::NodeHandle nh;
  targetPub = nh.advertise<geometry_msgs::PoseArray>("/action/move/target",1);
  setModeClient = nh.serviceClient<controller_msgs::setmode>("/controller/set_mode");
  startClient = nh.serviceClient<controller_msgs::start>("/sequence/start");
  // stopServer =  nh.advertiseService("/sequence/stop",&geometricCtrl::setModeCallback,this);
  seqStatusSub= nh.subscribe("/sequence/status", 1, seqStatusCallback, ros::TransportHints().tcpNoDelay());
  // Read and parse the YAML file
  YAML::Node yaml_node = YAML::LoadFile(ros::package::getPath("sequence_controller") + "/cfg/seq.yaml");
  ros::Duration(1.0).sleep();
  ros::Rate rate(10.0);
  
  // Process the YAML data
  int count,goal;
  Parse(seqList,yaml_node,count,goal);
  while (ros::ok()){
    rate.sleep();
    switch(seqState){
      case INIT:{
        ROS_INFO_STREAM("init" << index_ +1 );
        StartSeq(seqList[index_],index_);
        break;
      }
      case ACTIVE:{
        // ROS_INFO_STREAM("active" << index_ +1 );
        if(seqList[index_].finnish_s){
          ROS_INFO_STREAM("active_fin" << index_ +1 );
          if(seqList[index_].failsafe_s)
          index_ = seqList[index_].failsafe -1;
          else
          seqState = END;
        }
        break;
      }
      case END:{
        ROS_INFO_STREAM("end" << index_ +1 );
        index_+=1;
        // ROS_INFO_STREAM(count);
        if (index_ == goal) ros::shutdown();
        else seqState = INIT;
        break;
      }
    }
  ros::spinOnce();
  }

  return 0;
}