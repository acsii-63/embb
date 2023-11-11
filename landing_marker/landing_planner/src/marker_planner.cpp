#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <controller_msgs/FlatTarget.h>
#include <visualization_msgs/Marker.h>
#include <controller_msgs/setmode.h>
#include <controller_msgs/getmode.h>
#include <controller_msgs/start.h>
#include <marker_planner/terminate.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/resource.h>
#include <deque>
#include <vector>


class Gauss_pdf{
private: 
double mean = 0 ;
double sigma = 0.75;
constexpr static const double inv_sqrt_2pi = 0.3989422804014327;

public:
Gauss_pdf() {}
Gauss_pdf(double mean_ , double sigma_){
  mean = mean_;
  sigma = sigma_;
}
~Gauss_pdf(){}
void set_param(double mean_ , double sigma_) { mean = mean_; sigma = sigma_; }
double normal_pdf (double x)
{      
    double a = (x - mean) / sigma;

    return inv_sqrt_2pi / sigma * std::exp(-0.5f * a * a);
}
};

class Aruco_event{
private: 
ros::Time Event_start ;
Eigen::Vector3d sigma =  Eigen::Vector3d(1,1,1);
Eigen::Vector3d scalar = Eigen::Vector3d::Zero();
Gauss_pdf function[3];

public:
Aruco_event() {}
Aruco_event(Eigen::Vector3d sigma_, Eigen::Vector3d scalar_){
  Event_start  = ros::Time::now();
  sigma = sigma_;
  scalar = scalar_;
  for(int i =0 ; i < 3 ; i++){
	function[i] = Gauss_pdf(sigma[i]*3,sigma[i]);
  }
}
~Aruco_event(){}
bool check_available(){
	if ((ros::Time::now()-Event_start).toSec()<6*std::max(std::max(sigma(0),sigma(1)),sigma(2))) return true ;
	else return false ;
}
Eigen::Vector3d getscalar ()
{
return scalar;
}
Eigen::Vector3d getfuction_value ()
{
Eigen::Vector3d result;
    for(int i =0 ; i < 3 ; i++){
		result(i) = function[i].normal_pdf((ros::Time::now()-Event_start).toSec())*scalar(i);
	}
return result;
}
Eigen::Vector3d getz_value ()
{
    Eigen::Vector3d zvalue = ((ros::Time::now()-Event_start).toSec()*Eigen::Vector3d(1.0f,1.0f,1.0f)- 3.0 * sigma).cwiseProduct(sigma.cwiseInverse());
    return zvalue;
}
};

class MarkerPlanner
{
private:
	ros::NodeHandle nh;

	/******************************Subscriber***********************************/
	ros::Subscriber aruco_sub,whycon_sub,april_sub;
	ros::Subscriber odom_sub;

	/******************************Publisher***********************************/
	ros::Publisher pose_pub,setpoint_pub;
	ros::Publisher controller_pub;

    /******************************Publisher***********************************/
	ros::Timer planner,pose_mk;
	/******************************Variable***********************************/
	Gauss_pdf reference,matching,height_matching;
	bool safezone,marker_received;
	bool marker_terminated = false , planner_terminated = false;
	std::vector<bool> marker_enabled,marker_activated;
	std::vector<double> marker_range_max, marker_range_min;
	bool start =false,odom_ready = false;
	int marker_number,marker_number_total;
	Eigen::Vector3d discovery = Eigen::Vector3d::Zero();
	Eigen::Vector3d output = Eigen::Vector3d::Zero();
	Eigen::Vector3d safezone_output = Eigen::Vector3d::Zero();
	double zreference[100] = {0};
	double height_coeff=0.4;
    std::deque<Aruco_event> event_dq;
	Eigen::Quaterniond base_quaternion;
	Eigen::Vector3d base_pose;
	double trust_coeff;
	double discount,surcharge;
	std::vector<geometry_msgs::Point> pose_points,target_points,marker_points;
	enum state {INIT=0,EXECUTE=1,FINNISH=2} state_;
	int seqID = 0;
	controller_msgs::setmode setModeCall;
	marker_planner::terminate markerTerminate;
	ros::ServiceClient setModeClient;
	ros::ServiceClient terminateClientWhycon,terminateClientAruco,terminateClientApril;
	ros::ServiceServer startServer;

public:
	/*
	 * Constructor
	 */
	MarkerPlanner() : nh("")
	{	
		marker_number = 3;

		marker_number_total = 3;

		marker_activated.resize(3,false);

		aruco_sub = nh.subscribe("/landing/marker_pose/aruco",1,&MarkerPlanner::marker_callback,this);

		whycon_sub = nh.subscribe("/landing/marker_pose/whycon",1,&MarkerPlanner::marker_callback,this);

		odom_sub = nh.subscribe("/mavros/local_position/odom",1, &MarkerPlanner::odom_callback,this);

		april_sub = nh.subscribe("/landing/marker_pose/april",1,&MarkerPlanner::marker_callback,this);

		setModeClient = nh.serviceClient<controller_msgs::setmode>("/controller/set_mode");

		terminateClientWhycon = nh.serviceClient<marker_planner::terminate>("/landing/marker_terminate/whycon");

		terminateClientAruco = nh.serviceClient<marker_planner::terminate>("/landing/marker_terminate/aruco");

		terminateClientApril = nh.serviceClient<marker_planner::terminate>("/landing/marker_terminate/april");

		pose_pub = nh.advertise<visualization_msgs::Marker>("/landing/array/pose", 1);

		setpoint_pub = nh.advertise<visualization_msgs::Marker>("/landing/array/setpoint", 1);

		controller_pub = nh.advertise<controller_msgs::FlatTarget>("/controller/flatsetpoint",2);

		planner = nh.createTimer(ros::Duration(0.03), &MarkerPlanner::plannerCallback , this);

		pose_mk = nh.createTimer(ros::Duration(0.03), &MarkerPlanner::rvisualize , this);

		startServer = nh.advertiseService("/sequence/start", &MarkerPlanner::startCallback , this);
	
		double temp = 0;
		reference.set_param(0,1.0);
		for (int i = 0; i < 100; i++) {
			temp += reference.normal_pdf(-3.0+0.06*i)*0.06;
			zreference[i] = temp;  
		}
		discount = 1.0;
		surcharge = 0.05;
		safezone = false;
		marker_received = false;
		state_ = INIT;
		matching.set_param(0,3.0);
		height_matching.set_param(0,0.2);
	}
	Eigen::Vector3d landing_limmiter(Eigen::Vector3d &raw){
		double limmited =  std::min(std::max(fabs(raw(0))/sin(0.35),fabs(raw(1))/sin(0.65)),25.0);
		return raw + Eigen::Vector3d::UnitZ() * (limmited + 1);
	}

	void activate_marker(double height){
		for(int i=0; i<marker_number_total; i++){
			if(marker_enabled[i])
			{	
				bool inrange = false;

				if(height > marker_range_min[i] && height < marker_range_max[i]){
					inrange = true;
				}
				if(inrange && !marker_activated[i]){
					markerTerminate.request.action=markerTerminate.request.start;
					switch(i){
						case 0 :{
							terminateClientWhycon.call(markerTerminate);
							break;
						} 
						case 1 :{
							terminateClientAruco.call(markerTerminate);
							break;
						} 
						case 2 :{
							terminateClientApril.call(markerTerminate);
							break;
						} 
					}	
					if(markerTerminate.response.success == 1){
					marker_activated[i] = true;
					}
					else{
						ROS_ERROR_STREAM("cannot activate maker " << i);
					}
				}
				if(!inrange && marker_activated[i]){
					markerTerminate.request.action=markerTerminate.request.stop;
					switch(i){
						case 0 :{
							terminateClientWhycon.call(markerTerminate);
							break;
						} 
						case 1 :{
							terminateClientAruco.call(markerTerminate);
							break;
						} 
						case 2 :{
							terminateClientApril.call(markerTerminate);
							break;
						} 
					}	
					if(markerTerminate.response.success == 1){
					marker_activated[i] = false;
					}
					else{
						ROS_ERROR_STREAM("cannot stop maker " << i);
					}
				}
			}
			
		}
	}

	void pose2Range(std::vector<geometry_msgs::Pose> parray){
		ROS_INFO("planner pose2range");
		marker_range_max.clear();
		marker_range_min.clear();
		marker_enabled.clear();
		for(auto p : parray){ 
			marker_range_max.push_back(p.position.x);
			marker_range_min.push_back(p.position.y);
			
			if(p.position.z > 0)
			{
				marker_enabled.push_back(true);
				marker_number++;
			}
			else
			{
				marker_enabled.push_back(false);
			}
		}
	}

	bool startCallback(controller_msgs::start::Request& req, controller_msgs::start::Response& res){
  		ROS_INFO("planner start callback");
  		if(!start){
  			seqID = req.seq;
			// safe_height = req.sub;
			pose2Range(req.target.poses);
  			start = true ;
  			res.success = true;
  			res.result = 1;
			// activate_marker(10.0);
  			return true;
		ROS_INFO("planner start callback successful");
  		}
		ROS_INFO("planner start callback fail");
  		return false;
	}

	void marker_callback(const geometry_msgs::PoseWithCovariance &pose){
		if(start){
			Eigen::Quaterniond Base_camera = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.70710678118,-0.70710678118,0.0)));
			Eigen::Vector3d M_Cam = toEigen(pose.pose.position);
			Eigen::Vector3d M_Base  = Base_camera * M_Cam;
			Eigen::Vector3d M_Inertia  = base_quaternion * M_Base;
			Eigen::Vector3d Setpoint;
			Setpoint = landing_limmiter(M_Inertia);
			if(M_Inertia(2) < -2.0){
				safezone = false;
			}
			else
				safezone = true;
			
			if(target_points.size() > 20) 
				target_points.erase(target_points.begin());
			target_points.push_back(toGeometry_msgs(Setpoint+base_pose));
			event_execute(Setpoint);
			marker_received = true;
		}
	}

	Eigen::Vector3d getzreference(Eigen::Vector3d zvalue){
		Eigen::Vector3d result;
		zvalue += 3.0 * Eigen::Vector3d(1,1,1);
		zvalue /= 0.06;
		Eigen::Vector3i index (floor(zvalue(0)),floor(zvalue(1)),floor(zvalue(2)));
		for(int i=0 ; i < 3 ; i++) {
		if( index(i) < 0 ) result(i) = 1;
		else if( index(i) > 99 ) result(i) = 0;
		else result(i) = 1 - zreference[index(i)] ;
		}
		return result;
	}
	void plannerCallback(const ros::TimerEvent &event){
		if(odom_ready && start)
		activate_marker(base_pose(2));
		switch (state_) {
			case INIT : {
				if(marker_received){
			    	setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      				setModeCall.request.timeout = 50;
					setModeCall.request.seq = seqID;
      				setModeClient.call(setModeCall);
      				if(setModeCall.response.success)
      				state_ = EXECUTE;
					else{
						ROS_INFO_STREAM("failed");
					}
				}
				break;
			}
			case EXECUTE : {
				if(safezone){	
					discount *= 0.95;
					surcharge /= 0.95;
					surcharge = std::min(surcharge,1.0);
					discount = std::max(discount,0.04);
				}
				else
				{
					discount /= 0.95;
					surcharge *= 0.95;
					surcharge = std::max(surcharge,0.04);
					discount = std::min(discount,1.0);
				}
				check_event_deque();
				discovery = Eigen::Vector3d::Zero();
				output = Eigen::Vector3d::Zero();

				controller_msgs::FlatTarget control_msg;
				control_msg.header.stamp = ros::Time::now();
				control_msg.type_mask = 2 ;
				Eigen::Vector3d vel_sp = Eigen::Vector3d::Zero();
				if(event_dq.size()>0){
					for (int i = 0 ; i < event_dq.size();i++){
						discovery += getzreference(event_dq[i].getz_value()).cwiseProduct(event_dq[i].getscalar());
						output += event_dq[i].getfuction_value();
					}
					vel_sp = output*discount;
				}
				vel_sp += safezone_output * surcharge;
				if(vel_sp.norm()<0.001) vel_sp = Eigen::Vector3d::UnitZ()*0.001;
				control_msg.velocity = toV3_msgs(vel_sp);
				controller_pub.publish(control_msg);
				if(discount < 0.05 && safezone_output.norm()<0.1) state_ = FINNISH;
				break;
			}
			case FINNISH : {
				if(!planner_terminated){
				setModeCall.request.mode = setModeCall.request.AUTO_LAND;
      			setModeCall.request.timeout = 50;
      			setModeClient.call(setModeCall);
      			if(setModeCall.response.success){
				planner_terminated = true;}
				}
				if(!marker_terminated){
				markerTerminate.request.action=markerTerminate.request.terminate;
				terminateClientWhycon.call(markerTerminate);
				terminateClientAruco.call(markerTerminate);
				terminateClientApril.call(markerTerminate);
				marker_terminated = true;
				}
				if(marker_terminated && planner_terminated){
				ROS_INFO("Marker Planner Shutting down");
      			ros::shutdown();
				}
				break;
			}

		}
		
	}
	void event_execute(Eigen::Vector3d &setpoint){
		Eigen::Vector3d discovarable = setpoint - discovery;
		if(discovarable.hasNaN())
			return;
		else if(safezone){
			safezone_output = setpoint;
		}
		else{
			Eigen::Vector3d event_sigma = Eigen::Vector3d(4.0,4.0,6.0) - 18 * Eigen::Vector3d(matching.normal_pdf(discovarable(0)),matching.normal_pdf(discovarable(1)),matching.normal_pdf(discovarable(2)));
			Aruco_event new_event(event_sigma,discovarable);
			event_dq.push_back(new_event);
		}
	}
	void odom_callback(const nav_msgs::Odometry &msg){
		if(!isnan(msg.pose.pose.orientation.w)&&!isnan(msg.pose.pose.orientation.x)&&!isnan(msg.pose.pose.orientation.y)&&!isnan(msg.pose.pose.orientation.z)){
		base_quaternion.w() = msg.pose.pose.orientation.w;
		base_quaternion.x() = msg.pose.pose.orientation.x;
		base_quaternion.y() = msg.pose.pose.orientation.y;
		base_quaternion.z() = msg.pose.pose.orientation.z;
		base_pose = toEigen(msg.pose.pose.position);
		Eigen::Vector3d linear_velocity = toEigen(msg.twist.twist.linear);
		Eigen::Vector3d angular_velocity = toEigen(msg.twist.twist.angular);
		trust_coeff = 0.05/(log(linear_velocity.squaredNorm()+1.5)*log(angular_velocity.squaredNorm()+1.135));
		}
		pose_points.push_back(msg.pose.pose.position);
		odom_ready = true;
	}
	void check_event_deque(){
		if(event_dq.size()>0)
		while (!event_dq.at(0).check_available()) {
			event_dq.pop_front();
			if (event_dq.size() < 1 )
			break;
		}
	}
	geometry_msgs::Quaternion toGeometry_msgs( Eigen::Quaterniond e){
		geometry_msgs::Quaternion q;
		q.w = e.w();
		q.x = e.x();
		q.y = e.y();
		q.z = e.z();
		return q;
	}
	geometry_msgs::Point toGeometry_msgs( const Eigen::Vector3d &v3 ){
		geometry_msgs::Point r;
		r.x =v3(0) ;
		r.y =v3(1) ;
		r.z =v3(2) ;
		return r;
		}
	geometry_msgs::Vector3 toV3_msgs( const Eigen::Vector3d &v3 ){
		geometry_msgs::Vector3 r;
		r.x =v3(0) ;
		r.y =v3(1) ;
		r.z =v3(2) ;
		return r;
		}
	void rvisualize(const ros::TimerEvent &event){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "accelerations";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points = pose_points;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0; 
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		pose_pub.publish(marker);
		marker.points = target_points;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		setpoint_pub.publish(marker);
	}
		
	inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
     Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
     return ev3;
    }
	inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
     Eigen::Vector3d ev3(p.x, p.y, p.z);
     return ev3;
    }
	inline Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &p) {
     Eigen::Quaterniond q4(p.w ,p.x, p.y, p.z);
     return q4;
    }
	inline Eigen::Vector4d toEigen4V(const geometry_msgs::Quaternion &p) {
     Eigen::Vector4d v4(p.w ,p.x, p.y, p.z);
     return v4;
    }

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_detect");

	MarkerPlanner ArucoDetector;

	ROS_INFO("planner started");

	ros::spin();
}