#ifndef __ARUCO_GRIDBOARD_NODE_H__
#define __ARUCO_GRIDBOARD_NODE_H__

#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <marker_planner/terminate.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <boost/filesystem.hpp>

namespace aruco_gridboard{
        class Node{
        private:
                boost::mutex lock_;
                ros::NodeHandle n_;
                unsigned long queue_size_;
                std::string board_path_,camera_path_;
                std::string model_description_;
                std::string detector_param_path_;
                std::string camera_frame_name_;
                bool debug_display_;
                bool subscribe;
                bool status_tracker_;
                bool should_shutdown_;
                cv::Ptr<cv::aruco::Board> board_;
                image_geometry::PinholeCameraModel camera_model_;
                cv::Mat camMatrix_;
                cv::Mat_<double> distCoeffs_;
                ros::Publisher posePub_,rvecPub_;
                cv_bridge::CvImagePtr cv_ptr;
                std_msgs::Header image_header_;
                bool got_image_;
                unsigned int lastHeaderSeq_;
                int freq_;
                bool stop,camInfoready;
                double camera_offset_x_;
                double camera_offset_y_;
                double camera_offset_z_;
                marker_planner::terminate markerTerminate;
                ros::ServiceServer terminateServer;
                cv::Ptr<cv::aruco::Board> board;
                cv::Vec3d rvec, tvec;
                ros::Subscriber caminfo_sub,img_sub;

            void waitForImage();
            bool terminateCallback(marker_planner::terminate::Request& request, marker_planner::terminate::Response& response);
            void imageCallback(const sensor_msgs::ImageConstPtr &msg);
            void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

        public:
                Node();
                ~Node();
                void spin();
        };
}
#endif
