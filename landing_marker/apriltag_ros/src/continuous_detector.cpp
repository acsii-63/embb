/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");
  stop = true;
  should_terminate = false;

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  marker_pose_publisher_ = nh.advertise<geometry_msgs::PoseWithCovariance>("/landing/marker_pose/april",1);
  terminateServer = nh.advertiseService("/landing/marker_terminate/april",&ContinuousDetector::terminateCallback,this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }
}

bool ContinuousDetector::terminateCallback(marker_planner::terminate::Request& request, marker_planner::terminate::Response& response)
{
  response.success = 1;
      switch(request.action){
    
        case request.start :{
            ROS_INFO_STREAM("APRIL_START");
            stop =false;
            break;
        }
        case request.stop :{
            ROS_INFO_STREAM("APRIL_STOP");
            stop = true;
            break;
        }
        case request.terminate :{
            ROS_INFO_STREAM("APRIL_TERMINATE");
            should_terminate = true;
            break;
        }
      }
      
      return true;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  if(should_terminate)
    ros::shutdown();
  if(stop)
    return;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  apriltag_ros::AprilTagDetectionArray arr = tag_detector_->detectTags(cv_image_,camera_info);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  int marker = 0;
  for(auto m : arr.detections){
    auto mp = Eigen::Vector3d(m.pose.pose.pose.position.x,m.pose.pose.pose.position.y,m.pose.pose.pose.position.z);
    mean+=mp;
    marker++;
  }
  mean/=marker;
  geometry_msgs::PoseWithCovariance PWC;
  PWC.pose.position.x = mean(0);
  PWC.pose.position.y = mean(1);
  PWC.pose.position.z = mean(2);
  PWC.covariance[0] = 1.0;
  if(marker>0){
    marker_pose_publisher_.publish(PWC);
  }
  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
