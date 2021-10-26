#include <ros/ros.h>

#include "apriltag_ros/AprilTagDetectionArray.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PerceptionController {
protected:

  // Params
  int consecutive_frames_required = 3;
  int refresh_rate;
  int target_id = 0;
  double target_size = 0.288;
  std::string camera_topic;
  std::string camera_frame;

  bool requesting_detection = false;

  ros::Publisher target_publisher;
  ros::Publisher target_marker;
  ros::Subscriber tag_detections;
  ros::ServiceServer service;

  tf::TransformBroadcaster tf_br;
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  std::vector<apriltag_ros::AprilTagDetection> found_array;
  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped tf_target;

  bool request_detection(mavros_msgs::CommandBool::Request &req,
                         mavros_msgs::CommandBool::Response &res);

  void tag_callback(const apriltag_ros::AprilTagDetectionArray &msg);

  void found_target();
  
  void transform_frame();

public:
  PerceptionController(ros::NodeHandle nh){init(nh);};

  void init(ros::NodeHandle nh) {
    init_params(nh);
    init_publisher_subscriber(nh);
  };

  void init_params(ros::NodeHandle nh);

  void init_publisher_subscriber(ros::NodeHandle nh);
  
  void publish_rviz_marker(geometry_msgs::PoseStamped target);
};