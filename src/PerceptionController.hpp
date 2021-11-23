// PerceptionController Class
// Handles parameters, topics and algorithms for detecting markers with AprilTags

#include <ros/ros.h>
#include "ros/node_handle.h"

#include "apriltag_ros/AprilTagDetectionArray.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

class PerceptionController {
protected:

  // Params
  double consecutive_frames_required;
  double target_id = 0;
  double target_size = 0.288;
  std::string camera_topic;
  std::string camera_frame;

  // Requesting AprilTag detection
  bool requesting_detection = false;

  // Subscribers and Publishers
  ros::Publisher target_publisher;    // Target position publisher
  ros::Publisher target_marker;       // Rviz target marker publisher
  ros::Subscriber tag_detections;     // Apriltag detections subscriber
  ros::ServiceServer service;         // Apriltag request service

  tf::TransformBroadcaster tf_br;     // TF Broadcaster for transform between camera and target
  tf::TransformListener tf_listener;  // TF Listener for the transform
  tf::StampedTransform transform;     // Transform between camera and local drone frame

  std::vector<apriltag_ros::AprilTagDetection> found_array; // Vector of detected detections from apriltag_ros
  geometry_msgs::PoseStamped target;                        // Pose of target in camera frame 
  geometry_msgs::PoseStamped tf_target;                     // Pose of target in local drone frame


  /**
   * @brief Service request to subscribe to tag_detections. apriltag_ros package will start detecting when topic is subscribed.
            Detection process is computationally expensive. Should limit detection when not needed
   * 
   * @param req Request detection boolean. Uses mavros_msgs CommandBool type
   * @param res Status on request. Uses mavros_msgs CommandBool type
   * @return true 
   * @return false 
   */
  bool request_detection(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res);

  /**
   * @brief Check if apriltag_ros detection array has the target we need. Add it to vector of found targets
   * 
   * @param msg apriltag_ros published topic with detections
   */
  void tag_callback(const apriltag_ros::AprilTagDetectionArray &msg);

  /**
   * @brief Handles the vector of found targets, averages results, transforms it and publishes results
   * 
   */
  void found_target();

public:

  /**
   * @brief Construct a new Perception Controller object. Initialise params and publishers/subscribers
   * 
   * @param nh ROS NodeHandle
   */
  PerceptionController(ros::NodeHandle nh){
    init_params(nh);
    init_publisher_subscriber(nh);
  };

  /**
  * @brief Initalise params which are read from cfg/perception.yaml file
  * 
  * @param nh ROS NodeHandle
  */
  void init_params(ros::NodeHandle nh);

  /**
   * @brief Initalise publishers and subscribers
   * 
   * @param nh ROS NodeHandle
   */
  void init_publisher_subscriber(ros::NodeHandle nh);
  
  /**
   * @brief Publish the marker found in rviz compatible form so that it can be visualised
   * 
   * @param target Pose of the target found
   */
  void publish_rviz_marker(geometry_msgs::PoseStamped target);
};