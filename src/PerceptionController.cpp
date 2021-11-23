#include "PerceptionController.hpp"

bool PerceptionController::request_detection(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
  
  res.success = true;
  res.result = target_id; // Apriltag Marker ID 
  ros::NodeHandle nh("~");
  
  ROS_INFO("request: %ld", (long int)req.value);

  // Only subscribe to topic if true and not already subscribed
  if (req.value == requesting_detection) {
    res.success = false;
    ROS_INFO("Already requested. No change");
  } else if (req.value) {
    requesting_detection = req.value;
    tag_detections    = nh.subscribe("/tag_detections", 5, &PerceptionController::tag_callback, this);
    ROS_INFO("sending back response: requesting tag [%ld]",
             (long int)res.result);
  } else {
    requesting_detection = req.value;
    tag_detections.shutdown();
    ROS_INFO("Turning off detection");
  }

  return true;
}

void PerceptionController::tag_callback(const apriltag_ros::AprilTagDetectionArray &msg) {
  
  for (int i = 0; i < msg.detections.size(); i++) {

    // Check detection matches ID. Assume there is only one match of that ID
    if (msg.detections[i].id[0] == target_id) {
      
      if (found_array.size() > 0) {
        ros::Duration time_diff =
            found_array[found_array.size()].pose.header.stamp -
            found_array[0].pose.header.stamp;
            
        // Check that the last msg received wasn't more than 0.2 seconds ago
        if (time_diff > ros::Duration(0.2)) {
          // Start new vector
          found_array.clear();
        }
      }

      // Append the found detection
      found_array.push_back(msg.detections[i]);

      // If found targets now equal frames_required, consider it found
      if (found_array.size() >= consecutive_frames_required) {
        found_target();
      }
      return;
    }
  }

  found_array.clear();

}

void PerceptionController::found_target() {

  // Reset target pose
  target.pose.position.x = 0;
  target.pose.position.y = 0;
  target.pose.position.z = 0;
  target.pose.orientation.x = 0;
  target.pose.orientation.y = 0;
  target.pose.orientation.z = 0;
  target.pose.orientation.w = 0;

  // Average the pose
  int size = found_array.size();

  for (int i = 0; i < found_array.size(); i++) {
    target.pose.position.x += found_array[i].pose.pose.pose.position.x;
    target.pose.position.y += found_array[i].pose.pose.pose.position.y;
    target.pose.position.z += found_array[i].pose.pose.pose.position.z;
    target.pose.orientation.x += found_array[i].pose.pose.pose.orientation.x;
    target.pose.orientation.y += found_array[i].pose.pose.pose.orientation.y;
    target.pose.orientation.z += found_array[i].pose.pose.pose.orientation.z;
    target.pose.orientation.w += found_array[i].pose.pose.pose.orientation.w;
  }

  target.pose.position.x = target.pose.position.x / size;
  target.pose.position.y = target.pose.position.y / size;
  target.pose.position.z = target.pose.position.z / size;
  target.pose.orientation.x = target.pose.orientation.x / size;
  target.pose.orientation.y = target.pose.orientation.y / size;
  target.pose.orientation.z = target.pose.orientation.z / size;
  target.pose.orientation.w = target.pose.orientation.w / size;

  target.header.frame_id = "/apriltag_frame";
  target.header.stamp = found_array[size].pose.header.stamp;

  // Turn target pose into a transform
  tf::Stamped<tf::Transform> tag_transform;
  tf::poseStampedMsgToTF(target, tag_transform);

  // Get transform between camera and target
  tf::StampedTransform local_frame_transform = tf::StampedTransform(tag_transform, tag_transform.stamp_, camera_frame, "target");
  
  // Broadcast that transform
  tf_br.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, camera_frame, "target"));

  ros::Time now = ros::Time::now();

  // If transform possible
  if (tf_listener.canTransform(camera_frame,"/target", now)) {

    // Wait for transform between camera and target
    tf_listener.waitForTransform("/target", camera_frame, now, ros::Duration(3.0));
    tf_listener.lookupTransform("/target", camera_frame, now, transform);

    // Turn the transform into a pose to be published
    // TODO: Confirm if this is correct
    tf_target.header.frame_id = "/drone_local_frame";
    tf_target.header.stamp = tag_transform.stamp_;
    tf_target.pose.position.x = transform.getOrigin().x();
    tf_target.pose.position.y = transform.getOrigin().y();
    tf_target.pose.position.z = transform.getOrigin().z();
    tf_target.pose.orientation.x = transform.getRotation().x();
    tf_target.pose.orientation.y = transform.getRotation().y();
    tf_target.pose.orientation.z = transform.getRotation().z();
    tf_target.pose.orientation.w = transform.getRotation().w();

    // TODO: Account for difference in camera origin frame and local drone frame. Local drone frame changes everytime to takeoff point

    // Publish the message
    target_publisher.publish(tf_target);
    publish_rviz_marker(tf_target);
  }

  found_array.clear();
}

void PerceptionController::init_params(ros::NodeHandle nh) {

  // Read params from perception.yaml cfg
  nh.getParam("consecutive_frames_required", consecutive_frames_required);
  nh.getParam("target_id", target_id);
  nh.getParam("target_size", target_size);
  nh.getParam("camera_topic", camera_topic);
  nh.getParam("camera_frame", camera_frame);

}

void PerceptionController::init_publisher_subscriber(ros::NodeHandle nh) {
  
  std::string ros_namespace;
  
  if (!nh.hasParam("namespace")) {
    ROS_INFO("using default namespace");
  } else {
    nh.getParam("namespace", ros_namespace);
    ROS_INFO("using namespace %s", ros_namespace.c_str());
  }

  // Subscribe and Advertise
  tag_detections    = nh.subscribe((ros_namespace + "/tag_detections").c_str(), 5, &PerceptionController::tag_callback, this);
  target_publisher  = nh.advertise<geometry_msgs::PoseStamped>(ros_namespace + "/monash_perception/target", 5);
  target_marker     = nh.advertise<visualization_msgs::Marker>(ros_namespace + "/monash_perception/target_marker", 1);
  service           = nh.advertiseService("/monash_perception/request_detection", &PerceptionController::request_detection, this);
}

void PerceptionController::publish_rviz_marker(
    geometry_msgs::PoseStamped target) {
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = target.header.frame_id;
  marker.header.stamp = target.header.stamp;
  marker.ns = "target";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose = target.pose;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = target_size;
  marker.scale.y = target_size;
  marker.scale.z = 0.2f;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.9f;
  marker.color.b = 0.9f;
  marker.color.a = 1.0f;

  // Show the marker for 2 seconds
  marker.lifetime = ros::Duration(2);

  target_marker.publish(marker);
}