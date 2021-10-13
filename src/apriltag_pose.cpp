#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/String.h"

enum requested_mode {Apriltags};

// ros::Subscriber currentPos;
ros::Subscriber tag_detections;
ros::Publisher target_pose;

nav_msgs::Odometry current_pose_g;

int target_id = 0;
geometry_msgs::PoseStamped apriltag_found;
bool target_valid = true;

bool request_apriltag_detection = true;
bool apriltag_topic_subscribed = false;

tf::TransformListener tf_listener;
tf::StampedTransform transform_drone;
tf::Stamped<tf::Transform> transform_tag;

void pose_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose_g = *msg;
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
}

void apriltag_callback(const apriltag_ros::AprilTagDetectionArray &msg) {
  
  for (int i=0; i < msg.detections.size(); i++) {
    if (msg.detections[i].id[0] == target_id) {
      apriltag_found.pose = msg.detections[i].pose.pose.pose;
      apriltag_found.header = msg.detections[i].pose.header;
      tf::poseStampedMsgToTF(apriltag_found, transform_tag);
      // transform_tag
      // transform_tag();
      return;
    }
  }

  // // If set, publish the transform /tf topic
  // if (publish_tf_) {
  //   for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {
  //     geometry_msgs::PoseStamped pose;
  //     pose.pose = tag_detection_array.detections[i].pose.pose.pose;
  //     pose.header = tag_detection_array.detections[i].pose.header;
  //     tf::Stamped<tf::Transform> tag_transform;
  //     tf::poseStampedMsgToTF(pose, tag_transform);
  //     tf_pub_.sendTransform(
  //         tf::StampedTransform(tag_transform, tag_transform.stamp_,
  //                              image->header.frame_id, detection_names[i]));
  //   }
  // }

  return;
}

void tf_tag_to_world() {
  
};

int init_publisher_subscriber(ros::NodeHandle nh)
{
	std::string ros_namespace;
	if (!nh.hasParam("namespace"))
	{
		ROS_INFO("using default namespace");
	}else{
		nh.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}

  // target_pose = nh.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/monash_perception/target").c_str(), 5);
	// currentPos = nh.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);

	return 0;
}

void publish_target(geometry_msgs::PoseStamped tf_target) {

  target_pose.publish(tf_target);

}



int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "apriltag_pose_node");
  ros::NodeHandle apriltag_pose_node("~");

  init_publisher_subscriber(apriltag_pose_node);

  ros::Rate loop_rate(10);
  target_pose = apriltag_pose_node.advertise<geometry_msgs::PoseStamped>(("/monash_perception/target"), 5);
  while (ros::ok()) {

    apriltag_ros::AprilTagDetection tf_poses;
    apriltag_ros::AprilTagDetectionArray tf_poses_array;

    // target.pose = apriltag_found;
    // tf_tag_poses.publish(target);

    // Only subscribe if requested to save processing power
    // if (request_apriltag_detection && !apriltag_topic_subscribed) {
    //   tag_detections = nh.subscribe("/tag_detections", 5, apriltag_callback);
    //   apriltag_topic_subscribed = true;
    // } else if (!request_apriltag_detection) {
    //   tag_detections.shutdown();
    // }

    geometry_msgs::PoseStamped msg_target_pose;
    // msg_target_pose.header.stamp = transform.stamp_;
    // msg_target_pose.header.frame_id = transform.frame_id_;
    // msg_target_pose.pose.position.x = position_body.getX();
    // msg_target_pose.pose.position.y = position_body.getY();
    // msg_target_pose.pose.position.z = position_body.getZ();
    // msg_target_pose.pose.orientation.x = quat_body.getX();
    // msg_target_pose.pose.orientation.y = quat_body.getY();
    // msg_target_pose.pose.orientation.z = quat_body.getZ();
    // msg_target_pose.pose.orientation.w = quat_body.getW();
    msg_target_pose.header.frame_id = 1;
    msg_target_pose.pose.position.x = 4;
    msg_target_pose.pose.position.y = 9;
    msg_target_pose.pose.position.z = 6;
    msg_target_pose.pose.orientation.x = 1;
    msg_target_pose.pose.orientation.y = 0;
    msg_target_pose.pose.orientation.z = 0;
    msg_target_pose.pose.orientation.w = 0;

    if (target_valid) {
      publish_target(msg_target_pose);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


