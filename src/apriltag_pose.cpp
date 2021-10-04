#include <ros/ros.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
// include API

// int callback(const apriltag_ros::AprilTagDetectionArray& array) { return 0; }

// void chatterCallback(const std_msgs::String::ConstPtr &msg) {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

void chatterCallback(const apriltag_ros::AprilTagDetectionArray &msg) {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "apriltag_pose_node");
  ros::NodeHandle nh;

  ROS_INFO("ASDF");

  ros::Subscriber tag_detections{};

  ros::Publisher tf_tag_poses{};
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  tag_detections = nh.subscribe("/tag_detections", 5, chatterCallback);

  tf_tag_poses = nh.advertise<apriltag_ros::AprilTagDetectionArray>("monash_perception/tag_detection",10);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    apriltag_ros::AprilTagDetection tf_poses;
    apriltag_ros::AprilTagDetectionArray tf_poses_array;


    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}


