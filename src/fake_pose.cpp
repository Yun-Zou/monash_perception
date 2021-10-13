#include "PerceptionController.cpp"

#include "ros/time.h"

ros::Publisher target_publisher;

geometry_msgs::Pose fake_target;
std::string frame_id = "/camera_link";

void init() {
  fake_target.position.x = 2;
  fake_target.position.y = 2;
  fake_target.position.z = 0;
  fake_target.orientation.x = 0.5;
  fake_target.orientation.y = 0.5;
  fake_target.orientation.z = 0.3;
  fake_target.orientation.w = 0.4;
}

void publish_pose() {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  msg.pose = fake_target;
  target_publisher.publish(msg);
}

void update_fake_target() {
  int amplitude = 2;
  float x = ((double)rand() / (RAND_MAX))*amplitude - amplitude/2;
  float y = ((double)rand() / (RAND_MAX)) * amplitude - amplitude / 2;

  fake_target.position.x += x;
  fake_target.position.y += y;
  fake_target.position.z = 0;
}

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "fake_pose");
  ros::NodeHandle fake_pose("~");
   
  init();

  PerceptionController perception_controller;
  perception_controller.init();
  

  target_publisher = fake_pose.advertise<geometry_msgs::PoseStamped>(("/monash_perception/target"), 5);

  ros::Rate rate(2.0);
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    update_fake_target();
    publish_pose();


  }
  return 0;
}