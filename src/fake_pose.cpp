#include "PerceptionController.cpp"

#include "ros/time.h"

ros::Publisher target_publisher;

geometry_msgs::PoseStamped fake_target;
std::string frame_id = "/drone_local_frame";

void init() {
  fake_target.pose.position.x = 0;
  fake_target.pose.position.y = 0;
  fake_target.pose.position.z = 0;
  fake_target.pose.orientation.x = 0.0;
  fake_target.pose.orientation.y = 0.0;
  fake_target.pose.orientation.z = 0.0;
  fake_target.pose.orientation.w = 1.0;
}

void publish_pose() {
  target_publisher.publish(fake_target);
}

void update_fake_target() {
  int amplitude = 2;
  float x = ((double)rand() / (RAND_MAX))*amplitude - amplitude/2;
  float y = ((double)rand() / (RAND_MAX)) * amplitude - amplitude / 2;

  fake_target.header.frame_id = frame_id;
  fake_target.header.stamp = ros::Time::now();

  fake_target.pose.position.x += x;
  fake_target.pose.position.y += y;
  fake_target.pose.position.z = 0;
}

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "fake_pose");
  ros::NodeHandle fake_pose("~");
   
  init();

  PerceptionController perception_controller(fake_pose);
  

  target_publisher = fake_pose.advertise<geometry_msgs::PoseStamped>(("/monash_perception/target"), 5);

  ros::Rate rate(2.0);
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    update_fake_target();
    publish_pose();
    perception_controller.publish_rviz_marker(fake_target);


  }
  return 0;
}