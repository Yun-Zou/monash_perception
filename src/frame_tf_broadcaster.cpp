#include "nav_msgs/Path.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  // ros::Subscriber path = node.subscribe<mvg>("/body_frame/path", 5, &callback);
  float counter = 0 ;
  ros::Rate rate(10.0);
  while (node.ok()) {
    transform.setOrigin(tf::Vector3(0.0, 0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "map", "apriltags"));
    rate.sleep();
    counter++;
  }
  return 0;
};