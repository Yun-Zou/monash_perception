#include "PerceptionController.hpp"

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "apriltag_pose");
  ros::NodeHandle apriltag_pose("~");

  // PerceptionController perception_controller(apriltag_pose);

  ros::Rate rate(10.0);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

  }
  return 0;
}
