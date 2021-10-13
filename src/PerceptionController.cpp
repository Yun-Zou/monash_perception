#include <ros/ros.h>

#include "mavros_msgs/CommandBool.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

class PerceptionController {
    protected:
      double frame_required;
      double refresh_rate;
      double target_id = 0;

      double camera_pos_x;
      double camera_pos_y;
      double camera_pos_z;
      double camera_roll;
      double camera_pitch;
      double camera_yaw;
      double camera_gamma_world;

      bool requesting_detection = false;

      ros::Publisher target_publisher;
      ros::Subscriber current_pos;
      ros::ServiceServer service;

      bool request_detection(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
        
        res.success = true;
        res.result = target_id;

        ROS_INFO("request: %ld", (long int)req.value);
        if (req.value == requesting_detection) {
            res.success = false;
            ROS_INFO("Already requested. No change");
        } else if (req.value) {
            requesting_detection = req.value;
            ROS_INFO("sending back response: requesting tag [%ld]", (long int)res.result);
        } else {
            requesting_detection = req.value;
            ROS_INFO("Turning off detection");
        }

        return true;
      }

    public:
        void init() {
            this->init_params();
            this->init_publisher_subscriber();
        };

        void init_params() { 
            ros::NodeHandle nh("~");
         };

        void init_publisher_subscriber() { 
            ros::NodeHandle nh("~");

            std::string ros_namespace;
            if (!nh.hasParam("namespace")) {
              ROS_INFO("using default namespace");
            } else {
              nh.getParam("namespace", ros_namespace);
              ROS_INFO("using namespace %s", ros_namespace.c_str());
            }

            target_publisher = nh.advertise<geometry_msgs::PoseStamped>(ros_namespace + "/monash_perception/target", 5);
            service = nh.advertiseService("add_two_ints", &PerceptionController::request_detection, this);
        };

};