#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "target_track/BoundingBoxes.h"

// #include <stdio.h>
// #include <unistd.h>
// #include <stdlib.h>
// #include <sys/socket.h>
// #include <sys/stat.h>
// #include <string.h>
// #include <arpa/inet.h>

ros::Publisher vel_pub;

int lostCount = 0;
bool captured = false;
int x, y;
float d;
float kp_yaw = 1.2;
float kp_x = 3;
float kp_z = 1;
geometry_msgs::TwistStamped target_vel;
target_track::BoundingBoxes boxes;
void target_cb(const target_track::BoundingBoxes::ConstPtr& msg){
    // boxes = *msg;
    for(const auto& box : msg->bounding_boxes){
        if(box.Class == "person"){
            captured = true;
            lostCount = 0;
            x = (box.xmin + box.xmax) / 2;
            y = (box.ymin + box.ymax) / 2;
            d = box.ymax - box.ymin;
            ROS_INFO("Detected Target Person");
            ROS_INFO("x: %d", x);
            ROS_INFO("y: %d", y);
            ROS_INFO("d: %f", d);
            // target_vel.twist.angular.z = kp_yaw * (x - 320) * -0.005;
            // target_vel.twist.linear.z = kp_z * (y - 240) * -0.005;
            // target_vel.twist.linear.x = kp_x * (1 - d/240) * -5;
            // vel_pub.publish(target_vel);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolov5_track_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    ros::Subscriber target_sub = nh.subscribe<target_track::BoundingBoxes>("yolov5/targets", 10, target_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);\

    while(ros::ok()){
        ROS_INFO("captured: %d", captured);
        lostCount++;
        if(lostCount > 1000) lostCount = 1000;
        if(lostCount > 10)  captured = false;
        if(captured == false){
            target_vel.twist.angular.z = 0;
            target_vel.twist.linear.z = 0;
            target_vel.twist.linear.x = 0;
        }else if(captured == true){
            target_vel.twist.angular.z = kp_yaw * (x - 320) * -0.005;
            target_vel.twist.linear.z = kp_z * (y - 240) * -0.005;
            target_vel.twist.linear.x = kp_x * (1.0 - d/240.0);
        }
        vel_pub.publish(target_vel);
        ros::spinOnce();
        rate.sleep();
    }
}