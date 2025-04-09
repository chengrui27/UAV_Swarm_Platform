#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "target_track/BoundingBoxes.h"


ros::Publisher vel_pub;

int focus_area[2] = {0, 0};  //记录跟踪的目标的位置
int lostCount = 0;
bool captured = false;
int x, y;
float d, focus_d;
float kp_yaw = 1.0;
float kp_x = 2;
float kp_z = 1;
geometry_msgs::TwistStamped target_vel;
target_track::BoundingBoxes boxes;
void target_cb(const target_track::BoundingBoxes::ConstPtr& msg){
    // boxes = *msg;
    for(const auto& box : msg->bounding_boxes){
        if(box.Class == "person"){           
            lostCount = 0;
            x = (box.xmin + box.xmax) / 2;
            y = (box.ymin + box.ymax) / 2;
            d = box.ymax - box.ymin;
            if(captured == false){
                focus_area[0] = x;
                focus_area[1] = y;
                focus_d = d;
            }
            if(focus_area[0] - x < 80 && focus_area[0] - x > -80){
                if(focus_area[1] - y < 50 && focus_area[1] - y > -50){
                    if(focus_d - d < 10 && focus_d - d > -10){
                        focus_area[0] = x;
                        focus_area[1] = y;
                        focus_d = d;
                        captured = true;
                        ROS_INFO("x: %d", focus_area[0]);
                        ROS_INFO("y: %d", focus_area[1]);
                        ROS_INFO("d: %f", focus_d);
                    }
                    
                }
            }
            ROS_INFO("captured: %d", captured);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolov5_track_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    ros::Subscriber target_sub = nh.subscribe<target_track::BoundingBoxes>("yolov5/targets", 3, target_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    while(ros::ok()){
        // ROS_INFO("captured: %d", captured);
        lostCount++;
        if(lostCount > 1000) lostCount = 1000;
        if(lostCount > 7)  captured = false;
        if(captured == false){
            target_vel.twist.angular.z = 0;
            target_vel.twist.linear.z = 0;
            target_vel.twist.linear.x = 0;
        }else if(captured == true){
            target_vel.twist.angular.z = kp_yaw * (focus_area[0] - 320) * -0.005;
            target_vel.twist.linear.z = kp_z * (focus_area[1] - 320) * -0.005;
            target_vel.twist.linear.x = kp_x * (1.0 - focus_d/180.0);
        }
        vel_pub.publish(target_vel);
        ros::spinOnce();
        rate.sleep();
    }
}