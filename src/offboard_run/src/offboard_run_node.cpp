/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>

#include <common/mavlink.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

int start = 0;  //起飞标志
geometry_msgs::PoseStamped target_pose;
void handle_mavlink_message(const mavlink_message_t &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    /*******************************************************************************/
    //设置和绑定socket端口
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd == -1){
      perror("socket error");
      exit(1);
    }  

    //设置自己的端口号等信息，并于本地socket绑定
    struct sockaddr_in local_socket;
    memset(&local_socket, 0, sizeof(local_socket));
    local_socket.sin_family = AF_INET;
    local_socket.sin_port = htons(2001);
    local_socket.sin_addr.s_addr = htonl(INADDR_ANY);
    socklen_t ser_len = sizeof(local_socket);
    int ret = bind(fd, (struct sockaddr*)&local_socket, sizeof(local_socket));
    if(ret == -1){
        perror("bind error");
        exit(1);
    }

    

    //设置通信对象的端口号等信息
    struct sockaddr_in target_socket;
    target_socket.sin_family = AF_INET;
    target_socket.sin_port = 3001;
    target_socket.sin_addr.s_addr = (INADDR_ANY);
    socklen_t tar_len = sizeof(target_socket);

    //地面站始终开通65535端口用于接收连接请求
    struct sockaddr_in link_request;
    link_request.sin_family = AF_INET;
    link_request.sin_port = 65535;
    link_request.sin_addr.s_addr = (INADDR_ANY);
    socklen_t link_len = sizeof(link_request);
    

    char rx_buf[MAVLINK_MAX_PACKET_LEN] = {0};
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN] = {0};

    mavlink_message_t heartbreak_msg;
    mavlink_message_t position_msg;

    int heartbreak_rate = 0;  //通过heartbeat_rate计数来实现1s发送一次心跳包
    int linkrequest_rate = 0;  //通过linkrequest_rate计数来实现1s发送一次连接请求
    int linkrequest = 2001;  //向地面站发送自己的端口号以请求连接
    /*******************************************************************************/

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    //设置循环频率为20Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // geometry_msgs::PoseStamped target_pose;
    target_pose.pose = current_pos.pose;
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        /*******************************************************************************/
        //接收地面站的udp消息放入buf缓存区，并处理转换为mavlink消息
        int recvlen = recvfrom(fd, rx_buf, sizeof(rx_buf), MSG_DONTWAIT, 
                    (struct sockaddr*)&target_socket, &tar_len);
        if(recvlen == -1){
            // perror("recvform error");
        }

        mavlink_message_t msg;
        mavlink_status_t status;
        for(int i = 0; i < sizeof(rx_buf); i++){
            //将buf中的消息一字节一字节的移到msg中，当转移完成后进入handle_mavlink_message（）函数解码
            if(mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)){
                handle_mavlink_message(msg);
            }
        }

        /*******************************************************************************/
        
        /*******************************************************************************/
        //向地面站发送连接请求
        linkrequest_rate ++;
        if(linkrequest_rate % 20 == 0){
            char linkrequest_data[8] = {0};
            memcpy(linkrequest_data, &linkrequest, sizeof(linkrequest));
            sendto(fd, linkrequest_data, strlen(linkrequest_data)+1, 0, (struct sockaddr*)&link_request, sizeof(link_request));
        }

        //向地面站发送心跳包
        heartbreak_rate++;
        if (heartbreak_rate % 20 == 7){
            mavlink_msg_heartbeat_pack(
                1,  // 系统 ID
                1,  // 组件 ID
                &heartbreak_msg,
                MAV_TYPE_QUADROTOR,       // 设备类型（如地面站）
                MAV_AUTOPILOT_PX4, // 无自动驾驶仪
                0,  // 基础模式
                0,  // 自定义模式
                MAV_STATE_ACTIVE   // 系统状态
            );
            int heartbreak_len = mavlink_msg_to_send_buffer(tx_buf, &heartbreak_msg);
            sendto(fd, tx_buf, heartbreak_len, 0, (struct sockaddr*)&target_socket, sizeof(target_socket));
            std::cout << "Heartbeat sent!" << std::endl;
        }

        //向地面站发送位置信息
        mavlink_msg_local_position_ned_pack(
            1,  // 系统 ID
            1,  // 组件 ID
            &position_msg,
            ros::Time::now().toSec() * 1e3,  // 时间戳（毫秒）
            current_pos.pose.position.x,
            current_pos.pose.position.y,
            current_pos.pose.position.z,
            0, 0, 0  // 速度数据（不发送）
        );
        int pos_len = mavlink_msg_to_send_buffer(tx_buf, &position_msg);
        sendto(fd, tx_buf, pos_len, 0, (struct sockaddr*)&target_socket, sizeof(target_socket));


        // //发送位置信息到地面站(废案，早期通过自定义通讯协议发送位置信息)
        // float x = current_pos.pose.position.x;
        // float y = current_pos.pose.position.y;
        // float z = current_pos.pose.position.z;
        // // ROS_INFO("%f", x);
        // // ROS_INFO("%f", y);
        // // ROS_INFO("%f", z);
        // char pose_data[128] = {0};
        // int len = sizeof(x);
        // memcpy(pose_data, &x, len);
        // memcpy(pose_data + len, &y, len);
        // memcpy(pose_data + len*2, &z, len);
        // sendto(fd, pose_data, strlen(pose_data)+1, 0, (struct sockaddr*)&target_socket, sizeof(target_socket));

        //数据拆分测试代码（将double类型的x,y,z坐标转化为字符串，存入char[]数组中，用于发送udp消息）
        // char test[100] = {0};
        // double a = 1;
        // double b = 2;
        // double c = 3;
        // int len = sizeof(a);
        // memcpy(test, &a, len);
        // memcpy(test + len, &b, len);
        // memcpy(test + len*2, &c, len);
        // char A[10] = {0};
        // char B[10] = {0};
        // char C[10] = {0};
        // A[0] = test[0];
        // A[1] = test[1];
        // A[2] = test[2];
        // A[3] = test[3];
        // A[4] = test[4];
        // A[5] = test[5];
        // A[6] = test[6];
        // A[7] = test[7];
        // B[0] = test[8];
        // B[1] = test[9];
        // B[2] = test[10];
        // B[3] = test[11];
        // B[4] = test[12];
        // B[5] = test[13];
        // B[6] = test[14];
        // B[7] = test[15];
        // C[0] = test[16];
        // C[1] = test[17];
        // C[2] = test[18];
        // C[3] = test[19];
        // C[4] = test[20];
        // C[5] = test[21];
        // C[6] = test[22];
        // C[7] = test[23];
        // float x = *(double*) A;
        // float y = *(double*) B;
        // float z = *(double*) C;
        // ROS_INFO("%f", x);
        // ROS_INFO("%f", y);
        // ROS_INFO("%f", z);
        /*******************************************************************************/

        if(start == 1){
            if( current_state.mode != "OFFBOARD" &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( set_mode_client.call(offb_set_mode) &&
                   offb_set_mode.response.mode_sent){
                   ROS_INFO("Offboard enabled");
               }
               last_request = ros::Time::now();
            } else {
               if( !current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))){
                   if( arming_client.call(arm_cmd) &&
                       arm_cmd.response.success){
                       ROS_INFO("Vehicle armed");
                   }
                   last_request = ros::Time::now();
               }
            }          
        }
        
        // local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();

        // if(current_state.armed && current_state.mode == "OFFBOARD"){
        //     if(target_pose.pose.position.z - current_pos.pose.position.z < 0.2){
        //         target_pose.pose.position.x = 3;
        //         target_pose.pose.position.y = 4;
        //         target_pose.pose.position.z = 5;
        //     }
        // }

    }

    return 0;
}

//处理接收到的mavlink消息并执行指定任务
void handle_mavlink_message(const mavlink_message_t &msg){
    switch(msg.msgid){
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:{
            mavlink_set_position_target_local_ned_t position_target;
            mavlink_msg_set_position_target_local_ned_decode(&msg, &position_target);
            target_pose.pose.position.x = position_target.x;
            target_pose.pose.position.y = position_target.y;
            target_pose.pose.position.z = position_target.z;
            // ROS_INFO("Received SET_POSITION_TARGET_LOCAL_NED: x=%.2f, y=%.2f, z=%.2f",
            //              position_target.x, position_target.y, position_target.z);
            break;
        }case MAVLINK_MSG_ID_MISSION_REQUEST_INT:{
            mavlink_mission_request_int_t mission_request;
            mavlink_msg_mission_request_int_decode(&msg, &mission_request);
            if(mission_request.seq == 1){
                start = 1;
            }else if(mission_request.seq == 0){
                start = 0;
                target_pose.pose.position.x = 0;
                target_pose.pose.position.y = 0;
                target_pose.pose.position.z = 0;
            }
            // ROS_INFO("Mission Request: %d", mission_request.seq);
        }default:
            // ROS_INFO("Received unknown message with ID %d", msg.msgid);
            break;
        
    }
}

