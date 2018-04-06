/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  double rate_Hz = 20.0;
  ros::Rate rate(rate_Hz);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("connected");

  ////////////////////////////////////////////
  ///////////////////ARM//////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(arming_cl.call(srv))
  {
    ROS_INFO("ARM send ok %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed arming or disarming");
  }




  geometry_msgs::PoseStamped pose1;
  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  pose1.pose.position.z = 1;

  geometry_msgs::PoseStamped pose2;
  pose2.pose.position.x = 0;
  pose2.pose.position.y = 0;
  pose2.pose.position.z = 1;

  geometry_msgs::PoseStamped pose3;
  pose3.pose.position.x = 0;
  pose3.pose.position.y = 0;
  pose3.pose.position.z = 0.06;

  geometry_msgs::PoseStamped pose4;
  pose4.pose.position.x = 0;
  pose4.pose.position.y = 0;
  pose4.pose.position.z = 1;
  ////send a few setpoints before starting
  //for(int i = 100; ros::ok() && i > 0; --i)
  //{
  //  local_pos_pub.publish(pose);
  //  ros::spinOnce();
  //  rate.sleep();
  //}
  //ROS_INFO("point set");

  mavros_msgs::SetMode set_mode_msg;
  set_mode_msg.request.custom_mode = "OFFBOARD";



  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now() - ros::Duration(5.0);

  bool request_send = false;
  int cnt = 0;
  int total = rate_Hz*40;
  while(cnt < total)
  {
    ros::spinOnce();
    request_send = false;

    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if( set_mode_client.call(set_mode_msg) &&
          set_mode_msg.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      request_send = true;
    }

    if( !current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if( arming_client.call(arm_cmd) &&
          arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed");
      }
      request_send = true;
    }

    if(request_send)
    {
      last_request = ros::Time::now();
    }


    if(cnt<total/4) {
        local_pos_pub.publish(pose1);
    }else if(cnt<total*2/4) {
        local_pos_pub.publish(pose2);
    }else if(cnt<total*3/4) {
        local_pos_pub.publish(pose3);
    }else {
        local_pos_pub.publish(pose4);
    }
    ROS_INFO("point published");



    rate.sleep();
    cnt++;
  }

  ROS_INFO("land...");
  ////////////////////////////////////////////
  ///////////////////LAND/////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  srv_land.request.altitude = 0;
  srv_land.request.latitude = 0;
  srv_land.request.longitude = 0;
  srv_land.request.min_pitch = 0;
  srv_land.request.yaw = 0;
  if(land_cl.call(srv_land))
  {
    ROS_INFO("srv_land send ok %d", srv_land.response.success);
  }
  else
  {
    ROS_ERROR("Failed Land");
  }

  return 0;
}
