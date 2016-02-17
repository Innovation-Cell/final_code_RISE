/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <localization/vn100_msg.h>

float x_pre = 0, y_pre = 0, z_pre = 0, vx = 0, vy = 0, vz = 0, yaw_pre = 0;
float wx = 0, wy = 0, wz = 0;

void pre_odom_Callback(const nav_msgs::Odometry::ConstPtr& info)
{
	x_pre = info->pose.pose.position.x;
	y_pre = info->pose.pose.position.y;
	z_pre = info->pose.pose.position.z;
	
	vx = info->twist.twist.linear.x;
	vy = info->twist.twist.linear.y;
	vz = info->twist.twist.linear.z;
	
	wx = info->twist.twist.angular.x;
	wy = info->twist.twist.angular.y;
	wz = info->twist.twist.angular.z;
}

void yaw_Callback(const localization::vn100_msg::ConstPtr& info)
{
	yaw_pre = (info->linear.theta)*(3.1412/180);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  //cout << "Hello" << endl;

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber pre_odomMonitor = n.subscribe("/pre_odom", 1, pre_odom_Callback);
  ros::Subscriber vn100Monitor = n.subscribe("/vn100_monitor", 1, yaw_Callback);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(50);
  while(n.ok()){
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    //double dt = (current_time - last_time).toSec();
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    //x += delta_x;
    //y += delta_y;
    //th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_pre);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom_combined";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_pre;
    odom_trans.transform.translation.y = y_pre;
    odom_trans.transform.translation.z = z_pre;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom_combined";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x_pre;
    odom.pose.pose.position.y = y_pre;
    odom.pose.pose.position.z = z_pre;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
    ros::spinOnce();
  }
}
