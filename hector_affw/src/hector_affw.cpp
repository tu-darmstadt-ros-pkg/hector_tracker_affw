/*
 * hector_affw.cpp
 *
 *  Created on: 16.05.2016
 *      Author: NicolaiO
 */

#include <affw_msgs/ActionRequest.h>
#include <affw_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>

ros::Publisher pub_set_vel;
ros::Publisher pub_fdbk_state;
ros::ServiceClient srv_action;
ros::Time lastSetVelTime;

nav_msgs::Odometry lastOdom;
bool lastOdomReceived = false;

bool usePose = false;

void setVelCallback(const geometry_msgs::TwistStamped::ConstPtr& vel) {

	lastSetVelTime = ros::Time::now();

	affw_msgs::ActionRequest srv;
	srv.request.state.header = vel->header;
	srv.request.state.vel.push_back(vel->twist.linear.x);
	srv.request.state.vel.push_back(vel->twist.angular.z);

	if(usePose)
	{
		if(!lastOdomReceived)
			return;
		tf::Quaternion q;
		tf::quaternionMsgToTF(lastOdom.pose.pose.orientation, q);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		srv.request.state.custom.push_back(roll);
		srv.request.state.custom.push_back(pitch);
	}

	if (srv_action.call(srv)) {
		geometry_msgs::Twist outVel;
		outVel.linear.x = srv.response.outVel[0];
		outVel.angular.z = srv.response.outVel[1];

		pub_set_vel.publish(outVel);
		ros::spinOnce();
	} else {
		ROS_ERROR_THROTTLE(1, "Failed to get action from affw");
	}
}

void feedbackVelCallback(const nav_msgs::Odometry::ConstPtr& odom) {

	geometry_msgs::Vector3 velIn = odom->twist.twist.linear;
	affw_msgs::State state;
	state.header = odom->header;
	state.header.frame_id = "base_link";
	state.vel.push_back(velIn.x);
	state.vel.push_back(odom->twist.twist.angular.z);

	pub_fdbk_state.publish(state);
	ros::spinOnce();
}

void feedbackOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	lastOdom = *odom;
	lastOdomReceived = true;
}

void timerCallback(const ros::TimerEvent&)
{
	ros::Duration diff = ros::Time::now() - lastSetVelTime;
	if(diff.toSec() > 0.2 && diff.toSec() < 0.4)
	{
		geometry_msgs::Twist outVel;
		pub_set_vel.publish(outVel);
		ros::spinOnce();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hector_affw");
	ros::NodeHandle n;

	lastSetVelTime = ros::Time::now();

	std::string cmd_vel_topic = "/cmd_vel_raw";
	std::string state_topic = "/slam_odom_local";
	std::string odom_topic = "/odom";
	ros::param::get("cmd_vel_topic", cmd_vel_topic);
	ros::param::get("state_topic", state_topic);
	ros::param::get("odom_topic", odom_topic);

	ros::param::get("usePose", usePose);

	// unreliable transport
	ros::TransportHints th;
	th.unreliable();

	// send velocity to robot
	pub_set_vel = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

	// send robot state to affw
	pub_fdbk_state = n.advertise<affw_msgs::State>("/affw_ctrl/state", 1);

	// receive velocity cmd from external source
	ros::Subscriber sub_set_vel = n.subscribe("/affw_ctrl/target_vel", 1,
			setVelCallback, th);

	// receive robot state from robot
	ros::Subscriber sub_fdbk_vel = n.subscribe(state_topic, 1,
			feedbackVelCallback, th);
	ros::Subscriber sub_fdbk_odom = n.subscribe(odom_topic, 1,
			feedbackOdomCallback, th);

	srv_action = n.serviceClient<affw_msgs::ActionRequest>("/affw_ctrl/action");

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

	ros::spin();

	return 0;
}
