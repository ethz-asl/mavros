/**
 * @brief dynamixel data parser plugin
 * @file dynamixel.cpp
 * @author Weixuan Zhang <zhangweixuanpeter@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2020 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */


#include <cmath>
#include <ros/console.h>
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/DynamixelStatus.h>


namespace mavros {
namespace std_plugins {
//! @brief dynamixel publication plugin
class DynamixelPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DynamixelPlugin() : PluginBase(),
		dynamixel_nh("~dynamixel")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		dynamixel_pub = dynamixel_nh.advertise<mavros_msgs::DynamixelStatus>("dynamixel_status",10);

		// Reset has_* flags on connection change
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() {
		return {
			       make_handler(&DynamixelPlugin::handle_dynamixel),
		};
	}

private:
	ros::NodeHandle dynamixel_nh;
	std::string frame_id;

	ros::Publisher dynamixel_pub;

	/**
	 * @brief Handle DYNAMIXEL_STATUS MAVlink message.
	 * @param msg		Received Mavlink msg
	 * @param dynamixel		DYNAMIXEL_STATUS msg
	 */
	void handle_dynamixel(const mavlink::mavlink_message_t *msg, mavlink::omav::msg::DYNAMIXEL_STATUS &dyn_s)
	{
		auto dynamixel_status = boost::make_shared<mavros_msgs::DynamixelStatus>();
		dynamixel_status->header.stamp = m_uas->synchronise_stamp(dyn_s.time_boot_us);
		dynamixel_status->msg_arrival_time = ros::Time::now();
		for (int i; i<6; i++)
		{
			dynamixel_status->measured_angles[i]  = dyn_s.anglesMeasured[i];
			dynamixel_status->cmd_angles[i]  = dyn_s.anglesSet[i];
		}
		dynamixel_status->noutputs = dyn_s.noutputs;
		
		dynamixel_pub.publish(dynamixel_status);
		
	}


};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::DynamixelPlugin, mavros::plugin::PluginBase)
