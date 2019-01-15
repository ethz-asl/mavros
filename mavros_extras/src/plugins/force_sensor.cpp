/**
 * @brief Force Sensor plugin
 * @file force_sensor.cpp
 * @author Christoph Tobler <toblech@ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Karen Bodie	<karen.bodie@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Christoph Tobler.
 * Copyright 2017 Nuno Marques.
 * Copyright 2019 Karen Bodie.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief force_sensor plugin.
 * Sends force sensor data to FCU.
 */
class ForceSensorPlugin : public plugin::PluginBase {
public:
	ForceSensorPlugin() : PluginBase(),
		nh("~force_sensor")
	{ }

	void initialize(UAS &uas_)
	{
		// PluginBase::initialize(uas_);
		//
		// force_sensor_sub = nh.subscribe("force_sensor", 10, &ForceSensorPlugin::force_sensor_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber force_sensor_sub;

	/**
	 * @brief Send force sensor data through Force Sensor Mavlink msg
	 */
	void force_sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &data)
	{
		// mavlink::common::msg::FORCE_SENSOR force_msg {};
		//
		// // Fill in and send message
		// force_msg.time_usec = data->header.stamp.toNSec() / 1000;	// [useconds]
		// force_msg.force_x = data->wrench.force.x;		// [N]
		// force_msg.force_y = data->wrench.force.y;		// [N]
		// force_msg.force_z = data->wrench.force.z;		// [N]
		// force_msg.torque_x = data->wrench.torque.x;	// [Nm]
		// force_msg.torque_y = data->wrench.torque.y;	// [Nm]
		// force_msg.torque_z = data->wrench.torque.z;	// [Nm]
		//
		// UAS_FCU(m_uas)->send_message_ignore_drop(force_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ForceSensorPlugin, mavros::plugin::PluginBase)
