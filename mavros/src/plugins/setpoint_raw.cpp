/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WrenchTarget.h>
#include <mavros_msgs/TiltAngleTarget.h>
#include <mavros_msgs/TiltrotorActuatorCommands.h>
#include <mavros_msgs/AttitudeThrustTarget.h>
#include <mavros_msgs/AllocationMatrix.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
	private plugin::SetAttitudeTargetMixin<SetpointRawPlugin>,
	private plugin::SetWrenchTargetMixin<SetpointRawPlugin>,
	private plugin::SetTiltAngleTargetMixin<SetpointRawPlugin>,
	private plugin::SetTiltrotorActuatorCommandsMixin<SetpointRawPlugin>,
	private plugin::SetAttitudeThrustTargetMixin<SetpointRawPlugin>,
	private plugin::SetAllocationMatrixMixin<SetpointRawPlugin> {
public:
	SetpointRawPlugin() : PluginBase(),
		sp_nh("~setpoint_raw")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		ignore_rpyt_messages_ = false;
		if(!sp_nh.getParam("thrust_scaling_factor", thrust_scaling_)){
			ROS_FATAL("No thrust scaling factor found, DO NOT FLY");
			ignore_rpyt_messages_ = true;
		}
		if(!sp_nh.getParam("system_mass_kg", system_mass_kg_)){
			ROS_FATAL("No system mass found, DO NOT FLY");
			ignore_rpyt_messages_ = true;
		}
	    if(!sp_nh.getParam("yaw_rate_scaling_factor", yaw_rate_scaling_)){
	      ROS_FATAL("No yaw rate scaling factor found, DO NOT FLY");
	      ignore_rpyt_messages_ = true;
	    }

		local_sub = sp_nh.subscribe("local", 1, &SetpointRawPlugin::local_cb, this, ros::TransportHints().tcpNoDelay());
		global_sub = sp_nh.subscribe("global", 1, &SetpointRawPlugin::global_cb, this, ros::TransportHints().tcpNoDelay());
		attitude_sub = sp_nh.subscribe("attitude", 1, &SetpointRawPlugin::attitude_cb, this, ros::TransportHints().tcpNoDelay());
		rpyt_sub = sp_nh.subscribe("roll_pitch_yawrate_thrust", 1, &SetpointRawPlugin::rpyt_cb, this, ros::TransportHints().tcpNoDelay());
		wrench_sub = sp_nh.subscribe("wrench", 1, &SetpointRawPlugin::wrench_cb, this, ros::TransportHints().tcpNoDelay());
		attitude_thrust_sub = sp_nh.subscribe("attitude_thrust", 1, &SetpointRawPlugin::attitude_thrust_target_cb, this, ros::TransportHints().tcpNoDelay());
		tiltrotor_actuator_commands_sub = sp_nh.subscribe("tiltrotor_actuator_commands", 1, &SetpointRawPlugin::tiltrotor_actuator_commands_cb, this, ros::TransportHints().tcpNoDelay());
		allocation_matrix_sub = sp_nh.subscribe("allocation_matrix", 10, &SetpointRawPlugin::allocation_matrix_cb, this);
		target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
		target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
		target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&SetpointRawPlugin::handle_position_target_local_ned),
			       make_handler(&SetpointRawPlugin::handle_position_target_global_int),
			       make_handler(&SetpointRawPlugin::handle_attitude_target),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class SetPositionTargetGlobalIntMixin;
	friend class SetAttitudeTargetMixin;
	friend class SetWrenchTargetMixin;
	friend class SetTiltAngleTargetMixin;
	friend class SetTiltrotorActuatorCommandsMixin;
	friend class SetAttitudeThrustTargetMixin;
	friend class SetAllocationMatrixMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber local_sub, global_sub, attitude_sub, rpyt_sub, wrench_sub, tiltrotor_actuator_commands_sub, attitude_thrust_sub, allocation_matrix_sub;
	ros::Publisher target_local_pub, target_global_pub, target_attitude_pub;
	double thrust_scaling_, system_mass_kg_, yaw_rate_scaling_;
	bool ignore_rpyt_messages_;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_LOCAL_NED &tgt)
	{
		// Transform desired position,velocities,and accels from ENU to NED frame
		auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
		Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
		auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
		float yaw_rate = ang_vel_enu.z();

		auto target = boost::make_shared<mavros_msgs::PositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		tf::pointEigenToMsg(position, target->position);
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_local_pub.publish(target);
	}

	void handle_position_target_global_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_GLOBAL_INT &tgt)
	{
		// Transform desired velocities from ENU to NED frame
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
		Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
		auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
		float yaw_rate = ang_vel_enu.z();

		auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		target->latitude = tgt.lat_int / 1e7;
		target->longitude = tgt.lon_int / 1e7;
		target->altitude = tgt.alt;
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_global_pub.publish(target);
	}

	void handle_attitude_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_TARGET &tgt)
	{
		// Transform orientation from baselink -> ENU
		// to aircraft -> NED
		auto orientation = ftf::transform_orientation_ned_enu(
					ftf::transform_orientation_baselink_aircraft(
						Eigen::Quaterniond(tgt.q[0], tgt.q[1], tgt.q[2], tgt.q[3])));

		auto body_rate = ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

		auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->type_mask = tgt.type_mask;
		tf::quaternionEigenToMsg(orientation, target->orientation);
		tf::vectorEigenToMsg(body_rate, target->body_rate);
		target->thrust = tgt.thrust;

		target_attitude_pub.publish(target);
	}

	/* -*- callbacks -*- */

	void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d position, velocity, af;
		float yaw, yaw_rate;

		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		position = ftf::transform_frame_enu_ned(position);
		velocity = ftf::transform_frame_enu_ned(velocity);
		af = ftf::transform_frame_enu_ned(af);
		yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
		Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
		auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
		yaw_rate = ang_vel_ned.z();

		set_position_target_local_ned(
					req->header.stamp.toNSec() / 1000000,
					req->coordinate_frame,
					req->type_mask,
					position,
					velocity,
					af,
					yaw, yaw_rate);
	}

	void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d velocity, af;
		float yaw, yaw_rate;

		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		velocity = ftf::transform_frame_enu_ned(velocity);
		af = ftf::transform_frame_enu_ned(af);
		yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
		Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
		auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
		yaw_rate = ang_vel_ned.z();

		set_position_target_global_int(
					req->header.stamp.toNSec() / 1000000,
					req->coordinate_frame,
					req->type_mask,
					req->latitude * 1e7,
					req->longitude * 1e7,
					req->altitude,
					velocity,
					af,
					yaw, yaw_rate);
	}

	void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req)
	{
		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d baselink_angular_rate;

		tf::quaternionMsgToEigen(req->orientation, desired_orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(desired_orientation));

		auto body_rate = ftf::transform_frame_baselink_aircraft(baselink_angular_rate);

		tf::vectorMsgToEigen(req->body_rate, body_rate);

		set_attitude_target(
					req->header.stamp.toNSec() / 1000000,
					req->type_mask,
					ned_desired_orientation,
					body_rate,
					req->thrust);
	}

    void rpyt_cb(const mav_msgs::RollPitchYawrateThrustConstPtr msg) {
      if (ignore_rpyt_messages_) {
        ROS_FATAL(
            "Recieved roll_pitch_yaw_thrust_rate message, but "
            "ignore_rpyt_messages_ is true: the most likely cause of this "
            "is a failure to specify the thrust_scaling_factor, "
            "yaw_rate_scaling_factor or system_mass_kg parameters");
        return;
      }
      // the masks are much more limited than the docs would suggest so we don't use them
      uint8_t type_mask = 0;
      geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll, msg->pitch, 0);
      double thrust = std::min(1.0, std::max(0.0, msg->thrust.z * thrust_scaling_ * system_mass_kg_));

      Eigen::Quaterniond desired_orientation;
      Eigen::Vector3d body_rate;
      tf::quaternionMsgToEigen(orientation, desired_orientation);

      // Transform desired orientation to represent aircraft->NED,
      // MAVROS operates on orientation of base_link->ENU
      auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
          ftf::transform_orientation_baselink_aircraft(
              desired_orientation));
      body_rate.x() = 0;
      body_rate.y() = 0;
      body_rate.z() = -yaw_rate_scaling_ * msg->yaw_rate;
      set_attitude_target(msg->header.stamp.toNSec() / 1000000, type_mask,
                          ned_desired_orientation, body_rate, thrust);
    }

    void wrench_cb(const mavros_msgs::WrenchTarget::ConstPtr &req)
	{
		Eigen::Vector3d a_lin, a_ang;

		tf::vectorMsgToEigen(req->linear_acceleration, a_lin);
		tf::vectorMsgToEigen(req->angular_acceleration, a_ang);

		// Transform frame ENU->NED
		a_lin = ftf::transform_frame_enu_ned(a_lin);
		a_ang = ftf::transform_frame_enu_ned(a_ang);

		set_wrench_target(a_lin, a_ang);
	}
    void tilt_angle_cb(const mavros_msgs::TiltAngleTarget::ConstPtr &req)
	{
		float alpha[6];
		for (int i=0;i<6;i++) {
			alpha[i] = req->alpha[i];
		}
		set_tilt_angle_target(alpha);
	}
    void tiltrotor_actuator_commands_cb(const mavros_msgs::TiltrotorActuatorCommands::ConstPtr &req)
	{
		float u[18];
		for (int i=0;i<6;i++) {
			u[i] = req->u_tiltangles[i];
		}
		for (int i=6;i<18;i++) {
			u[i] = req->u_rotors[i-6];
		}
		set_tiltrotor_actuator_commands(u);
	}
    void attitude_thrust_target_cb(const mavros_msgs::AttitudeThrustTarget::ConstPtr &req)
	{
		Eigen::Vector3d a_lin, a_ang, rates_sp;

		tf::vectorMsgToEigen(req->linear_acceleration, a_lin);
		tf::vectorMsgToEigen(req->angular_acceleration, a_ang);
		tf::vectorMsgToEigen(req->rates_sp, rates_sp);

		Eigen::Quaterniond desired_orientation;
		tf::quaternionMsgToEigen(req->orientation, desired_orientation);

		auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(desired_orientation));

		// Transform frame ENU->NED
		a_lin = ftf::transform_frame_enu_ned(a_lin);
		a_ang = ftf::transform_frame_enu_ned(a_ang);
		rates_sp = ftf::transform_frame_enu_ned(rates_sp);

		set_attitude_thrust_target(a_lin, a_ang, ned_desired_orientation, rates_sp);
	}
    void allocation_matrix_cb(const mavros_msgs::AllocationMatrix::ConstPtr &req)
	{
		Eigen::VectorXd alloc_matrix(36);
		Eigen::VectorXd tilt_angles(6);
		for (int i=0;i<36;i++) {
			alloc_matrix(i) = req->allocation_matrix[i];
		}
		for (int i=0;i<6;i++) {
			tilt_angles(i) = req->tilt_angles[i];
		}
		set_allocation_matrix(alloc_matrix, tilt_angles);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointRawPlugin, mavros::plugin::PluginBase)
