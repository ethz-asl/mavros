/**
 * @brief ViconPoseEstimate plugin
 * @file vicon_pose_estimate.cpp
 * @author D. Hentzen <hentzend@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 D. Hentzen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define RAD_TO_DEG_D 180.0/3.1415926

namespace mavros {
namespace extra_plugins{
/**
 * @brief Vicon pose estimate plugin
 *
 * Send pose estimation from vicon motion capture system
 * to FCU position and attitude estimators.
 *
 */
class ViconPoseEstimatePlugin : public plugin::PluginBase
{
public:
	ViconPoseEstimatePlugin() : PluginBase(),
		sp_nh("~vicon_pose")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		vicon_sub = sp_nh.subscribe("vicon_tf", 10, &ViconPoseEstimatePlugin::vicon_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;

	ros::Subscriber vicon_sub;

	ros::Time last_transform_stamp;

	/* -*- low-level send -*- */
	/**
	 * @brief Send vicon estimate transform to FCU position controller
	 */
	void send_vicon_estimate(const ros::Time &stamp, const Eigen::Affine3d &tr, const geometry_msgs::PoseWithCovariance::_covariance_type &cov)
	{
		/**
		 * @warning Issue #60.
		 * This now affects pose callbacks too.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vicon_pose", "Vicon: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto rpy = ftf::quaternion_to_rpy(
				ftf::transform_orientation_enu_ned(
				ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))));

		auto cov_ned = ftf::transform_frame_enu_ned(cov);
		ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());

		auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
		ROS_DEBUG_STREAM_NAMED("vicon_pose", "Vicon: Covariance URT: " << std::endl << urt_view);

		mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};

		vp.usec = stamp.toNSec() / 1000;
		// [[[cog:
		// for f in "xyz":
		//     cog.outl("vp.%s = position.%s();" % (f, f))
		// for a, b in zip("xyz", ('roll', 'pitch', 'yaw')):
		//     cog.outl("vp.%s = rpy.%s();" % (b, a))
		// ]]]
		vp.x = position.x();
		vp.y = position.y();
		vp.z = position.z();
		vp.roll = rpy.x();
		vp.pitch = rpy.y();
		vp.yaw = rpy.z();
		// [[[end]]] (checksum: 2048daf411780847e77f08fe5a0b9dd3)
		// printf("Vicon estimate roll pitch yaw is: %f, %f, %f \n", vp.roll*RAD_TO_DEG_D, -vp.pitch*RAD_TO_DEG_D, 90 - vp.yaw*RAD_TO_DEG_D);
		// just the URT of the 6x6 Pose Covariance Matrix, given
		// that the matrix is symmetric
		ftf::covariance_urt_to_mavlink(cov_map, vp.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(vp);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void vicon_cb(const geometry_msgs::TransformStamped &transform)
	{
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);
		ftf::Covariance6d cov {};	// zero initialized, no covariance info for mocap

		send_vicon_estimate(transform.header.stamp, tr, cov);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ViconPoseEstimatePlugin, mavros::plugin::PluginBase)
