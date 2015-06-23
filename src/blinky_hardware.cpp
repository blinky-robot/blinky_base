/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "blinky_hardware/blinky_hardware.h"

#include <joint_limits_interface/joint_limits_rosparam.h>

namespace blinky_hardware
{
	BlinkyHardware::BlinkyHardware(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
		: drive_joint_name("wheels"),
		  steering_joint_name("steering_angle"),
		  front_left_wheel_joint_name("front_left_wheel_joint"),
		  front_right_wheel_joint_name("front_right_wheel_joint"),
		  rear_left_wheel_joint_name("rear_left_wheel_joint"),
		  rear_right_wheel_joint_name("rear_right_wheel_joint"),
		  front_left_steering_joint_name("front_left_steering_joint"),
		  front_right_steering_joint_name("front_right_steering_joint"),
		  joint_names_done(getJointNames(nh_priv)),
		  drive_act_cmd(0.0),
		  drive_act_eff(0.0),
		  drive_act_pos(0.0),
		  drive_act_vel(0.0),
		  drive_cmd(0.0),
		  drive_eff(0.0),
		  drive_handle(drive_joint_name, &drive_pos, &drive_vel, &drive_eff),
		  drive_handle_cmd(drive_handle, &drive_cmd),
		  drive_joint_limits_done(getDriveJointLimits(nh_priv)),
		  drive_joint_limits_handle(drive_handle_cmd, drive_joint_limits),
		  drive_pos(0.0),
		  drive_trans(12, 0.0),
		  drive_vel(0.0),
		  wheel_front_left_handle(front_left_wheel_joint_name, &drive_pos, &drive_vel, &drive_eff),
		  wheel_front_right_handle(front_right_wheel_joint_name, &drive_pos, &drive_vel, &drive_eff),
		  wheel_rear_left_handle(rear_left_wheel_joint_name, &drive_pos, &drive_vel, &drive_eff),
		  wheel_rear_right_handle(rear_right_wheel_joint_name, &drive_pos, &drive_vel, &drive_eff),
		  nh(nh),
		  nh_priv(nh_priv),
		  servo_bus(nh, ros::NodeHandle(nh_priv, "scservo_driver")),
		  servo_id(0),
		  servo_torque_enabled(false),
		  steering_act_cmd(0.0),
		  steering_act_eff(0.0),
		  steering_act_pos(0.0),
		  steering_act_vel(0.0),
		  steering_cmd(0.0),
		  steering_eff(0.0),
		  steering_handle(steering_joint_name, &steering_pos, &steering_vel, &steering_eff),
		  steering_handle_cmd(steering_handle, &steering_cmd),
		  steering_joint_limits_done(getSteeringJointLimits(nh_priv)),
		  steering_joint_limits_handle(steering_handle_cmd, steering_joint_limits),
		  steering_pos(0.0),
		  steering_trans(-2.35, 0.0),
		  steering_vel(0.0),
		  steering_servo_arm_handle(steering_servo_arm_joint_name, &steering_pos, &steering_vel, &steering_eff),
		  wheel_front_left_steering_handle(front_left_steering_joint_name, &steering_pos, &steering_vel, &steering_eff),
		  wheel_front_right_steering_handle(front_right_steering_joint_name, &steering_pos, &steering_vel, &steering_eff),
		  vesc(nh, ros::NodeHandle(nh_priv, "vesc_driver"))
	{
		servo_bus.start();
		vesc.start();

		nh_priv.param("servo_id", servo_id, servo_id);
		if (!joint_limits_interface::getJointLimits(steering_joint_name, nh_priv, steering_joint_limits))
		{
			ROS_WARN("Failed to populate joint limits for steering_joint");
		}
		if (!joint_limits_interface::getJointLimits(drive_joint_name, nh_priv, drive_joint_limits))
		{
			ROS_WARN("Failed to populate joint limits for drive_joint");
		}

		// Joint Interfaces
		joint_state_interface.registerHandle(drive_handle);
		joint_state_interface.registerHandle(wheel_front_left_handle);
		joint_state_interface.registerHandle(wheel_front_right_handle);
		joint_state_interface.registerHandle(wheel_rear_left_handle);
		joint_state_interface.registerHandle(wheel_rear_right_handle);
		joint_state_interface.registerHandle(wheel_front_left_steering_handle);
		joint_state_interface.registerHandle(wheel_front_right_steering_handle);
		joint_state_interface.registerHandle(steering_handle);
		joint_state_interface.registerHandle(steering_servo_arm_handle);
		registerInterface(&joint_state_interface);

		joint_vel_interface.registerHandle(drive_handle_cmd);
		registerInterface(&joint_vel_interface);

		joint_pos_interface.registerHandle(steering_handle_cmd);
		registerInterface(&joint_pos_interface);

		// Transmission Interfaces
		drive_act_state_data.effort.push_back(&drive_act_eff);
		drive_act_state_data.position.push_back(&drive_act_pos);
		drive_act_state_data.velocity.push_back(&drive_act_vel);
		steering_act_state_data.effort.push_back(&steering_act_eff);
		steering_act_state_data.position.push_back(&steering_act_pos);
		steering_act_state_data.velocity.push_back(&steering_act_vel);

		drive_state_data.effort.push_back(&drive_eff);
		drive_state_data.position.push_back(&drive_pos);
		drive_state_data.velocity.push_back(&drive_vel);
		steering_state_data.effort.push_back(&steering_eff);
		steering_state_data.position.push_back(&steering_pos);
		steering_state_data.velocity.push_back(&steering_vel);

		drive_act_cmd_data.velocity.push_back(&drive_act_cmd);
		drive_cmd_data.velocity.push_back(&drive_cmd);
		steering_act_cmd_data.velocity.push_back(&steering_act_cmd);
		steering_cmd_data.velocity.push_back(&steering_cmd);

		drive_act_to_joint_state.registerHandle(transmission_interface::ActuatorToJointStateHandle("drive_trans", &drive_trans, drive_act_state_data, drive_state_data));
		drive_joint_to_act_vel.registerHandle(transmission_interface::JointToActuatorVelocityHandle("drive_trans", &drive_trans, drive_act_cmd_data, drive_cmd_data));
		steering_act_to_joint_state.registerHandle(transmission_interface::ActuatorToJointStateHandle("steering_trans", &steering_trans, steering_act_state_data, steering_state_data));
		steering_joint_to_act_vel.registerHandle(transmission_interface::JointToActuatorVelocityHandle("steering_trans", &steering_trans, steering_act_cmd_data, steering_cmd_data));

		// Joint Limits
		joint_pos_limits_interface.registerHandle(steering_joint_limits_handle);
		joint_vel_limits_interface.registerHandle(drive_joint_limits_handle);

		trySetServoTorque(true);
	}

	BlinkyHardware::~BlinkyHardware()
	{
		trySetServoTorque(false);
		vesc.stop();
		servo_bus.stop();
	}

	void BlinkyHardware::read(ros::Time time, ros::Duration period)
	{
		try
		{
			servo_bus.getStatus(servo_id, steering_act_vel, steering_act_pos, steering_act_eff);
		}
		catch (ft_scservo_driver::Exception &e)
		{
			ROS_WARN_THROTTLE(1, "Failed to get servo status: %s", e.what());
		}

		try
		{
			vesc.getStatus(drive_act_vel, drive_act_pos);
		}
		catch (vesc_driver::Exception &e)
		{
			ROS_WARN_THROTTLE(1, "Failed to get VESC status: %s", e.what());
		}

		drive_act_to_joint_state.propagate();
		steering_act_to_joint_state.propagate();
	}

	void BlinkyHardware::write(ros::Time time, ros::Duration period)
	{
		drive_joint_to_act_vel.propagate();
		steering_joint_to_act_vel.propagate();

		joint_pos_limits_interface.enforceLimits(period);
		joint_vel_limits_interface.enforceLimits(period);

		if (!servo_torque_enabled)
		{
			trySetServoTorque(true);
		}

		try
		{
			servo_bus.setPosition(servo_id, steering_act_cmd);
		}
		catch (ft_scservo_driver::Exception &e)
		{
			ROS_WARN_THROTTLE(1, "Failed to set servo position: %s", e.what());
		}

		try
		{
			vesc.setVelocity(drive_act_cmd);
		}
		catch (vesc_driver::Exception &e)
		{
			ROS_WARN_THROTTLE(1, "Failed to set VESC velocity: %s", e.what());
		}
	}

	bool BlinkyHardware::getDriveJointLimits(ros::NodeHandle &nh_priv)
	{
		if (!joint_limits_interface::getJointLimits(drive_joint_name, nh_priv, drive_joint_limits))
		{
			ROS_WARN("Failed to populate joint limits for drive_joint");
			return false;
		}

		return true;
	}

	bool BlinkyHardware::getSteeringJointLimits(ros::NodeHandle &nh_priv)
	{
		if (!joint_limits_interface::getJointLimits(steering_joint_name, nh_priv, steering_joint_limits))
		{
			ROS_WARN("Failed to populate joint limits for steering_joint");
			return false;
		}

		return true;
	}

	bool BlinkyHardware::getJointNames(ros::NodeHandle &nh)
	{
		nh.param("drive_joint_name", drive_joint_name, drive_joint_name);
		nh.param("steering_joint_name", steering_joint_name, steering_joint_name);
		nh.param("steering_servo_arm_joint_name", steering_servo_arm_joint_name, steering_servo_arm_joint_name);
		nh.param("front_left_steering_joint_name", front_left_steering_joint_name, front_left_steering_joint_name);
		nh.param("front_right_steering_joint_name", front_right_steering_joint_name, front_right_steering_joint_name);
		nh.param("front_left_wheel_joint_name", front_left_wheel_joint_name, front_left_wheel_joint_name);
		nh.param("front_right_wheel_joint_name", front_right_wheel_joint_name, front_right_wheel_joint_name);
		nh.param("rear_left_wheel_joint_name", rear_left_wheel_joint_name, rear_left_wheel_joint_name);
		nh.param("rear_right_wheel_joint_name", rear_right_wheel_joint_name, rear_right_wheel_joint_name);

		return true;
	}

	void BlinkyHardware::trySetServoTorque(bool enable)
	{
		try
		{
			servo_bus.setEnableTorque(servo_id, enable);
			servo_torque_enabled = enable;
		}
		catch (ft_scservo_driver::Exception &e)
		{
			ROS_WARN_THROTTLE(1, "Failed to %s servo torque", enable ? "enable" : "disable");
		}
	}
}
