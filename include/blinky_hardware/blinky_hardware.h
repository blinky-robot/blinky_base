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

#ifndef _blinky_hardware_h
#define _blinky_hardware_h

#include <ft_scservo_driver/scservo_driver.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <vesc_driver/vesc_driver.hpp>

namespace blinky_hardware
{
	class BlinkyHardware : public hardware_interface::RobotHW
	{
	public:
		BlinkyHardware(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
		~BlinkyHardware();
		void read(ros::Time time, ros::Duration period);
		void write(ros::Time time, ros::Duration period);


	private:
		bool getDriveJointLimits(ros::NodeHandle &nh);
		bool getSteeringJointLimits(ros::NodeHandle &nh);
		bool getJointNames(ros::NodeHandle &nh);
		void trySetServoTorque(bool enable);

		std::string drive_joint_name;
		std::string steering_joint_name;
		bool joint_names_done;

		double drive_act_cmd;
		transmission_interface::ActuatorData drive_act_cmd_data;
		double drive_act_eff;
		double drive_act_pos;
		transmission_interface::ActuatorData drive_act_state_data;
		transmission_interface::ActuatorToJointStateInterface drive_act_to_joint_state;
		double drive_act_vel;
		double drive_cmd;
		transmission_interface::JointData drive_cmd_data;
		double drive_eff;
		hardware_interface::JointStateHandle drive_handle;
		hardware_interface::JointHandle drive_handle_cmd;
		joint_limits_interface::JointLimits drive_joint_limits;
		bool drive_joint_limits_done;
		joint_limits_interface::VelocityJointSaturationHandle drive_joint_limits_handle;
		transmission_interface::JointToActuatorVelocityInterface drive_joint_to_act_vel;
		double drive_pos;
		transmission_interface::JointData drive_state_data;
		transmission_interface::SimpleTransmission drive_trans;
		double drive_vel;

		hardware_interface::PositionJointInterface joint_pos_interface;
		joint_limits_interface::PositionJointSaturationInterface joint_pos_limits_interface;
		hardware_interface::JointStateInterface joint_state_interface;
		hardware_interface::VelocityJointInterface joint_vel_interface;
		joint_limits_interface::VelocityJointSaturationInterface joint_vel_limits_interface;

		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;

		ft_scservo_driver::SCServoBus servo_bus;
		int servo_id;
		bool servo_torque_enabled;

		double steering_cmd;
		double steering_eff;
		hardware_interface::JointStateHandle steering_handle;
		hardware_interface::JointHandle steering_handle_cmd;
		joint_limits_interface::JointLimits steering_joint_limits;
		bool steering_joint_limits_done;
		joint_limits_interface::PositionJointSaturationHandle steering_joint_limits_handle;
		double steering_pos;
		double steering_vel;

		vesc_driver::VESC vesc;
	};
}

#endif /* _blinky_hardware_h */
