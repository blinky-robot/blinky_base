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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

namespace blinky_hardware
{
	BlinkyHardware::BlinkyHardware(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
		: nh(nh),
		  nh_priv(nh_priv),
		  robot_description(""),
		  diag(nh, nh_priv),
		  diag_freq(diagnostic_updater::FrequencyStatusParam(&diag_freq_min, &diag_freq_max, 0.1, 10)),
		  diag_freq_min(400.0 / 9.0),
		  diag_freq_max(600.0 / 11.0),
		  servo_handle(nh, ros::NodeHandle(nh_priv, "scservo_driver")),
		  vesc_handle(nh, ros::NodeHandle(nh_priv, "vesc_driver"))
	{
		nh.param("robot_description", robot_description, robot_description);

		diag.setHardwareID("Blinky Base Hardware Interface");
		diag.add(diag_freq);

		servo_handle.registerPositionHandles(actuator_position_interface);
		servo_handle.registerStateHandles(actuator_state_interface);
		servo_handle.registerVelocityHandles(actuator_velocity_interface);

		vesc_handle.registerStateHandle(actuator_state_interface);
		vesc_handle.registerVelocityHandle(actuator_velocity_interface);

		registerInterface(&actuator_position_interface);
		registerInterface(&actuator_state_interface);
		registerInterface(&actuator_velocity_interface);

		transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &transmissions));
		transmission_loader->load(robot_description);
	}

	BlinkyHardware::~BlinkyHardware()
	{
	}

	void BlinkyHardware::read(ros::Time time, ros::Duration period)
	{
		servo_handle.read();
		vesc_handle.read();

		if(transmissions.get<transmission_interface::ActuatorToJointStateInterface>())
			transmissions.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
	}

	void BlinkyHardware::update(ros::Time time, ros::Duration period)
	{
		diag_freq.tick();
		diag.update();
	}

	void BlinkyHardware::write(ros::Time time, ros::Duration period)
	{
		if(transmissions.get<transmission_interface::JointToActuatorPositionInterface>())
			transmissions.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
		if(transmissions.get<transmission_interface::JointToActuatorVelocityInterface>())
			transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();

		vesc_handle.write();
		servo_handle.write();
	}
}
