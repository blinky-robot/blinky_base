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

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <ft_scservo_driver/ft_scservo_handle.hpp>
#include <hardware_interface/robot_hw.h>
#include <vesc_driver/vesc_handle.hpp>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>

namespace blinky_hardware
{
	class BlinkyHardware : public hardware_interface::RobotHW
	{
	public:
		BlinkyHardware(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
		~BlinkyHardware();
		void read(ros::Time time, ros::Duration period);
		void update(ros::Time time, ros::Duration period);
		void write(ros::Time time, ros::Duration period);


	private:
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		std::string robot_description;
		diagnostic_updater::Updater diag;
		diagnostic_updater::FrequencyStatus diag_freq;
		double diag_freq_min;
		double diag_freq_max;

		transmission_interface::RobotTransmissions transmissions;
		boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader;

		hardware_interface::PositionActuatorInterface actuator_position_interface;
		hardware_interface::ActuatorStateInterface actuator_state_interface;
		hardware_interface::VelocityActuatorInterface actuator_velocity_interface;

		ft_scservo_driver::SCServoHandle servo_handle;
		vesc_driver::VESCHandle vesc_handle;
	};
}

#endif /* _blinky_hardware_h */
