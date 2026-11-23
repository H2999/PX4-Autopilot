/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once
#include "TurtleRecovery.hpp"
#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/turtle_status.h>

namespace turtle
{
// Identical sensor validation, geometry, state machine and arming heartbeat for
// hardware DShot and Gazebo. Only the final ESC transport differs.
class Control : public ModuleParams
{
public:
	Control(MixingOutput &mixing, unsigned outputs) : ModuleParams(nullptr), _mixing_output(mixing), _num_outputs(outputs) { updateParams(); }
	Output update(uint32_t available_outputs, bool three_d = false)
	{
		if (_output_mask != available_outputs || _three_d != three_d) { _reload_geometry = true; }

		_output_mask = available_outputs; _three_d = three_d;
		return update();
	}
	void mappingChanged() { _reload_geometry = true; }
	bool active() const { return _recovery.active(); }
	bool enabled() const { return _param_turtle_enable.get(); }
	void frameSent() { _recovery.frameSent(); }
private:
	void loadGeometry();
	Output update();
	MixingOutput &_mixing_output;
	const unsigned _num_outputs;
	uint32_t _output_mask{};
	bool _three_d{}, _reload_geometry{true};
	Recovery _recovery{};
	Geometry _geometry{};
	Config _config{};
	hrt_abstime _ground_since{}, _last_publish{};
	State _last_state{State::Idle};
	Reason _last_reason{Reason::None};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _rc_sub{ORB_ID(input_rc)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 100_ms};
	uORB::Publication<turtle_status_s> _status_pub{ORB_ID(turtle_status)};
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::TRTL_EN>) _param_turtle_enable,
		(ParamFloat<px4::params::TRTL_PWR>) _param_turtle_power,
		(ParamFloat<px4::params::TRTL_RATE>) _param_turtle_rate,
		(ParamFloat<px4::params::TRTL_PULSE>) _param_turtle_pulse,
		(ParamFloat<px4::params::TRTL_LPULSE>) _param_turtle_level_pulse,
		(ParamFloat<px4::params::TRTL_TOUT>) _param_turtle_timeout
	)
};
}
