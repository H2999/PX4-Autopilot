/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "TurtleControl.hpp"
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

void turtle::Control::loadGeometry()
{
	// Snapshot geometry only when no direction restoration is outstanding.
	if (_recovery.active()) { _reload_geometry = true; return; }

	_reload_geometry = false;
	_geometry = {};
	int32_t count = 0, airframe = -1, reversible = -1;
	param_get(param_find("CA_ROTOR_COUNT"), &count);
	param_get(param_find("CA_AIRFRAME"), &airframe);
	param_get(param_find("CA_R_REV"), &reversible);

	if (count != 4 || airframe != 0 || reversible != 0 || _three_d) { return; }

	unsigned quadrants = 0;
	float z0 = 0.f;

	for (int motor = 0; motor < 4; ++motor) {
		char name[17];
		float x = NAN, y = NAN, z = NAN;
		snprintf(name, sizeof(name), "CA_ROTOR%d_PX", motor); param_get(param_find(name), &x);
		snprintf(name, sizeof(name), "CA_ROTOR%d_PY", motor); param_get(param_find(name), &y);
		snprintf(name, sizeof(name), "CA_ROTOR%d_PZ", motor); param_get(param_find(name), &z);

		if (!PX4_ISFINITE(x) || !PX4_ISFINITE(y) || !PX4_ISFINITE(z) || fabsf(x) < .001f || fabsf(y) < .001f) { return; }

		if (motor == 0) { z0 = z; }

		if (fabsf(z - z0) > .01f) { return; }

		unsigned quadrant = (x > 0.f ? 1 : 0) + (y > 0.f ? 2 : 0);

		if (quadrants & (1u << quadrant)) { return; }

		quadrants |= 1u << quadrant;
		uint32_t physical = 0;

		for (unsigned output = 0; output < _num_outputs; ++output) {
			if (static_cast<int>(_mixing_output.outputFunction(output)) == static_cast<int>(OutputFunction::Motor1) + motor) {
				if (physical != 0 || !(_output_mask & (1u << output))) { return; }

				physical = 1u << output;
			}
		}

		if (physical == 0) { return; }

		_geometry.rotors[motor] = {x, y, physical};
		_geometry.mask |= physical;
	}

	// Do not let this override a fifth motor, servo or an unrecognised DShot load.
	for (unsigned output = 0; output < _num_outputs; ++output) {
		if ((_output_mask & (1u << output)) && _mixing_output.isFunctionSet(output)
		    && !(_geometry.mask & (1u << output))) { return; }
	}

	_geometry.valid = quadrants == 15;
}

turtle::Output turtle::Control::update()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{}; _parameter_update_sub.copy(&update); updateParams(); _reload_geometry = true;
	}

	if (_reload_geometry && !_recovery.active()) { loadGeometry(); }

	vehicle_status_s vehicle{};
	vehicle_attitude_s attitude{};
	vehicle_angular_velocity_s rates{};
	vehicle_acceleration_s acceleration{};
	vehicle_land_detected_s land{};
	manual_control_setpoint_s manual{};
	input_rc_s rc{};
	_vehicle_sub.copy(&vehicle);

	if (!_param_turtle_enable.get() && !_recovery.active()
	    && vehicle.nav_state != vehicle_status_s::NAVIGATION_STATE_TURTLE) {
		// Keep the disabled normal-flight path cheap. Still clear a previous recovery
		// heartbeat so Commander does not retain a stale output-ownership claim.
		turtle::Input idle{};
		idle.now = now;
		const auto out = _recovery.update(idle, _geometry, _config);

		if (now - _last_publish >= 100_ms) {
			turtle_status_s status{};
			status.timestamp = now; status.ready_for_arming = out.ready;
			_status_pub.publish(status);
			_last_publish = now;
		}

		return out;
	}

	_attitude_sub.copy(&attitude);
	_rates_sub.copy(&rates);
	_acceleration_sub.copy(&acceleration);
	_land_sub.copy(&land);
	_manual_sub.copy(&manual);
	_rc_sub.copy(&rc);

	auto fresh = [now](uint64_t stamp, uint64_t age) { return stamp != 0 && stamp <= now && now - stamp < age; };
	const auto &armed = _mixing_output.armed();
	const State state = _recovery.state();
	// Ballistic motion after thrust cutoff must remain unpowered. It is not a
	// reason to abandon landing observation or to start another pulse in the air.
	const bool coasting = state == State::StopNormal || state == State::Normal || state == State::Coast;
	turtle::Input in{};
	in.now = now;
	in.enabled = _param_turtle_enable.get();
	in.requested = vehicle.nav_state == vehicle_status_s::NAVIGATION_STATE_TURTLE;
	in.armed = armed.armed || vehicle.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool quaternion_ok = turtle::gravityBody(attitude.q, in.gravity);
	in.rate = sqrtf(rates.xyz[0] * rates.xyz[0] + rates.xyz[1] * rates.xyz[1] + rates.xyz[2] * rates.xyz[2]);
	bool mapping_ok = _geometry.valid;

	for (int motor = 0; motor < 4; ++motor) {
		for (unsigned output = 0; output < _num_outputs; ++output) {
			if (_geometry.rotors[motor].output & (1u << output)) {
				mapping_ok &= (_output_mask & (1u << output)) &&
					      static_cast<int>(_mixing_output.outputFunction(output)) == static_cast<int>(OutputFunction::Motor1) + motor;
			}
		}
	}

	const float rate_limit = _param_turtle_rate.get();
	const float power = _param_turtle_power.get(), pulse = _param_turtle_pulse.get(), timeout = _param_turtle_timeout.get();
	const float level_pulse = _param_turtle_level_pulse.get();
	const bool config_ok = PX4_ISFINITE(power) && power >= .05f && power <= 1.f &&
			       PX4_ISFINITE(rate_limit) && rate_limit >= .5f && rate_limit <= 6.f &&
			       PX4_ISFINITE(pulse) && pulse >= .1f && pulse <= 6.f &&
			       PX4_ISFINITE(level_pulse) && level_pulse >= .1f && level_pulse <= 6.f && PX4_ISFINITE(timeout) && timeout >= 2.f && timeout <= 20.f;

	if (config_ok) {
		// Accept old stored values up to 1.0, but never output above the new hard cap.
		_config.power = math::min(power, turtle::MAX_POWER);
		_config.rate_limit = rate_limit;
		_config.pulse_us = static_cast<uint64_t>(pulse * 1e6f);
		_config.level_pulse_us = static_cast<uint64_t>(level_pulse * 1e6f);
		_config.timeout_us = static_cast<uint64_t>(timeout * 1e6f);
	}

	// Commander publishes at 2 Hz. Unchanged RC sticks are republished only
	// every 300 ms by rc_update; raw RC below independently enforces live reception.
	in.safe = fresh(vehicle.timestamp, 750_ms) && fresh(armed.timestamp, 750_ms) &&
		  fresh(acceleration.timestamp_sample, 100_ms) && PX4_ISFINITE(acceleration.xyz[0]) &&
		  PX4_ISFINITE(acceleration.xyz[1]) && PX4_ISFINITE(acceleration.xyz[2]) && fresh(attitude.timestamp_sample, 100_ms) && quaternion_ok
		  && fresh(rates.timestamp_sample, 100_ms) && PX4_ISFINITE(in.rate) &&
		  fresh(manual.timestamp_sample, 500_ms) && manual.valid && manual.data_source == manual_control_setpoint_s::SOURCE_RC &&
		  fresh(rc.timestamp_last_signal, 300_ms) && !rc.rc_lost && !rc.rc_failsafe &&
		  PX4_ISFINITE(manual.throttle) && manual.throttle < -.8f &&
		  PX4_ISFINITE(manual.roll) && fabsf(manual.roll) < .15f && PX4_ISFINITE(manual.pitch) && fabsf(manual.pitch) < .15f &&
		  PX4_ISFINITE(manual.yaw) && fabsf(manual.yaw) < .15f &&
		  fresh(land.timestamp, 1500_ms) && (!land.freefall || coasting) &&
		  !armed.kill && !armed.lockdown && !armed.termination && !armed.in_esc_calibration_mode &&
		  !vehicle.calibration_enabled && !vehicle.rc_calibration_in_progress && !vehicle.failsafe &&
		  (!vehicle.safety_button_available || vehicle.safety_off) &&
		  vehicle.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !vehicle.is_vtol &&
		  mapping_ok && config_ok && !_mixing_output.inActuatorTest() && !_three_d && _mixing_output.reversibleOutputs() == 0;

	float acceleration_gravity[3] {};
	const bool acceleration_at_rest = gravityAtRest(acceleration.xyz, acceleration_gravity);

	if (!in.armed && in.safe && acceleration_at_rest && !land.freefall && (land.landed || land.ground_contact || land.at_rest)
	    && in.rate < .15f) {
		if (_ground_since == 0) { _ground_since = now; }

	} else {
		_ground_since = 0;
	}

	in.inverted = acceleration_at_rest && acceleration_gravity[2] < -.1736482f;
	in.ground = _ground_since != 0 && now - _ground_since >= 500_ms;

	// At qualified rest, specific force gives a direct body gravity measurement.
	// Use it to check landing and reselect motors even if EKF tilt is still settling.
	// During a thrust pulse only the attitude estimator is used.
	if (in.ground && state != State::Drive) {
		for (int i = 0; i < 3; ++i) { in.gravity[i] = acceleration_gravity[i]; }
	}

	const turtle::Output out = _recovery.update(in, _geometry, _config);
	turtle_status_s status{};
	status.timestamp = now;
	status.state = static_cast<uint8_t>(_recovery.state());
	status.reason = static_cast<uint8_t>(_recovery.reason());
	status.active = out.own;
	status.ready_for_arming = out.ready;
	status.motor_mask = _recovery.pair();
	status.tilt = quaternion_ok ? acosf(math::constrain(in.gravity[2], -1.f, 1.f)) : NAN;
	status.power = out.power;
	status.levelling = _recovery.levelling();

	if (now - _last_publish >= 50_ms || _recovery.state() != _last_state || _recovery.reason() != _last_reason) {
		_status_pub.publish(status);
		_last_publish = now;
	}

	if (_recovery.state() != _last_state || _recovery.reason() != _last_reason) {
		PX4_INFO("Turtle state=%u reason=%u outputs=0x%lx tilt=%.1f", status.state, status.reason,
			 static_cast<unsigned long>(status.motor_mask), static_cast<double>(math::degrees(status.tilt)));
		_last_state = _recovery.state(); _last_reason = _recovery.reason();
	}

	return out;
}
