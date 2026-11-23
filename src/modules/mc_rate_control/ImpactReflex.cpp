/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ImpactReflex.hpp"

#include <float.h>
#include <math.h>
#include <lib/mathlib/math/Functions.hpp>

using namespace matrix;

void ImpactReflex::setConfig(const Config &config)
{
	const Mode prev_mode = _cfg.mode;
	_cfg = config;

	// Sanitize parameter bounds defensively
	_cfg.acc_threshold_mps2 = math::constrain(_cfg.acc_threshold_mps2, 15.0f, 200.0f);
	_cfg.debounce_s = math::constrain(_cfg.debounce_s, 0.0f, 0.03f);
	_cfg.idle_thrust = math::constrain(_cfg.idle_thrust, 0.0f, 0.25f);
	_cfg.drop_time_60_s = math::constrain(_cfg.drop_time_60_s, 0.01f, 0.30f);
	_cfg.drop_time_120_s = math::constrain(_cfg.drop_time_120_s, 0.02f, 0.35f);
	_cfg.recover_thrust = math::constrain(_cfg.recover_thrust, 0.15f, 0.85f);
	_cfg.recover_time_s = math::constrain(_cfg.recover_time_s, 0.05f, 1.0f);
	_cfg.recover_pitch_rad = math::constrain(_cfg.recover_pitch_rad, -0.35f, 0.60f);
	_cfg.recover_roll_rad = math::constrain(_cfg.recover_roll_rad, -0.60f, 0.60f);
	_cfg.recover_tilt_p = math::constrain(_cfg.recover_tilt_p, 2.0f, 16.0f);
	_cfg.recover_rate_max_rads = math::constrain(_cfg.recover_rate_max_rads, 1.5f, 18.0f);

	// Explicitly resetting mode to Disabled clears the one-shot Done latch so the
	// companion computer can re-arm the reflex for the next attack run (0 -> 1/2).
	if (_cfg.mode == Mode::Disabled) {
		reset();

	} else if (prev_mode == Mode::Disabled && _state == State::Done) {
		_state = State::Idle;
	}
}

void ImpactReflex::reset()
{
	_state = State::Idle;
	_debounce_timer_s = 0.0f;
	_phase_timer_s = 0.0f;
	_yaw_hold_rad = 0.0f;
	_yaw_hold_valid = false;
}

bool ImpactReflex::isImpactDetected(const Vector3f &body_accel_frd) const
{
	if (!body_accel_frd.isAllFinite()) {
		return false;
	}

	// Quadrotor propellers only produce thrust along body -Z. Any large specific
	// force in the body XY plane (frontal/lateral collision) or downward +Z push
	// (top guard hitting a 120 deg overhang/ceiling) indicates external contact.
	const float accel_xy = sqrtf(body_accel_frd(0) * body_accel_frd(0) +
				     body_accel_frd(1) * body_accel_frd(1));
	const float ceiling_push_z = body_accel_frd(2);

	return (accel_xy >= _cfg.acc_threshold_mps2) ||
	       (ceiling_push_z >= 0.6f * _cfg.acc_threshold_mps2);
}

Vector3f ImpactReflex::computeCatchRates(const Quatf &q_curr) const
{
	Quatf q = q_curr;
	const float norm = q.norm();

	if (!q.isAllFinite() || norm < 1e-4f) {
		return Vector3f{0.f, 0.f, 0.f};
	}

	q /= norm;

	// Desired catch attitude keeps the heading captured at impact entry while
	// commanding a gentle nose-up retreat (+pitch in FRD) and optional lateral
	// roll-out to clear rotating Outpost blades.
	const float yaw_sp = _yaw_hold_valid ? _yaw_hold_rad : Eulerf(q).psi();
	Quatf qd(Eulerf(_cfg.recover_roll_rad, _cfg.recover_pitch_rad, yaw_sp));
	qd.normalize();

	// Tilt-prioritized SO(3) attitude error (same geometric structure as PX4
	// AttitudeControl) so roll/pitch leveling is never starved by yaw error.
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		qd_red = qd;

	} else {
		qd_red *= q;
	}

	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	static constexpr float kYawWeight = 0.2f;
	const Quatf qd_weighted = qd_red * Quatf(cosf(kYawWeight * acosf(qd_dyaw(0))), 0.f, 0.f,
				  sinf(kYawWeight * asinf(qd_dyaw(3))));

	const Quatf q_err = q.inversed() * qd_weighted;
	const Vector3f eq = 2.f * q_err.canonical().imag();

	Vector3f rate_sp;
	rate_sp(0) = math::constrain(eq(0) * _cfg.recover_tilt_p, -_cfg.recover_rate_max_rads, _cfg.recover_rate_max_rads);
	rate_sp(1) = math::constrain(eq(1) * _cfg.recover_tilt_p, -_cfg.recover_rate_max_rads, _cfg.recover_rate_max_rads);
	rate_sp(2) = math::constrain(eq(2) * (_cfg.recover_tilt_p * 0.4f), -0.5f * _cfg.recover_rate_max_rads,
				     0.5f * _cfg.recover_rate_max_rads);
	return rate_sp;
}

ImpactReflex::Output ImpactReflex::update(const Input &in)
{
	Output out{};
	const float dt = math::constrain(in.dt_s, 0.0f, 0.02f);

	// Hard safety gate: any non-Offboard mode (e.g. Manual/Stabilized takeover),
	// disarm/kill switch, landing, or Disabled parameter immediately yields control.
	if (_cfg.mode == Mode::Disabled || !in.offboard_active || !in.armed || !in.flying) {
		reset();
		out.state = _state;
		return out;
	}

	switch (_state) {
	case State::Idle:
		_debounce_timer_s = 0.0f;
		_phase_timer_s = 0.0f;
		_state = State::Armed;
		break;

	case State::Armed:
		if (in.accel_valid && isImpactDetected(in.body_accel_frd)) {
			_debounce_timer_s += dt;

			if (_debounce_timer_s >= _cfg.debounce_s) {
				_state = State::DetachDrop;
				_phase_timer_s = 0.0f;
				_debounce_timer_s = 0.0f;

				if (in.attitude_valid && in.attitude_q.isAllFinite() && in.attitude_q.norm() > 1e-4f) {
					_yaw_hold_rad = Eulerf(in.attitude_q.normalized()).psi();
					_yaw_hold_valid = true;

				} else {
					_yaw_hold_valid = false;
				}

				// First tick of DetachDrop: immediately clear rate PID integrator windup
				// accumulated during wall contact, drop thrust to idle, and damp body rates.
				out.override_setpoints = true;
				out.reset_integral = true;
				out.rates_sp_rads = Vector3f{0.f, 0.f, 0.f};
				out.thrust_sp_body = Vector3f{0.f, 0.f, -_cfg.idle_thrust};
			}

		} else {
			_debounce_timer_s = 0.0f;
		}

		break;

	case State::DetachDrop: {
			_phase_timer_s += dt;
			const float drop_duration = (_cfg.mode == Mode::Overhang120) ? _cfg.drop_time_120_s : _cfg.drop_time_60_s;

			if (_phase_timer_s >= drop_duration) {
				_state = State::CatchRecover;
				_phase_timer_s = 0.0f;
				out.override_setpoints = true;
				out.rates_sp_rads = in.attitude_valid ? computeCatchRates(in.attitude_q) : Vector3f{0.f, 0.f, 0.f};
				out.thrust_sp_body = Vector3f{0.f, 0.f, -_cfg.recover_thrust};

			} else {
				out.override_setpoints = true;
				out.rates_sp_rads = Vector3f{0.f, 0.f, 0.f};
				out.thrust_sp_body = Vector3f{0.f, 0.f, -_cfg.idle_thrust};
			}

			break;
		}

	case State::CatchRecover:
		_phase_timer_s += dt;

		if (_phase_timer_s >= _cfg.recover_time_s) {
			// One-shot latch: transition to Done and hand control cleanly back to Offboard.
			// Companion must cycle MC_IMP_MODE (0 -> 1/2) before next attack to re-arm.
			_state = State::Done;
			_phase_timer_s = 0.0f;
			out.override_setpoints = false;

		} else {
			out.override_setpoints = true;
			out.rates_sp_rads = in.attitude_valid ? computeCatchRates(in.attitude_q) : Vector3f{0.f, 0.f, 0.f};
			out.thrust_sp_body = Vector3f{0.f, 0.f, -_cfg.recover_thrust};
		}

		break;

	case State::Done:
		out.override_setpoints = false;
		break;
	}

	out.state = _state;
	return out;
}
