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

/**
 * @file ImpactReflex.hpp
 *
 * High-rate (500-1000 Hz) deterministic impact-detach and attitude-catch
 * reflex for agile quadrotors executing 4-6 m/s intentional target impacts
 * against upward-facing slopes (~60 deg) and downward-facing overhangs
 * (~105-120 deg) where immediate pitch-up would wedge the frame.
 */

#pragma once

#include <stdint.h>
#include <lib/matrix/matrix/math.hpp>

class ImpactReflex
{
public:
	enum class Mode : int32_t {
		Disabled = 0,
		Slope60 = 1,    ///< Upward-facing slope (~60-75 deg, e.g. Base armor)
		Overhang120 = 2 ///< Downward-facing overhang (~105-120 deg, e.g. Outpost armor)
	};

	enum class State : uint8_t {
		Idle = 0,          ///< Disabled or not in eligible flight state
		Armed = 1,         ///< Monitoring BMI088 body acceleration for impact
		DetachDrop = 2,    ///< Idle thrust + zero rate damping to clear wall/overhang
		CatchRecover = 3,  ///< High-TWR tilt-prioritized catch & optional roll-out
		Done = 4           ///< One-shot latch completed; requires mode reset (0 -> 1/2) to re-arm
	};

	struct Config {
		Mode mode{Mode::Disabled};
		float acc_threshold_mps2{45.0f}; ///< Body XY shock threshold (or 0.6x on +Z ceiling push)
		float debounce_s{0.003f};        ///< Continuous shock duration required before trigger
		float idle_thrust{0.07f};        ///< Upward collective thrust during DetachDrop [0..0.25]
		float drop_time_60_s{0.05f};     ///< DetachDrop duration for Slope60 [s]
		float drop_time_120_s{0.11f};    ///< DetachDrop duration for Overhang120 [s]
		float recover_thrust{0.45f};     ///< Safe high-TWR collective thrust during CatchRecover [0..0.85]
		float recover_time_s{0.25f};     ///< CatchRecover duration [s]
		float recover_pitch_rad{0.14f};  ///< Positive FRD pitch (nose-up retreat) during catch [rad]
		float recover_roll_rad{0.0f};    ///< Optional FRD roll-out during catch (e.g. rotating Outpost) [rad]
		float recover_tilt_p{8.0f};      ///< Tilt recovery P gain [1/s]
		float recover_rate_max_rads{10.47f}; ///< Max angular rate command during catch (~600 deg/s) [rad/s]
	};

	struct Input {
		float dt_s{0.001f};
		bool offboard_active{false};        ///< True only in NAVIGATION_STATE_OFFBOARD
		bool armed{false};                  ///< Vehicle is armed
		bool flying{false};                 ///< True when !landed
		bool accel_valid{false};            ///< True when vehicle_acceleration is fresh & finite
		bool attitude_valid{false};         ///< True when vehicle_attitude is fresh & finite
		matrix::Vector3f body_accel_frd{};  ///< Specific force in FRD body frame [m/s^2]
		matrix::Quatf attitude_q{1.f, 0.f, 0.f, 0.f}; ///< FRD -> NED quaternion
	};

	struct Output {
		bool override_setpoints{false};     ///< True during DetachDrop and CatchRecover
		bool reset_integral{false};         ///< One-tick pulse on DetachDrop entry to clear contact windup
		matrix::Vector3f rates_sp_rads{};   ///< FRD body rate setpoint [rad/s]
		matrix::Vector3f thrust_sp_body{};  ///< FRD normalized thrust setpoint (z <= 0 is upward)
		State state{State::Idle};
	};

	void setConfig(const Config &config);
	void reset();

	Output update(const Input &in);

	State state() const { return _state; }

private:
	bool isImpactDetected(const matrix::Vector3f &body_accel_frd) const;
	matrix::Vector3f computeCatchRates(const matrix::Quatf &q_curr) const;

	Config _cfg{};
	State _state{State::Idle};
	float _debounce_timer_s{0.0f};
	float _phase_timer_s{0.0f};
	float _yaw_hold_rad{0.0f};
	bool _yaw_hold_valid{false};
};
