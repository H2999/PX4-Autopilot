/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "ImpactReflex.hpp"

using namespace matrix;

namespace
{
ImpactReflex::Config defaultConfig(ImpactReflex::Mode mode)
{
	ImpactReflex::Config cfg{};
	cfg.mode = mode;
	cfg.acc_threshold_mps2 = 45.0f;
	cfg.debounce_s = 0.003f;
	cfg.idle_thrust = 0.07f;
	cfg.drop_time_60_s = 0.05f;
	cfg.drop_time_120_s = 0.11f;
	cfg.recover_thrust = 0.45f;
	cfg.recover_time_s = 0.25f;
	cfg.recover_pitch_rad = 0.14f;
	cfg.recover_roll_rad = 0.0f;
	cfg.recover_tilt_p = 8.0f;
	cfg.recover_rate_max_rads = 10.47f;
	return cfg;
}

ImpactReflex::Input defaultFlightInput()
{
	ImpactReflex::Input in{};
	in.dt_s = 0.001f;
	in.offboard_active = true;
	in.armed = true;
	in.flying = true;
	in.accel_valid = true;
	in.attitude_valid = true;
	in.body_accel_frd = Vector3f{0.f, 0.f, -15.f}; // Normal high-thrust forward flight
	in.attitude_q = Quatf(Eulerf(0.f, -0.6f, 0.3f)); // Nose-down 34 deg dash
	return in;
}
} // namespace

TEST(ImpactReflexTest, DisabledModeNeverOverrides)
{
	ImpactReflex reflex;
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Disabled));

	auto in = defaultFlightInput();
	in.body_accel_frd = Vector3f{-120.f, 0.f, -10.f};

	for (int i = 0; i < 20; i++) {
		const auto out = reflex.update(in);
		EXPECT_FALSE(out.override_setpoints);
		EXPECT_FALSE(out.reset_integral);
		EXPECT_EQ(out.state, ImpactReflex::State::Idle);
	}
}

TEST(ImpactReflexTest, AggressiveThrustFlightAndShortSpikesDoNotTrigger)
{
	ImpactReflex reflex;
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Overhang120));

	auto in = defaultFlightInput();
	// First tick transitions Idle -> Armed
	EXPECT_EQ(reflex.update(in).state, ImpactReflex::State::Armed);

	// 5g pure upward propeller thrust (body -Z) must NEVER trigger impact
	in.body_accel_frd = Vector3f{2.5f, -1.5f, -55.0f};

	for (int i = 0; i < 50; i++) {
		const auto out = reflex.update(in);
		EXPECT_FALSE(out.override_setpoints);
		EXPECT_EQ(out.state, ImpactReflex::State::Armed);
	}

	// 2 ms vibration spike (< 3 ms debounce) followed by normal reading must not trigger
	in.body_accel_frd = Vector3f{-80.0f, 0.0f, -10.0f};
	EXPECT_FALSE(reflex.update(in).override_setpoints);
	EXPECT_FALSE(reflex.update(in).override_setpoints);

	in.body_accel_frd = Vector3f{0.0f, 0.0f, -10.0f};
	EXPECT_FALSE(reflex.update(in).override_setpoints);
	EXPECT_EQ(reflex.state(), ImpactReflex::State::Armed);
}

TEST(ImpactReflexTest, Overhang120SequenceDropThenCatchThenOneShotDone)
{
	ImpactReflex reflex;
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Overhang120));

	auto in = defaultFlightInput();
	// Idle -> Armed
	reflex.update(in);

	// Sustained 6g frontal impact for 3 ms triggers DetachDrop on 3rd tick
	in.body_accel_frd = Vector3f{-60.0f, 5.0f, -10.0f};
	EXPECT_FALSE(reflex.update(in).override_setpoints);
	EXPECT_FALSE(reflex.update(in).override_setpoints);

	const auto trigger_out = reflex.update(in);
	EXPECT_TRUE(trigger_out.override_setpoints);
	EXPECT_TRUE(trigger_out.reset_integral);
	EXPECT_EQ(trigger_out.state, ImpactReflex::State::DetachDrop);
	EXPECT_FLOAT_EQ(trigger_out.rates_sp_rads(0), 0.0f);
	EXPECT_FLOAT_EQ(trigger_out.rates_sp_rads(1), 0.0f);
	EXPECT_FLOAT_EQ(trigger_out.rates_sp_rads(2), 0.0f);
	EXPECT_NEAR(trigger_out.thrust_sp_body(2), -0.07f, 1e-5f);

	// During the next 109 ms, must remain in DetachDrop (NO nose-up pitch command!)
	in.body_accel_frd = Vector3f{0.0f, 0.0f, -1.0f}; // Falling freely under overhang

	for (int i = 0; i < 109; i++) {
		const auto out = reflex.update(in);
		EXPECT_TRUE(out.override_setpoints);
		EXPECT_FALSE(out.reset_integral);
		EXPECT_EQ(out.state, ImpactReflex::State::DetachDrop);
		EXPECT_FLOAT_EQ(out.rates_sp_rads(1), 0.0f);
		EXPECT_NEAR(out.thrust_sp_body(2), -0.07f, 1e-5f);
	}

	// At 110 ms, transitions to CatchRecover: nose-down attitude (-0.6 rad) should
	// generate POSITIVE pitch rate (nose-up recovery in FRD) and high-TWR thrust -0.45.
	const auto catch_out = reflex.update(in);
	EXPECT_TRUE(catch_out.override_setpoints);
	EXPECT_EQ(catch_out.state, ImpactReflex::State::CatchRecover);
	EXPECT_GT(catch_out.rates_sp_rads(1), 2.0f);
	EXPECT_NEAR(catch_out.thrust_sp_body(2), -0.45f, 1e-5f);

	// Run remaining 249 ms of CatchRecover
	for (int i = 0; i < 249; i++) {
		const auto out = reflex.update(in);
		EXPECT_TRUE(out.override_setpoints);
		EXPECT_EQ(out.state, ImpactReflex::State::CatchRecover);
	}

	// At 250 ms of CatchRecover, transitions to Done and releases control
	const auto done_out = reflex.update(in);
	EXPECT_FALSE(done_out.override_setpoints);
	EXPECT_EQ(done_out.state, ImpactReflex::State::Done);

	// One-shot guarantee: another shock while still in Done must NOT re-trigger
	in.body_accel_frd = Vector3f{-100.0f, 0.0f, -10.0f};

	for (int i = 0; i < 10; i++) {
		const auto out = reflex.update(in);
		EXPECT_FALSE(out.override_setpoints);
		EXPECT_EQ(out.state, ImpactReflex::State::Done);
	}

	// Cycling mode 0 -> 1 re-arms for next strike
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Disabled));
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Slope60));
	in.body_accel_frd = Vector3f{0.0f, 0.0f, -10.0f};
	EXPECT_EQ(reflex.update(in).state, ImpactReflex::State::Armed);
}

TEST(ImpactReflexTest, ManualTakeoverImmediatelyAbortsReflex)
{
	ImpactReflex reflex;
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Slope60));

	auto in = defaultFlightInput();
	reflex.update(in);
	in.body_accel_frd = Vector3f{-70.0f, 0.0f, -10.0f};
	reflex.update(in);
	reflex.update(in);
	ASSERT_EQ(reflex.update(in).state, ImpactReflex::State::DetachDrop);

	// Pilot flips RC mode switch out of Offboard
	in.offboard_active = false;
	const auto abort_out = reflex.update(in);
	EXPECT_FALSE(abort_out.override_setpoints);
	EXPECT_EQ(abort_out.state, ImpactReflex::State::Idle);
}

TEST(ImpactReflexTest, CeilingOverhangPushAlongPositiveZTriggersDetach)
{
	ImpactReflex reflex;
	reflex.setConfig(defaultConfig(ImpactReflex::Mode::Overhang120));

	auto in = defaultFlightInput();
	reflex.update(in);

	// Top guard slams upward into 120 deg overhang -> downward specific force (+Z in FRD)
	in.body_accel_frd = Vector3f{0.0f, 0.0f, 30.0f}; // >= 0.6 * 45 = 27 m/s^2
	reflex.update(in);
	reflex.update(in);
	const auto out = reflex.update(in);
	EXPECT_TRUE(out.override_setpoints);
	EXPECT_TRUE(out.reset_integral);
	EXPECT_EQ(out.state, ImpactReflex::State::DetachDrop);
}
