/**
 * @file RateLADRC.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <LADRC.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/time.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/ladrc_status.h>
class RateLADRC
{
public:
	RateLADRC() = default;
	~RateLADRC() = default;

	LADRC ratex;
	LADRC ratey;
	LADRC ratez;

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const float dt, const bool landed);

	void record_rateloop_ladrc_status(ladrc_status_s &ratex_status,ladrc_status_s &ratey_status,ladrc_status_s &ratez_status);
private:


	void record_adrc_status(ladrc_status_s &rate_status , LADRC &ladrc);
};
