#pragma once

#include <matrix/matrix/math.hpp>
// #include <lib/mixer_module/MultirotorMixer/MultirotorMixer.hpp>

class TrackingDifferentiator
{
public:
	TrackingDifferentiator() = default;
	~TrackingDifferentiator() = default;

	void set_td_coef(const float a0,const float a1)
	{
		_a0 = a0;
		_a1 = a1;
	}

	void set_td_ratio_frequency(const float xi,const float wn)
	{
		_a0 = wn * wn;
		_a1 = 2.0f * xi * wn;
	}

	void update(const float input,const float dt)
	{
		v = input;
		v1 += dt * v2;
		v2 += dt * (_a0 * (v - v1) - _a1 * v2);
	}

	float get_input_signal()
	{
		return v;
	}

	float get_tracking_signal()
	{
		return v1;
	}

	float get_differential_signal()
	{
		return v2;
	}

	void reset()
	{
		v = v1 = v2 = 0.0f;
	}
private:
	float _a0{};
	float _a1{};

	float v1{};
	float v2{};
	float v{};
};

class ErrorCombine
{
public:
	ErrorCombine() = default;
	~ErrorCombine() = default;

	void set_error_combine_coef(const float beta1,const float beta2)
	{
		_beta1 = beta1;
		_beta2 = beta2;
	}

	void set_output_limit(const float umin,const float umax)
	{
		_u_min = umin;
		_u_max = umax;
	}

	void set_disturb_gain(const float gain)
	{
		_disturb_gain = gain
	}

	void update(const float v1,const float z1,const float v2,const float z2,const float disturb)
	{
		e1 = v1 - z1;
		e2 = v2 - z2;

		u0 = _beta1 * e1 + _beta2 * e2;
		u = u0 + _disturb_gain * disturb;

		u = math::constrain(u,_u_min,_u_max);
	}

	void reset()
	{
		e1 = 0.0f;
		e2 = 0.0f;
		u0 = 0.0f;
		u = 0.0f;
	}

	float get_error1()
	{
		return e1;
	}

	float get_error2()
	{
		return e2;
	}

	float get_sum_error()
	{
		return u0;
	}

	float get_result()
	{
		return u;
	}

private:
	float _beta1{};
	float _beta2{};

	float _u_min{};
	float _u_max{};

	float e1{};
	float e2{};
	float u0{};
	float u{};

	float _disturb_gain{1.0f};
};

class ExtendedStateObserver
{
public:
	ExtendedStateObserver() = default;
	~ExtendedStateObserver() = default;

	void set_eso_coef(const float beta1,const float beta2,const float beta3,const float b0)
	{
		_beta1 = beta1;
		_beta2 = beta2;
		_beta3 = beta3;
		_b0 = b0;
	}

	void set_eso_gain_cutoff_frequency(const float b0,const float wc)
	{
		_beta1 = 3.0f * wc;
		_beta2 = 3.0f * wc * wc;
		_beta3 = wc * wc * wc;
	}

	void set_disturb_limit(const float min,const float max)
	{
		disturb_min = min;
		disturb_max = max;
	}

	void update(const float output,const float dt,const float input)
	{
		u = input;
		y = output;
		float err = z1 - y;

		z1 += dt * (z2 - _beta1 * err);
		z2 += dt * (z3 + _b0 * u - _beta2 * err);
		z3 += dt * (-_beta3 * err);

		disturb = -z3/_b0;
		disturb = math::constrain(disturb,disturb_min,disturb_max);
	}

	float get_eso_reference()
	{
		return y;
	}

	float get_eso_state1()
	{
		return z1;
	}

	float get_eso_state2()
	{
		return z2;
	}

	float get_eso_state3()
	{
		return z3;
	}

	float get_disturb()
	{
		return disturb;
	}

	void reset()
	{
		z1 = 0.0f;
		z2 = 0.0f;
		z3 = 0.0f;
		disturb = 0.0f;
	}
private:
	float z1{};
	float z2{};
	float z3{};

	float _beta1{};
	float _beta2{};
	float _beta3{};

	float _b0{};
	float y{};
	float u{};

	float disturb{};
	float disturb_min{};
	float disturb_max{};
};

class LADRC
{
public:
	LADRC() = default;
	~LADRC() = default;

	void reset()
	{
		TD.reset();
		EC.reset();
		ESO.reset();
	}

	float update(const float input,const float output,const float dt)
	{
		TD.update(input,dt);

		float v1 = TD.get_tracking_signal();
		float v2 = TD.get_differential_signal();

		float z1 = ESO.get_eso_state1();
		float z2 = ESO.get_eso_state2();
		float z3 = ESO.get_eso_state3();
		float disturb = ESO.get_disturb();

		EC.update(v1,z1,v2,z2,disturb);

		ESO.update(output,dt,input);

		const float result = EC.get_result();
		return result
	}

private:
	TrackingDifferentiator TD;
	ErrorCombine EC;
	ExtendedStateObserver ESO;
};
