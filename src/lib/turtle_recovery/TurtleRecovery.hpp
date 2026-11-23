/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>

namespace turtle
{
constexpr float LEVEL_START_COS = .9986295f; // 3 degrees from upright.
constexpr float LEVEL_STOP_COS = .9998477f; // 1 degree from upright.
constexpr float MAX_POWER = .6f; // Hard limit for every recovery output, including legacy parameters.
// Pure controller: no hardware or uORB access, also exercised by host tests.
struct Rotor { float x{}, y{}; uint32_t output{}; };
struct Geometry {
	Rotor rotors[4] {};
	uint32_t mask{};
	bool valid{};
};

// q rotates body FRD into NED. This is R^T * [0,0,1], NOT Euler roll/pitch.
// Absolute heading cancels: changing yaw alone must not change the selected body-side.
inline bool gravityBody(const float q[4], float g[3])
{
	float n = 0.f;

	for (int i = 0; i < 4; ++i) { if (!std::isfinite(q[i])) { return false; } n += q[i] * q[i]; }

	if (n < .81f || n > 1.21f) { return false; }

	g[0] = 2.f * (q[1] * q[3] - q[0] * q[2]) / n;
	g[1] = 2.f * (q[2] * q[3] + q[0] * q[1]) / n;
	g[2] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) / n;
	return true;
}

// Supported geometry: coplanar Quad-X, exactly one rotor in each FRD quadrant.
// Candidate pairs are geometric neighbours, never a diagonal or ESC-number shortcut.
inline uint32_t selectPair(const Geometry &geo, const float g[3], uint32_t blocked_axis = 0, bool forward = false)
{
	if (!geo.valid) { return 0; }

	float best = -1e10f;
	uint32_t selected = 0;

	for (int side = 0; side < 4; ++side) {
		float x = 0.f, y = 0.f;
		uint32_t mask = 0;
		int count = 0;

		for (const Rotor &r : geo.rotors) {
			bool include = side == 0 ? r.y > 0.f : side == 1 ? r.y < 0.f : side == 2 ? r.x > 0.f : r.x < 0.f;

			if (include) { x += r.x; y += r.y; mask |= r.output; ++count; }
		}

		const float length = std::sqrt(x * x + y * y);

		if (count != 2 || length < .001f || (blocked_axis && (mask == blocked_axis || mask == (geo.mask ^ blocked_axis)))) { continue; }

		// Reverse thrust is +body-Z, torque is (sum(y), -sum(x), 0).
		// Desired righting rotation is body-Z cross gravity = (-g_y, g_x, 0).
		// Normal thrust is -body-Z: choose the lower side instead.
		const float score = (forward ? 1.f : -1.f) * (x * g[0] + y * g[1]) / length;

		if (blocked_axis && score < -.0001f) { continue; }

		if (score > best + .0001f) { best = score; selected = mask; }
	}

	// At exactly 180 degrees all scores tie. The first geometric side breaks the
	// symmetry deterministically; it is not an estimate of unknown ground obstacles.
	return selected;
}

// At rest the accelerometer measures specific force: FRD Z is -g upright,
// +g inverted. Reject freefall, impacts and grossly non-gravitational samples.
inline bool gravityAtRest(const float acceleration[3], float g[3])
{
	float norm_sq = 0.f;

	for (int i = 0; i < 3; ++i) {
		if (!std::isfinite(acceleration[i])) { return false; }

		norm_sq += acceleration[i] * acceleration[i];
	}

	const float norm = std::sqrt(norm_sq);

	if (norm <= 7.f || norm >= 12.5f) { return false; }

	for (int i = 0; i < 3; ++i) { g[i] = -acceleration[i] / norm; }

	return true;
}

inline bool invertedAtRest(const float acceleration[3])
{
	float g[3] {};
	return gravityAtRest(acceleration, g) && g[2] < -.1736482f;
}

enum class State : uint8_t { Idle, StopReverse, Reverse, Drive, StopNormal, Normal, Coast, WaitExit };
enum class Reason : uint8_t { None, Disabled, InvalidInputs, InvalidGeometry, NotInverted, Complete, Timeout, Cancelled, TooFast, OutsideRecoveryRange };
struct Input {
	uint64_t now{};
	bool enabled{}, requested{}, safe{}, ground{}, armed{}, inverted{};
	float gravity[3] {0.f, 0.f, 1.f};
	float rate{};
};
struct Config { float power{.4f}, rate_limit{4.f}; uint64_t pulse_us{1500000}, level_pulse_us{5000000}, timeout_us{10000000}; };
struct Output {
	bool own{}, ready{};
	uint8_t command{255}; // 255 = ordinary throttle data, 0 = stop, 20/21 = relative direction
	uint32_t mask{};
	float power{};
};

class Recovery
{
public:
	State state() const { return _state; }
	Reason reason() const { return _reason; }
	uint32_t pair() const { return _pair; }
	bool levelling() const { return _levelling; }
	bool active() const { return _state != State::Idle && _state != State::WaitExit; }
	void frameSent() { if (_frames < 1000) { ++_frames; } }

	Output update(const Input &in, const Geometry &geo, const Config &cfg)
	{
		if (!in.requested) { _seen_outside = true; }

		// Restore saved ESC directions once before allowing arming with this feature.
		// Geometry/mapping cannot change during recovery (driver snapshots it).
		if (in.enabled && !_initialized && geo.valid && !in.armed) {
			_initialized = true; _dirty = geo.mask; _finish = false;
			transition(State::StopNormal, in.now);
		}

		const bool run_safe = in.enabled && in.safe && geo.valid && !in.armed;

		// Cancellation and timeout also cover the unpowered landing verification.
		if (_attempt && (!in.requested || !run_safe || (_levelling && in.gravity[2] <= 0.f) || in.now - _start >= cfg.timeout_us)) {
			_reason = !in.requested ? Reason::Cancelled : !run_safe ? Reason::InvalidInputs :
				  (_levelling && in.gravity[2] <= 0.f) ? Reason::OutsideRecoveryRange : Reason::Timeout;
			_attempt = false; _finish = true;

			if (_state != State::StopNormal && _state != State::Normal) { transition(State::StopNormal, in.now); }
		}

		// First allow inertia after 60 degrees. If that landing fails, a retry
		// continues toward 30 degrees. Completion always requires a stable 30 degrees.
		if (_attempt && !_confirming && in.gravity[2] >= _cutoff_cos - 1e-6f) {
			_confirming = true; _upright_since = 0;

			if (_state != State::StopNormal && _state != State::Normal) { transition(State::StopNormal, in.now); }
		}

		switch (_state) {
		case State::Idle:
			if (in.requested) {
				_reason = !in.enabled ? Reason::Disabled : !geo.valid ? Reason::InvalidGeometry :
					  !run_safe || !in.ground || !_seen_outside ? Reason::InvalidInputs :
					  (!(in.inverted && in.gravity[2] <= -.1736482f) &&
					   !(in.gravity[2] > 0.f && in.gravity[2] <= LEVEL_START_COS + 1e-6f)) ? Reason::OutsideRecoveryRange : Reason::None;
				_seen_outside = false;

				if (_reason != Reason::None) { transition(State::WaitExit, in.now); break; }

				_start = in.now; _attempt = true; _finish = false; _confirming = false;
				_levelling = in.gravity[2] > 0.f;
				_completion_cos = _levelling ? LEVEL_STOP_COS : .8660254f;
				_cutoff_cos = _levelling ? LEVEL_STOP_COS : .5f;
				_pair = selectPair(geo, in.gravity, 0, _levelling);
				_last_settled_z = in.gravity[2];
				transition(State::StopReverse, in.now);
			}

			break;

		case State::StopReverse:
			if (in.now - _since >= 300000) { transition(State::Reverse, in.now); }

			break;

		case State::Reverse:
			_dirty |= _pair; // even a partial command burst must be restored on cancellation

			if (_frames >= 10 && in.now - _since >= 100000) { transition(State::Drive, in.now); }

			break;

		case State::Drive:
			if (in.rate > 8.f || in.now - _since >= (_levelling ? cfg.level_pulse_us : cfg.pulse_us)) {
				if (in.rate > 8.f) { _reason = Reason::TooFast; }

				transition(State::StopNormal, in.now);
			}

			break;

		case State::StopNormal:
			if (in.now - _since >= 300000 && !in.armed) { transition(State::Normal, in.now); }

			break;

		case State::Normal:
			if ((_dirty == 0 || _frames >= 10) && in.now - _since >= 100000 && !in.armed) {
				_dirty = 0;
				transition(_finish || (in.requested && !_attempt) ? State::WaitExit : _attempt ? State::Coast : State::Idle, in.now);
			}

			break;

		case State::Coast:
			if (!in.ground || in.rate >= .1f || in.gravity[2] < _completion_cos - 1e-6f) { _upright_since = 0; }

			// Wait for motion to settle before reselecting a different neighbouring pair.
			if (in.now - _since >= 300000 && in.ground && in.rate < .15f) {
				if (_confirming) {
					if (in.gravity[2] >= _completion_cos - 1e-6f) {
						if (in.rate >= .1f) { break; }

						if (_upright_since == 0) { _upright_since = in.now; }

						if (in.now - _upright_since >= 2000000) {
							_attempt = false; _finish = true; _reason = Reason::Complete;
							transition(State::WaitExit, in.now);
						}

						break;
					}

					_confirming = false; _cutoff_cos = _completion_cos;
				}

				const uint32_t preferred = selectPair(geo, in.gravity, 0, _levelling);
				// If the same axis made no settled improvement, the support geometry
				// may trap this rocking motion. Try the better perpendicular side,
				// chosen from the current attitude, within the original time limit.
				const bool blocked = !_levelling && in.ground && in.rate < .5f && preferred == _pair &&
						     in.gravity[2] < _last_settled_z + .04f;
				const uint32_t alternative = blocked ? selectPair(geo, in.gravity, preferred) : 0;
				_pair = alternative ? alternative : preferred;
				_last_settled_z = in.gravity[2];
				transition(State::StopReverse, in.now);
			}

			break;

		case State::WaitExit:
			if (!in.requested) { transition(State::Idle, in.now); }

			break;
		}

		Output out{};
		out.own = in.requested || active();
		out.ready = !in.requested && !active() && _dirty == 0 && (!in.enabled || (_initialized && geo.valid));

		if (!out.own) { return out; }

		out.command = 0;

		if (_state == State::Reverse && run_safe && in.requested) { out.command = _levelling ? 20 : 21; out.mask = _pair; _dirty |= _pair; }

		if (_state == State::Normal && !in.armed) { out.command = 20; out.mask = _dirty; }

		if (_state == State::Drive && run_safe && in.requested) {
			out.command = 255; out.mask = _pair;
			const float ramp = static_cast<float>(in.now - _since) / 50000.f;
			// Soft rate feedback throttles without reversing directions or ending
			// the pulse; unlike the hard overspeed stop, it can resume as speed falls.
			// Upright levelling uses a gentler rate ceiling (about 57 deg/s).
			const float rate_limit = _levelling && cfg.rate_limit > 1.f ? 1.f : cfg.rate_limit;
			const float margin = rate_limit - in.rate;
			const float rate_scale = margin > 1.f ? 1.f : margin > 0.f ? margin : 0.f;
			const float power = std::isfinite(cfg.power) && cfg.power > 0.f ?
					    (cfg.power < MAX_POWER ? cfg.power : MAX_POWER) : 0.f;
			out.power = power * rate_scale * (ramp < 1.f ? ramp : 1.f);
		}

		return out;
	}

private:
	void transition(State state, uint64_t now) { _state = state; _since = now; _frames = 0; }
	State _state{State::Idle};
	Reason _reason{Reason::None};
	uint64_t _since{}, _start{}, _upright_since{};
	uint32_t _pair{}, _dirty{};
	float _last_settled_z{}, _cutoff_cos{.5f}, _completion_cos{.8660254f};
	unsigned _frames{};
	bool _initialized{}, _seen_outside{}, _attempt{}, _finish{}, _confirming{}, _levelling{};
};
} // namespace turtle
