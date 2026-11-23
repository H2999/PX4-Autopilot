// Host regression: g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined ...
#include "../../../lib/turtle_recovery/TurtleRecovery.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
#include <random>

using namespace turtle;
Geometry quad()
{
	Geometry geo{};
	// Motor numbering is deliberately unrelated to side; physical outputs 2/5/7/9.
	geo.rotors[0] = {.2f, .2f, 1u << 5};
	geo.rotors[1] = {-.2f, -.2f, 1u << 9};
	geo.rotors[2] = {.2f, -.2f, 1u << 2};
	geo.rotors[3] = {-.2f, .2f, 1u << 7};

	for (const auto &r : geo.rotors) { geo.mask |= r.output; }

	geo.valid = true;
	return geo;
}
void attitude(float roll, float pitch, float yaw, float g[3])
{
	float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2), cy = cosf(yaw / 2), sy = sinf(yaw / 2);
	float q[4] {cr *cp *cy + sr *sp * sy, sr *cp *cy - cr *sp * sy,
		    cr *sp *cy + sr *cp * sy, cr *cp *sy - sr *sp *cy
		   };
	assert(gravityBody(q, g));
}
struct Harness {
	Recovery controller{};
	Geometry geo{quad()};
	Config cfg{};
	Input in{};
	Output out{};
	bool reverse_seen{};
	Harness() { in.now = 1000000; in.enabled = in.safe = in.ground = in.inverted = true; in.gravity[2] = -1.f; }
	void step()
	{
		out = controller.update(in, geo, cfg);
		reverse_seen |= out.command == 21;

		if ((out.command == 20 || out.command == 21) && out.mask) { controller.frameSent(); }

		assert(out.power <= cfg.power && out.power <= MAX_POWER && out.power >= 0.f);

		if (out.power > 0.f) { assert(in.requested && in.safe && !in.armed && out.own); }

		in.now += 10000;
	}
	void wait(int n) { for (int i = 0; i < n; ++i) { step(); } }
	void initialise() { wait(60); assert(out.ready); assert(controller.state() == State::Idle); }
	void enter() { in.requested = true; step(); }
	void until(State state) { for (int i = 0; i < 500 && controller.state() != state; ++i) { step(); } assert(controller.state() == state); }
};
int main()
{
	const Geometry geo = quad();
	float g[3] {};
	const float pi = 3.14159265358979323846f;

	for (float yaw : {0.f, pi / 2, pi, -pi / 2}) {
		attitude(5 * pi / 6, 0.f, yaw, g);
		assert(selectPair(geo, g) == ((1u << 9) | (1u << 2))); // left higher
		attitude(-5 * pi / 6, 0.f, yaw, g);
		assert(selectPair(geo, g) == ((1u << 5) | (1u << 7)));
		attitude(pi, .4f, yaw, g);
		assert(selectPair(geo, g) == ((1u << 5) | (1u << 2))); // front higher
		attitude(pi, -.4f, yaw, g);
		assert(selectPair(geo, g) == ((1u << 9) | (1u << 7)));
	}
	// Sample arbitrary attitudes: selected neighbouring pair's torque always
	// increases body-Z dot world-down (reduces tilt), apart from exact symmetry.
	std::mt19937 rng(27);
	std::uniform_real_distribution<float> angle(-pi, pi);

	for (int n = 0; n < 10000; ++n) {
		const float roll = angle(rng), pitch = angle(rng);
		attitude(roll, pitch, angle(rng), g);
		const uint32_t mask = selectPair(geo, g);
		int count = 0; float x = 0.f, y = 0.f;

		for (const auto &r : geo.rotors) { if (mask & r.output) { ++count; x += r.x; y += r.y; } }

		assert(count == 2 && (fabsf(x) < 1e-5f || fabsf(y) < 1e-5f));
		assert(-(x * g[0] + y * g[1]) >= -.0001f);
		float low_x = 0.f, low_y = 0.f;
		const uint32_t low = selectPair(geo, g, 0, true);
		for (const auto &r : geo.rotors) { if (low & r.output) { low_x += r.x; low_y += r.y; } }
		assert(low_x * g[0] + low_y * g[1] >= -.0001f);
		float other[3] {}; attitude(roll, pitch, angle(rng), other);
		assert(selectPair(geo, other) == mask);
	}

	float invalid_q[4] {NAN, 0.f, 0.f, 0.f}; assert(!gravityBody(invalid_q, g));
	float zero_q[4] {}; assert(!gravityBody(zero_q, g));
	// Startup restoration, disarmed drive, timeout and no automatic retrigger.
	Harness h; h.initialise(); h.enter(); h.until(State::Drive); h.wait(10); assert(h.out.power > 0.f);
	h.in.gravity[2] = 1.f; h.step(); assert(h.out.power == 0.f);
	h.until(State::WaitExit); assert(h.controller.reason() == Reason::Complete); h.wait(100); assert(h.out.power == 0.f);
	h.in.requested = false; h.step(); assert(h.out.ready);

	for (int fault = 0; fault < 5; ++fault) {
		Harness f; f.initialise(); f.enter(); f.until(State::Drive); f.wait(5);

		if (fault == 0) { f.in.safe = false; } // lost RC/IMU, kill, safety or invalid config

		if (fault == 1) { f.in.requested = false; }

		if (fault == 2) { f.in.armed = true; }

		if (fault == 3) { f.in.enabled = false; }

		if (fault == 4) { f.in.now += f.cfg.timeout_us; }

		f.step(); assert(f.out.power == 0.f && f.out.own);

		if (f.in.armed) { f.wait(100); assert(!f.out.ready); f.in.armed = false; }

		f.wait(70); assert(f.out.power == 0.f);
		f.in.requested = false; f.step(); assert(f.out.ready);
	}

	// Longer pulses must survive the former 0.25/1 second limits and stop on time.
	for (uint64_t duration : {1500000ULL, 6000000ULL}) {
		Harness long_pulse; long_pulse.cfg.pulse_us = duration;
		long_pulse.initialise(); long_pulse.enter(); long_pulse.until(State::Drive);
		const uint64_t start = long_pulse.in.now;

		while (long_pulse.in.now - start < duration - 20000) {
			long_pulse.step(); assert(long_pulse.controller.state() == State::Drive);
			assert(long_pulse.out.power > 0.f);
		}

		long_pulse.wait(4);
		assert(long_pulse.controller.state() == State::StopNormal && long_pulse.out.power == 0.f);
	}

	float upright_accel[3] {0.f, 0.f, -9.81f}, inverted_accel[3] {0.f, 0.f, 9.81f};
	assert(!invertedAtRest(upright_accel)); assert(invertedAtRest(inverted_accel));
	float freefall[3] {}, impact[3] {0.f, 0.f, 25.f};
	assert(!invertedAtRest(freefall)); assert(!invertedAtRest(impact));
	float rest_g[3] {}; float measured[3] {-4.905f, 0.f, -8.49571f};
	assert(gravityAtRest(measured, rest_g) && fabsf(rest_g[0] - .5f) < .001f);
	assert(!gravityAtRest(freefall, rest_g) && !gravityAtRest(impact, rest_g));
	Harness not_inverted; not_inverted.initialise(); not_inverted.in.inverted = false;
	not_inverted.enter(); assert(not_inverted.controller.reason() == Reason::OutsideRecoveryRange);

	// The first 60-degree crossing cuts power; success requires a settled 30 degrees.
	for (float yaw : {0.f, pi / 2, pi}) {
		for (int axis = 0; axis < 2; ++axis) {
			for (float deg : {59.9f, 60.f, 60.1f}) {
				Harness boundary; boundary.initialise(); boundary.enter(); boundary.until(State::Drive);
				const float angle = deg * pi / 180.f;
				attitude(axis == 0 ? angle : 0.f, axis == 1 ? angle : 0.f, yaw, boundary.in.gravity);
				boundary.step();

				if (deg <= 60.f) {
					assert(boundary.controller.state() == State::StopNormal && boundary.out.power == 0.f);
					boundary.until(State::Coast); boundary.wait(40);
					assert(boundary.controller.reason() != Reason::Complete);
					boundary.in.gravity[2] = 1.f; boundary.until(State::WaitExit);
					assert(boundary.controller.reason() == Reason::Complete);

				} else {
					assert(boundary.controller.state() == State::Drive && boundary.controller.reason() != Reason::Complete);
				}
			}
		}
	}
	Harness combined; combined.initialise(); combined.enter(); combined.until(State::Drive);
	attitude(50.f * pi / 180.f, 50.f * pi / 180.f, 0.f, combined.in.gravity);
	combined.step(); assert(combined.controller.state() == State::Drive); // total tilt is about 66 degrees
	attitude(20.f * pi / 180.f, 20.f * pi / 180.f, 0.f, combined.in.gravity);
	combined.step(); assert(combined.out.power == 0.f);
	combined.until(State::WaitExit); assert(combined.controller.reason() == Reason::Complete);

	// Brief low-rate contact is not enough: completion requires a continuous 2-second dwell.
	Harness briefly_still; briefly_still.initialise(); briefly_still.enter(); briefly_still.until(State::Drive);
	briefly_still.in.gravity[2] = .9f; briefly_still.until(State::Coast); briefly_still.wait(100);
	assert(briefly_still.controller.reason() != Reason::Complete);
	briefly_still.in.rate = .12f; briefly_still.wait(50); assert(briefly_still.controller.reason() != Reason::Complete);
	briefly_still.in.rate = 0.f; briefly_still.until(State::WaitExit);
	assert(briefly_still.controller.reason() == Reason::Complete);

	// A transient 60-degree crossing stops thrust but must not latch success after a bounce.
	Harness bounce; bounce.initialise(); bounce.enter(); bounce.until(State::Drive);
	bounce.in.gravity[2] = .6f; bounce.in.rate = 2.f; bounce.step(); assert(bounce.out.power == 0.f);
	bounce.until(State::Coast); bounce.wait(40); assert(bounce.out.power == 0.f);
	bounce.in.gravity[2] = .1f; bounce.in.rate = 0.f; bounce.until(State::Drive);
	assert(bounce.controller.reason() != Reason::Complete);

	// After landing at 50 degrees, another pulse must continue through 60 toward 30.
	Harness retry; retry.cfg.timeout_us = 10000000; retry.initialise(); retry.enter(); retry.until(State::Drive);
	retry.in.gravity[2] = cosf(50.f * pi / 180.f); retry.step(); retry.until(State::Coast);
	retry.until(State::Drive); retry.step(); assert(retry.controller.state() == State::Drive);

	for (float degrees : {30.1f, 30.f}) {
		retry.in.gravity[2] = cosf(degrees * pi / 180.f); retry.step();
		assert((retry.controller.state() == State::Drive) == (degrees > 30.f));
	}
	retry.until(State::WaitExit); assert(retry.controller.reason() == Reason::Complete);

	// Upright levelling: low side, saved normal direction only, heading independent.
	for (float yaw : {0.f, pi / 2, pi, -pi / 2}) {
		for (int axis = 0; axis < 4; ++axis) {
			Harness level; level.initialise(); level.in.inverted = false;
			float roll = axis == 0 ? .7f : axis == 1 ? -.7f : 0.f;
			float pitch = axis == 2 ? .7f : axis == 3 ? -.7f : 0.f;
			attitude(roll, pitch, yaw, level.in.gravity);
			const uint32_t low = selectPair(level.geo, level.in.gravity, 0, true);
			level.enter(); level.until(State::Drive); level.wait(10);
			assert(level.controller.levelling() && level.controller.pair() == low);
			assert(level.out.power > 0.f && !level.reverse_seen);
			level.in.rate = 1.1f; level.step(); assert(level.out.power == 0.f);
			level.in.rate = 0.f;
			attitude(1.1f * pi / 180.f, 0.f, yaw, level.in.gravity);
			level.step(); assert(level.controller.state() == State::Drive);
			attitude(1.f * pi / 180.f, 0.f, yaw, level.in.gravity);
			level.step(); assert(level.out.power == 0.f);
			level.until(State::WaitExit); assert(level.controller.reason() == Reason::Complete);
			assert(!level.reverse_seen); level.wait(40); assert(level.out.power == 0.f);
		}
	}

	for (float degrees : {0.f, .5f, 1.f, 2.9f, 3.f, 3.1f, 5.f, 10.f, 20.f, 89.f}) {
		Harness threshold; threshold.initialise(); threshold.in.inverted = false;
		attitude(degrees * pi / 180.f, 0.f, 0.f, threshold.in.gravity); threshold.enter();
		assert(threshold.controller.levelling() == (degrees >= 3.f));
		assert((threshold.controller.state() == State::WaitExit) == (degrees < 3.f));
	}

	// Small real tilt must drive, last beyond the old 3.5s limit, then stop at 5s.
	Harness small; small.initialise(); small.in.inverted = false;
	attitude(5.f * pi / 180.f, 0.f, 0.f, small.in.gravity);
	small.enter(); small.until(State::Drive);
	const uint64_t level_start = small.in.now;

	while (small.in.now - level_start < 4900000) {
		small.step(); assert(small.controller.state() == State::Drive);
		assert(small.out.power > 0.f && !small.reverse_seen);
	}

	small.wait(12); assert(small.out.power == 0.f && small.controller.state() == State::StopNormal);
	// The two pulse parameters are independent; upright may explicitly use the 6s maximum.
	Harness level_max; level_max.cfg.pulse_us = 100000; level_max.cfg.level_pulse_us = 6000000;
	level_max.initialise(); level_max.in.inverted = false;
	attitude(.1f, 0.f, 0.f, level_max.in.gravity); level_max.enter(); level_max.until(State::Drive);
	level_max.wait(590); assert(level_max.controller.state() == State::Drive && level_max.out.power > 0.f);
	level_max.wait(12); assert(level_max.controller.state() == State::StopNormal && level_max.out.power == 0.f);

	// Actual user wiring: MAIN1=M3, MAIN2=M2, MAIN3=M1, MAIN4=M4.
	Geometry wired{}; wired.valid = true; wired.mask = 15;
	wired.rotors[0] = {.03f, .04f, 4}; wired.rotors[1] = {-.03f, -.04f, 2};
	wired.rotors[2] = {.03f, -.04f, 1}; wired.rotors[3] = {-.03f, .04f, 8};
	attitude(.7f, 0.f, 0.f, g); assert(selectPair(wired, g, 0, true) == 12); // right low: MAIN3/4
	attitude(-.7f, 0.f, 0.f, g); assert(selectPair(wired, g, 0, true) == 3); // left low: MAIN1/2
	attitude(0.f, .7f, 0.f, g); assert(selectPair(wired, g, 0, true) == 10); // rear low: MAIN2/4
	attitude(0.f, -.7f, 0.f, g); assert(selectPair(wired, g, 0, true) == 5); // front low: MAIN1/3

	for (int fault = 0; fault < 6; ++fault) {
		Harness level; level.initialise(); level.in.inverted = false;
		attitude(.7f, 0.f, 0.f, level.in.gravity); level.enter(); level.until(State::Drive); level.wait(10);
		if (fault == 0) { level.in.requested = false; }
		if (fault == 1) { level.in.safe = false; }
		if (fault == 2) { level.in.armed = true; }
		if (fault == 3) { level.in.now += level.cfg.timeout_us; }
		if (fault == 4) { level.in.gravity[2] = -.1f; }
		if (fault == 5) { level.in.rate = 8.1f; }
		level.step(); assert(level.out.power == 0.f && !level.reverse_seen);
	}

	// A bounce outside 1 degree resets completion; no new pulse while airborne.
	Harness upright_bounce; upright_bounce.initialise(); upright_bounce.in.inverted = false;
	attitude(.7f, 0.f, 0.f, upright_bounce.in.gravity); upright_bounce.enter(); upright_bounce.until(State::Drive);
	attitude(.01f, 0.f, 0.f, upright_bounce.in.gravity); upright_bounce.until(State::Coast); upright_bounce.wait(100);
	attitude(.4f, 0.f, 0.f, upright_bounce.in.gravity); upright_bounce.in.ground = false;
	upright_bounce.wait(80); assert(upright_bounce.out.power == 0.f && upright_bounce.controller.reason() != Reason::Complete);
	upright_bounce.in.ground = true; upright_bounce.until(State::Drive); assert(!upright_bounce.reverse_seen);

	// The hard 60% cap survives legacy/oversized settings and preserves lower settings.
	for (float requested : {.15f, .6f, 1.f, 50.f}) {
		Harness capped; capped.cfg.power = requested; capped.initialise(); capped.enter(); capped.until(State::Drive); capped.wait(10);
		assert(fabsf(capped.out.power - (requested < MAX_POWER ? requested : MAX_POWER)) < 1e-6f);
	}

	for (float invalid : {NAN, INFINITY, -.1f}) {
		Harness invalid_power; invalid_power.initialise(); invalid_power.enter(); invalid_power.until(State::Drive);
		invalid_power.cfg.power = invalid;
		assert(invalid_power.controller.update(invalid_power.in, invalid_power.geo, invalid_power.cfg).power == 0.f);
	}

	// Soft speed regulation pauses throttle and resumes within the same direction/pulse.
	Harness regulated; regulated.initialise(); regulated.enter(); regulated.until(State::Drive); regulated.wait(10);
	regulated.in.rate = regulated.cfg.rate_limit + .1f; regulated.step();
	assert(regulated.out.power == 0.f && regulated.controller.state() == State::Drive);
	regulated.in.rate = regulated.cfg.rate_limit - .5f; regulated.step(); assert(regulated.out.power > 0.f);

	Harness overspeed; overspeed.initialise(); overspeed.enter(); overspeed.until(State::Drive);
	overspeed.in.rate = 8.1f; overspeed.step();
	assert(overspeed.controller.reason() == Reason::TooFast && overspeed.out.power == 0.f);

	// Exit during the FIRST reverse frame must still restore the selected pair.
	Harness p; p.initialise(); p.enter(); p.until(State::Reverse); assert(p.out.command == 21);
	p.in.requested = false; p.step(); p.until(State::Normal); assert(p.out.command == 20 && p.out.mask != 0);
	// Once a burst begins, merely running a timer cannot stand in for sent frames.
	Recovery c; Input i{}; i.now = 1000000; i.enabled = true; c.update(i, geo, {});
	i.now += 400000; auto o = c.update(i, geo, {}); assert(o.command == 20);
	i.now += 1000000; o = c.update(i, geo, {}); assert(!o.ready && o.command == 20);
	// Mode selected at boot: require a deliberate exit and re-entry.
	Harness b; b.in.requested = true; b.wait(70); assert(b.controller.state() == State::WaitExit); assert(b.out.power == 0.f);

	// Unsupported geometry, not inverted, raised throttle (safe=false), airborne.
	for (int fault = 0; fault < 4; ++fault) {
		Harness f; f.initialise();

		if (fault == 0) { f.geo.valid = false; }

		if (fault == 1) { f.in.gravity[2] = 1.f; }

		if (fault == 2) { f.in.safe = false; }

		if (fault == 3) { f.in.ground = false; }

		f.enter(); f.wait(100); assert(f.out.power == 0.f && f.controller.state() == State::WaitExit);
	}

	// No new pulse until ground-rest qualification returns after a ballistic coast.
	Harness airborne_coast; airborne_coast.initialise(); airborne_coast.enter(); airborne_coast.until(State::Coast);
	airborne_coast.in.ground = false; airborne_coast.wait(100);
	assert(airborne_coast.controller.state() == State::Coast && airborne_coast.out.power == 0.f);
	airborne_coast.in.ground = true; airborne_coast.until(State::Drive);

	// An asymmetric airframe must never use a fallback that tilts away from upright.
	Geometry asymmetric = quad();

	for (auto &rotor : asymmetric.rotors) { rotor.x = rotor.x > 0.f ? .4f : -.01f; }

	float asymmetric_g[3] {.5f, 0.f, -.866f};
	uint32_t preferred = selectPair(asymmetric, asymmetric_g);
	assert(preferred != 0 && selectPair(asymmetric, asymmetric_g, preferred) == 0);

	// Repeated rocking without settled progress tries a perpendicular axis.
	Harness trapped; trapped.initialise(); attitude(2.9f, 0.f, 0.f, trapped.in.gravity);
	trapped.enter(); trapped.until(State::Drive); uint32_t blocked = trapped.controller.pair();
	trapped.until(State::Coast); trapped.until(State::Drive);
	assert(trapped.controller.pair() != blocked && trapped.controller.pair() != (trapped.geo.mask ^ blocked));

	// Next pulse must re-evaluate the attitude rather than retaining a motor pair.
	Harness r; r.initialise(); attitude(2.6f, 0.f, 0.f, r.in.gravity); r.enter(); r.until(State::Drive);
	uint32_t first = r.controller.pair(); r.until(State::Coast);
	attitude(3.14f, .4f, 0.f, r.in.gravity); r.until(State::Drive); assert(r.controller.pair() != first);
	std::puts("PASS: 10000 attitude/yaw cases, physical mapping, 60/30-degree boundaries, restoration, cancellation, re-entry and fault paths");
}
