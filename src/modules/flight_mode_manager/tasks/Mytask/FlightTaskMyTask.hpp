#pragma once

#include "FlightTask.hpp"

class FlightTaskMyTask : public FlightTask
{
pulic:
	FlightTaskMyTask() = default;
	~FlightTaskMyTask() = default;
	bool update() override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
private:
	float _origin_z{0.f};
}
