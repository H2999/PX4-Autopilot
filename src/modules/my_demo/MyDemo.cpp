/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MyDemo.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

MyDemo::MyDemo() :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

MyDemo::~MyDemo()
{
}

bool
MyDemo::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MyDemo::parameters_updated()
{
}

void
MyDemo::Run()
{
        perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) 
	{
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		if(_param_houzhong_en.get())
		{
			PX4_INFO("Hello, PX4! %f",(double)dt);
		}
        	else
		{
			PX4_INFO("Hello, World!  %f",(double)_param_houzhong_test.get());

		}
	}
}

int MyDemo::task_spawn(int argc, char *argv[])
{
        MyDemo *instance = new MyDemo();

        if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init()) {
                        return PX4_OK;
                }

        } else {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

int MyDemo::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

int MyDemo::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
Demo

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("my_demo", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

extern "C" __EXPORT int my_demo_main(int argc, char *argv[])
{
        return MyDemo::main(argc, argv);
}
