/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "FakeGps.hpp"

using namespace time_literals;

FakeGps::FakeGps(double latitude_deg, double longitude_deg, double altitude_m) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_latitude(latitude_deg),
	_longitude(longitude_deg),
	_altitude(altitude_m)
{
}

bool FakeGps::init()
{
	ScheduleOnInterval(SENSOR_INTERVAL_US);
	return true;
}

void FakeGps::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	sensor_gps_s sensor_gps{};
	sensor_gps.time_utc_usec = hrt_absolute_time() + 1613692609599954;
	sensor_gps.latitude_deg = _latitude;
	sensor_gps.longitude_deg = _longitude;
	sensor_gps.altitude_msl_m = _altitude;
	sensor_gps.altitude_ellipsoid_m = _altitude;
	sensor_gps.s_variance_m_s = 0.3740f;
	sensor_gps.c_variance_rad = 0.6737f;
	sensor_gps.eph = 2.1060f;
	sensor_gps.epv = 3.8470f;
	sensor_gps.hdop = 0.8800f;
	sensor_gps.vdop = 1.3300f;
	sensor_gps.noise_per_ms = 101;
	sensor_gps.jamming_indicator = 35;
	sensor_gps.vel_m_s = 0.0420f;
	sensor_gps.vel_n_m_s = 0.0370f;
	sensor_gps.vel_e_m_s = 0.0200f;
	sensor_gps.vel_d_m_s = -0.0570f;
	sensor_gps.cog_rad = 0.3988f;
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = 0.0000;
	sensor_gps.fix_type = 4;
	sensor_gps.jamming_state = 0;
	sensor_gps.spoofing_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = 14;
	sensor_gps.timestamp = hrt_absolute_time();
	_sensor_gps_pub.publish(sensor_gps);

	sensor_gnss_status_s sensor_gnss_status{};
	sensor_gnss_status.quality_corrections = 0;
	sensor_gnss_status.quality_receiver = 9;
	sensor_gnss_status.quality_gnss_signals = 10;
	sensor_gnss_status.quality_post_processing = 255;
	sensor_gnss_status.timestamp = hrt_absolute_time();
	_sensor_gnss_status_pub.publish(sensor_gnss_status);
}

int FakeGps::task_spawn(int argc, char *argv[])
{
	FakeGps *instance = new FakeGps();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
	//task_id_is_work_queue是一个特殊值-2,表示这个模块不是独立线程,而是运行在工作队列上
	//如果 _task_id = -1 表示未运行，_task_id >= 0 是独立线程的句柄，-2 表示工作队列任务。
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

// 当用户在终端输入 fake_gps xxx（其中 xxx 不是 start、stop 或 status）时，这个函数会被调用
// 它直接调用 print_usage("unknown command")，告诉用户“你输入的命令我不认识”，并打印帮助信息
// 如果模块未来需要支持额外命令(如 fake_gps reset) 就可以在这里添加分支逻辑 目前它只负责"报错+显示帮助"
int FakeGps::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeGps::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);		//如果有错误原因，先打印警告
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");						//打印模块描述（目前是空的）

	PRINT_MODULE_USAGE_NAME("fake_gps", "driver");  // 模块名称 + 类别
    	PRINT_MODULE_USAGE_COMMAND("start");            // 说明支持 start 命令
    	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();          // 自动添加 stop 和 status 说明
	return 0;
}

extern "C" __EXPORT int fake_gps_main(int argc, char *argv[])
{
	return FakeGps::main(argc, argv);
}
