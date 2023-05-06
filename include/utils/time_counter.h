#ifndef _TIME_COUNTER_H_
#define _TIME_COUNTER_H_

#include <chrono>
#include <iostream>

enum TimerUint
{
	SEC,
	MILLISEC,
	MICROSEC,
	NANOSEC
};

std::chrono::time_point<std::chrono::system_clock> g_last_time;

static std::chrono::time_point<std::chrono::system_clock> start_timer_clock()
{
	return g_last_time = std::chrono::system_clock::now();
}

static double end_timer_clock(const TimerUint &uint)
{
	auto end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - g_last_time);
	double dur = -1.0F;

	switch (uint)
	{
	case SEC:
		dur = double(duration.count()) * std::chrono::nanoseconds::period::num / std::chrono::nanoseconds::period::den;
		break;
	case MILLISEC:
		dur = double(duration.count()) * std::chrono::nanoseconds::period::num / std::chrono::microseconds::period::den;
		break;
	case MICROSEC:
		dur = double(duration.count()) * std::chrono::nanoseconds::period::num / std::chrono::milliseconds::period::den;
		break;
	case NANOSEC:
		dur = double(duration.count()) * std::chrono::nanoseconds::period::num;
		break;
	default:
		break;
	}

	g_last_time = end;

	return dur;
}

#endif // timer_clock.h