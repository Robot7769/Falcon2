#ifndef TIME_HPP
#define TIME_HPP

#include <stdint.h>
#include <Arduino.h>
#include "stopwatch.hpp"

struct base_timer_type
{
	typedef uint32_t time_type;
	time_type value() const { return micros(); }
};
base_timer_type base_timer;

struct stopwatch
	:avrlib::stopwatch<base_timer_type>
{
	stopwatch(bool run = true)
		:avrlib::stopwatch<base_timer_type>(base_timer)
	{
		if(run)
			return;
		stop();
		clear();
	}
};

struct timeout
	:avrlib::timeout<base_timer_type>
{
	timeout(avrlib::timeout<base_timer_type>::time_type timeout)
		:avrlib::timeout<base_timer_type>(base_timer, timeout)
	{
	}
};

void wait(base_timer_type::time_type time)
{
	avrlib::wait(base_timer, time);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process)
{
	avrlib::wait(base_timer, time, process);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process, int)
{
	avrlib::wait(base_timer, time, process, 0);
}

#define  sec(value) stopwatch::time_type(1000000UL*value)
#define msec(value) stopwatch::time_type(1000UL*value)
#define usec(value) stopwatch::time_type(1UL*value)

#endif
