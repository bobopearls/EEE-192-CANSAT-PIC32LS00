// MARIE NOTE: I DID NOT REALLY USE THIS CODE. JUST ADD IT IN CASE THE OTHER STUFF FROM 158 NEED IT
// but i'm pretty sure that I didn't use this (Isaac lmk if this was used in the GPS)

// Common include for the XC32 compiler
#include <xc.h>
#include <stdbool.h>
#include <string.h>

#include "../platform.h"

/////////////////////////////////////////////////////////////////////////////

// Normalize a timespec
void platform_timespec_normalize(platform_timespec_t *ts)
{
	while (ts->nr_nsec >= 1000000000) {
		ts->nr_nsec -= 1000000000;
		if (ts->nr_sec < UINT32_MAX) {
			++ts->nr_sec;
		} else {
			ts->nr_nsec = (1000000000 - 1);
			break;
		}
	}
}

// Compare two timestamps
int platform_timespec_compare(const platform_timespec_t *lhs,
	const platform_timespec_t *rhs)
{
	if (lhs->nr_sec < rhs->nr_sec)
		return -1;
	else if (lhs->nr_sec > rhs->nr_sec)
		return +1;
	else if (lhs->nr_nsec < rhs->nr_nsec)
		return -1;
	else if (lhs->nr_nsec > rhs->nr_nsec)
		return +1;
	else
		return 0;
}

/////////////////////////////////////////////////////////////////////////////

// SysTick handling
static volatile platform_timespec_t ts_wall = PLATFORM_TIMESPEC_ZERO;
static volatile uint32_t ts_wall_cookie = 0;
void __attribute__((used, interrupt())) SysTick_Handler(void)
{
	platform_timespec_t t = ts_wall;
	
	t.nr_nsec += (PLATFORM_TICK_PERIOD_US * 1000);
	while (t.nr_nsec >= 1000000000) {
		t.nr_nsec -= 1000000000;
		++t.nr_sec;	// Wrap-around intentional
	}
	
	++ts_wall_cookie;	// Wrap-around intentional
	ts_wall = t;
	++ts_wall_cookie;	// Wrap-around intentional
	
	// Reset before returning.
	SysTick->VAL  = 0x00158158;	// Any value will clear
	return;
}
#define SYSTICK_RELOAD_VAL ((24/2)*PLATFORM_TICK_PERIOD_US)
void platform_systick_init(void)
{
	/*
	 * Since SysTick might be unknown at this stage, do the following, per
	 * the Arm v8-M reference manual:
	 * 
	 * - Program LOAD
	 * - Clear (VAL)
	 * - Program CTRL
	 */
	SysTick->LOAD = SYSTICK_RELOAD_VAL;
	SysTick->VAL  = 0x00158158;	// Any value will clear
	SysTick->CTRL = 0x00000007;
	return;
}
void platform_tick_count(platform_timespec_t *tick)
{
	uint32_t cookie;
	
	// A cookie is used to make sure we get coherent data.
	do {
		cookie = ts_wall_cookie;
		*tick = ts_wall;
	} while (ts_wall_cookie != cookie);
}
void platform_tick_hrcount(platform_timespec_t *tick)
{
	platform_timespec_t t;
	uint32_t s = SYSTICK_RELOAD_VAL - SysTick->VAL;
	
	platform_tick_count(&t);
	t.nr_nsec += (1000 * s)/12;
	while (t.nr_nsec >= 1000000000) {
		t.nr_nsec -= 1000000000;
		++t.nr_sec;	// Wrap-around intentional
	}
	
	*tick = t;
}

// Difference between two ticks
void platform_tick_delta(
	platform_timespec_t *diff,
	const platform_timespec_t *lhs, const platform_timespec_t *rhs
	)
{
	platform_timespec_t d = PLATFORM_TIMESPEC_ZERO;
	uint32_t c = 0;
	
	// Seconds...
	if (lhs->nr_sec < rhs->nr_sec) {
		// Wrap-around
		d.nr_sec = (UINT32_MAX - rhs->nr_sec) + lhs->nr_sec + 1;
	} else {
		// No wrap-around
		d.nr_sec = lhs->nr_sec - rhs->nr_sec;
	}
	
	// Nano-seconds...
	if (lhs->nr_sec < rhs->nr_sec) {
		// Wrap-around
		c = rhs->nr_sec - lhs->nr_sec;
		while (c >= 1000000000) {
			c -= 1000000000;
			if (d.nr_sec == 0) {
				d.nr_sec = UINT32_MAX;
			} else {
				--d.nr_sec;
			}
		}
		if (d.nr_sec == 0) {
			d.nr_sec = UINT32_MAX;
		} else {
			--d.nr_sec;
		}
	} else {
		// No wrap-around
		d.nr_nsec = lhs->nr_nsec - rhs->nr_nsec;
	}
	
	// Normalize...
	*diff = d;
	return;
}
