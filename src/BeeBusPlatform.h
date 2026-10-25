#pragma once

#include <stdint.h>
#include <stddef.h>

// --- Platform detection ---

#if defined(ARDUINO)
    #define BEEBUS_PLATFORM_ARDUINO 1
#else
    #define BEEBUS_PLATFORM_HOST 1
#endif

// --- Time ---

// Fixed at 32 bits on every target. `unsigned long` is 32-bit on AVR/ESP32 and
// MSVC but 64-bit on Linux/macOS, which would change rollover behaviour between
// platforms. Pinning the width keeps timeout arithmetic identical everywhere.
typedef uint32_t bb_time_t;

// Milliseconds since startup. Wraps every ~49.7 days.
bb_time_t bb_millis();

// Microseconds since startup. Wraps every ~71.6 minutes.
bb_time_t bb_micros();

// Elapsed time between two stamps. Unsigned arithmetic makes this correct
// across a wrap, provided the interval itself is shorter than the wrap period.
// Prefer it over a bare subtraction: it says the wrap was considered.
inline bb_time_t bb_elapsed(bb_time_t since, bb_time_t now) {
    return (bb_time_t)(now - since);
}

// Whether a deadline is still ahead. Half the range counts as the future, which
// is the standard test for a clock that wraps rather than saturates.
inline bool bb_isFuture(bb_time_t now, bb_time_t deadline) {
    const bb_time_t remaining = bb_elapsed(now, deadline);
    return remaining != 0 && remaining < 0x80000000u;
}

// --- Entropy ---

// 32 bits that differ from one start of the program to the next: the hardware
// random source where the target has one, else whatever the moment of the call
// gives. Used once, for the session ID, never for anything secret.
uint32_t bb_entropy();

// Replaces that source, for a test that has to replay exactly: the session ID
// is on the wire, so a random one changes every frame's escaping and timing.
// nullptr restores the platform's.
typedef uint32_t (*bb_entropy_fn)();
void bb_setEntropySource(bb_entropy_fn source);

// --- Clock source ---

// The protocol reads time only through bb_millis() and bb_micros(), and both
// dispatch through a replaceable source. Overriding it serves two purposes:
//
//   Porting  - drive the protocol from an RTOS tick, a hardware timer, or any
//              clock a bare-metal target provides, without touching the core.
//   Testing  - advance time deterministically, so timing behaviour does not
//              depend on how fast the host runs, and so the 49.7 day rollover
//              can be exercised in milliseconds.
//
// A source must be monotonic within its width and must wrap rather than
// saturate, matching the platform clocks it replaces.
typedef bb_time_t (*bb_clock_fn)();

// Installs a clock source. Passing nullptr for either restores the platform
// clock for that one.
void bb_setClockSource(bb_clock_fn millisSource, bb_clock_fn microsSource);

// Restores both platform clocks.
void bb_resetClockSource();
