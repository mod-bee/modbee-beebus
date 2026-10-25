#include "BeeBusPlatform.h"

#if defined(BEEBUS_PLATFORM_ARDUINO)

#include <Arduino.h>
#include "BeeBusNoExcept.h"

static bb_time_t platformMillis() { return (bb_time_t)millis(); }
static bb_time_t platformMicros() { return (bb_time_t)micros(); }

static uint32_t platformEntropy() {
#if defined(ESP32) || defined(ESP_PLATFORM)
    return esp_random();
#elif defined(ARDUINO_ARCH_RP2040)
    return rp2040.hwrand32();
#else
    // No hardware source: the time since reset in microseconds, mixed with
    // where the stack happens to be. Differs between starts unless two starts
    // reach this call at the same microsecond.
    volatile uint8_t marker = 0;
    uint32_t mixed = (uint32_t)micros() ^ ((uint32_t)(uintptr_t)&marker << 16);
    mixed ^= mixed >> 15;
    mixed *= 0x2C1B3C6Du;
    mixed ^= mixed >> 12;
    return mixed;
#endif
}

#else

#include <chrono>
#include <random>
#include "BeeBusNoExcept.h"

namespace {

// Anchored at first use so both clocks start near zero, matching Arduino.
std::chrono::steady_clock::time_point startupTime() {
    static const std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();
    return t0;
}

} // namespace

static bb_time_t platformMillis() {
    using namespace std::chrono;
    const auto delta = steady_clock::now() - startupTime();
    return (bb_time_t)(uint64_t)duration_cast<milliseconds>(delta).count();
}

static bb_time_t platformMicros() {
    using namespace std::chrono;
    const auto delta = steady_clock::now() - startupTime();
    return (bb_time_t)(uint64_t)duration_cast<microseconds>(delta).count();
}

static uint32_t platformEntropy() {
    // The OS source, mixed with the wall clock so a platform whose
    // random_device is deterministic still differs between starts.
    std::random_device device;
    const uint64_t now = (uint64_t)std::chrono::system_clock::now().time_since_epoch().count();
    return (uint32_t)device() ^ (uint32_t)now ^ (uint32_t)(now >> 32);
}

#endif

// --- Clock source ---

namespace {

// Constant-initialised with the platform clocks, so there is no static
// initialisation order concern and no null check on the hot path.
bb_clock_fn g_millisSource = &platformMillis;
bb_clock_fn g_microsSource = &platformMicros;

} // namespace

void bb_setClockSource(bb_clock_fn millisSource, bb_clock_fn microsSource) {
    g_millisSource = millisSource ? millisSource : &platformMillis;
    g_microsSource = microsSource ? microsSource : &platformMicros;
}

namespace {
bb_entropy_fn g_entropySource = &platformEntropy;
}  // namespace

uint32_t bb_entropy() { return g_entropySource(); }

void bb_setEntropySource(bb_entropy_fn source) {
    g_entropySource = source ? source : &platformEntropy;
}

void bb_resetClockSource() {
    g_millisSource = &platformMillis;
    g_microsSource = &platformMicros;
}

bb_time_t bb_millis() { return g_millisSource(); }
bb_time_t bb_micros() { return g_microsSource(); }
