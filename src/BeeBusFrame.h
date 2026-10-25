#pragma once
#include "BeeBusGlobal.h"

/**
 * BeeBusFrame - the wire format.
 *
 * Assembling a frame that carries sections is the caller's job, because only
 * the caller knows which operations fit; what lives here is the header every
 * frame shares and the checks a receiver applies before believing any of it.
 *
 * A frame is built in three steps: buildHeader, then any sections, then finish
 * to stamp the CRC.
 */
class BeeBusFrame {
public:
    // --- Building ---
    // Writes the frame marker and header. Returns the position sections start
    // at, or 0 if the buffer cannot hold the header.
    static uint16_t buildHeader(uint8_t* buffer, uint16_t bufLen, const BeeBusHeader& header);

    // Appends the CRC. Returns the finished frame length, or 0 if it will not
    // fit.
    static uint16_t finish(uint8_t* buffer, uint16_t bufLen, uint16_t pos);

    // A header-only frame, built and finished in one call.
    static uint16_t buildControlFrame(uint8_t* buffer, uint16_t bufLen, const BeeBusHeader& header);

    // --- Parsing ---
    static bool parseHeader(const uint8_t* buffer, uint16_t bufLen, BeeBusHeader& header);
    // Walks the sections after the header, one per call. Start with pos = 0.
    // True with the next one in `section`; false at the end, or at a malformed
    // section (and then nothing after it is read).
    static bool nextSection(const uint8_t* buffer, uint16_t bufLen,
                            const BeeBusHeader& header, uint16_t& pos,
                            BeeBusSection& section);

    // --- Validation ---
    static uint16_t calculateCRC(const uint8_t* buffer, uint16_t length);
    static bool verifyCRC(const uint8_t* buffer, uint16_t length);
};
