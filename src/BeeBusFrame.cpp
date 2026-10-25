#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// --- Building ---
uint16_t BeeBusFrame::buildHeader(uint8_t* buffer, uint16_t bufLen, const BeeBusHeader& header) {
    if (!buffer) {
        return 0;
    }

    // Fixed bytes, plus one for each flag that carries a value.
    uint16_t needed = BEEBUS_HDR_FIXED_LEN;
    if (header.has(BEEBUS_HDR_SOLICIT))       needed += 2;
    if (header.has(BEEBUS_HDR_MEMBER_JOINED)) needed++;
    if (header.has(BEEBUS_HDR_MEMBER_LEFT))   needed++;

    if (bufLen < needed + 2) {   // Room for the CRC as well
        return 0;
    }

    uint16_t pos = 0;

    // buffer[0] is the frame marker. It is covered by the CRC and becomes the
    // opening flag on the wire, where it is written unescaped.
    buffer[pos++] = BEEBUS_FLAG;
    buffer[pos++] = header.srcNodeID;
    buffer[pos++] = header.successorID;
    buffer[pos++] = header.flags;

    // Optional fields in bit order, so a receiver can walk them without
    // knowing anything but the flags byte.
    if (header.has(BEEBUS_HDR_SOLICIT)) {
        buffer[pos++] = header.solicitRound;
        buffer[pos++] = header.solicitWindow;
    }
    if (header.has(BEEBUS_HDR_MEMBER_JOINED)) buffer[pos++] = header.joinedNodeID;
    if (header.has(BEEBUS_HDR_MEMBER_LEFT))   buffer[pos++] = header.leftNodeID;

    return pos;
}

uint16_t BeeBusFrame::finish(uint8_t* buffer, uint16_t bufLen, uint16_t pos) {
    if (!buffer || pos < BEEBUS_HDR_FIXED_LEN || pos + 2 > bufLen) {
        return 0;
    }

    const uint16_t crc = calculateCRC(buffer, pos);
    bb_packBE16(&buffer[pos], crc);
    pos += 2;
    return pos;
}

uint16_t BeeBusFrame::buildControlFrame(uint8_t* buffer, uint16_t bufLen,
                                        const BeeBusHeader& header) {
    const uint16_t pos = buildHeader(buffer, bufLen, header);
    if (pos == 0) {
        return 0;
    }
    return finish(buffer, bufLen, pos);
}

// --- Parsing ---
bool BeeBusFrame::parseHeader(const uint8_t* buffer, uint16_t length, BeeBusHeader& header) {
    if (!buffer || length < BEEBUS_MIN_FRAME_LEN || buffer[0] != BEEBUS_FLAG) {
        return false;
    }

    header = BeeBusHeader();
    header.srcNodeID   = buffer[BEEBUS_HDR_SRC];
    header.successorID = buffer[BEEBUS_HDR_NEXT];
    header.flags       = buffer[BEEBUS_HDR_FLAGS];

    // Everything but the CRC is available to the optional fields.
    const uint16_t limit = (uint16_t)(length - 2);
    uint16_t pos = BEEBUS_HDR_FIXED_LEN;

    if (header.has(BEEBUS_HDR_SOLICIT)) {
        if (pos + 1 >= limit) return false;
        header.solicitRound = buffer[pos++];
        header.solicitWindow = buffer[pos++];
    }
    if (header.has(BEEBUS_HDR_MEMBER_JOINED)) {
        if (pos >= limit) return false;
        header.joinedNodeID = buffer[pos++];
    }
    if (header.has(BEEBUS_HDR_MEMBER_LEFT)) {
        if (pos >= limit) return false;
        header.leftNodeID = buffer[pos++];
    }

    header.sectionStart = pos;
    return true;
}

bool BeeBusFrame::verifyCRC(const uint8_t* buffer, uint16_t length) {
    // The frame marker is part of the CRC, so a frame that lost its opening
    // flag would verify against the wrong bytes.
    if (!buffer || length < BEEBUS_MIN_FRAME_LEN || buffer[0] != BEEBUS_FLAG) {
        return false;
    }

    const uint16_t receivedCRC = bb_unpackBE16(&buffer[length - 2]);
    return receivedCRC == calculateCRC(buffer, length - 2);
}

// --- Sections ---
bool BeeBusFrame::nextSection(
    const uint8_t* buffer,
    uint16_t length,
    const BeeBusHeader& header,
    uint16_t& pos,
    BeeBusSection& section) {

    if (!buffer || length < BEEBUS_MIN_FRAME_LEN || !header.has(BEEBUS_HDR_HAS_SECTIONS)) {
        return false;
    }

    // Walk the length-prefixed sections that follow the header. Each is
    // [LEN16][DEST][PDU...], where LEN counts DEST plus the PDU, so no payload
    // byte can be mistaken for structure.
    if (pos == 0) {
        pos = header.sectionStart;
    }
    const uint16_t dataEnd = (uint16_t)(length - 2);   // Exclude the CRC

    if (pos + 2 > dataEnd) {
        return false;
    }
    const uint16_t sectionHeader = bb_unpackBE16(&buffer[pos]);

    // Only the low bits are the length. The high bits are flags, and must
    // be masked off before the length is used for anything: treating a flag
    // as part of the length would push the section end past the frame and
    // silently discard this section and every one after it.
    const uint16_t sectionLen = sectionHeader & BEEBUS_SECTION_LEN_MASK;

    // A section needs a destination and at least a function code.
    if (sectionLen < 2) {
        BEEBUS_DEBUG_IO("SECTION: Invalid length %d at %d, stopping", sectionLen, pos);
        pos = dataEnd;
        return false;
    }

    const uint16_t sectionStart = (uint16_t)(pos + 2);   // Destination node ID
    const uint16_t sectionEnd = (uint16_t)(sectionStart + sectionLen);

    if (sectionEnd > dataEnd) {
        BEEBUS_DEBUG_IO("SECTION: Truncated section at %d, stopping", pos);
        pos = dataEnd;
        return false;
    }

    section.start = sectionStart;
    section.end = sectionEnd;
    section.ackRequested = (sectionHeader & BEEBUS_SECTION_ACK_REQUESTED) != 0;
    section.declaredResponse = (sectionHeader & BEEBUS_SECTION_RESPONSE) != 0;
    section.pduType = (uint8_t)((sectionHeader & BEEBUS_SECTION_PDU_MASK) >>
                               BEEBUS_SECTION_PDU_SHIFT);
    pos = sectionEnd;
    return true;
}

// --- CRC calculation ---
uint16_t BeeBusFrame::calculateCRC(const uint8_t* buffer, uint16_t length) {
    if (!buffer || length == 0) {
        return 0;
    }

    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)buffer[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}
