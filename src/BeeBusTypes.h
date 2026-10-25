#pragma once

// Self-contained so transports and tests can pull in the wire constants
// without dragging in the whole library.
#include "BeeBusPlatform.h"
#include "BeeBusTopicTypes.h"
#include "BeeBusFixed.h"

#include <functional>

// --- Byte order ---
// Everything 16 bit on the wire is big-endian, Modbus and BeeBus alike.

inline void bb_packBE16(uint8_t* out, uint16_t value) {
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)(value & 0xFF);
}

inline uint16_t bb_unpackBE16(const uint8_t* in) {
    return (uint16_t)(((uint16_t)in[0] << 8) | in[1]);
}

// --- Frame delimiting ---
// HDLC-style framing:
//
//     FLAG | escaped(SRC NEXT FLAGS [optional fields] [sections] CRC16) | FLAG
//
// FLAG and ESC are escaped inside the body as ESC followed by the byte XORed
// with 0x20, so FLAG never occurs within a frame and a receiver can always
// resynchronise on the next one.
#define BEEBUS_FLAG              0x7E    // Frame delimiter, start and end
#define BEEBUS_ESC               0x7D    // Escape byte
#define BEEBUS_ESC_XOR           0x20    // Applied to the byte following ESC

// Sent alone in the gap after a frame by a node waiting to be invited, and read
// only outside a frame, where nothing else carries meaning. One byte because
// the gap guarantees room for two and a frame needs a dozen; a joiner that
// sends a frame there is transmitted over before it ends.
//
// It admits nobody, so it needs no address and nothing has to agree on it: it
// says only that somebody is waiting, and whoever hears it invites its own gap
// on the next turn. A byte the line invented costs one invitation.
#define BEEBUS_ANNOUNCE_BYTE     0x55    // Alternating bits; not FLAG or ESC

// Network configuration limits
#define BEEBUS_MAX_RX_BUFFER     512     // Maximum receive buffer size
#define BEEBUS_MAX_TX_BUFFER     512     // Maximum transmit buffer size
// The longest frame as it travels: every byte after the opening flag escaped
// (two on the wire), and the closing flag. What a datagram transport must be
// able to receive in one piece.
#define BEEBUS_MAX_WIRE_FRAME    (2 * BEEBUS_MAX_TX_BUFFER + 2)

// FLAG, SRC, NEXT, FLAGS, CRC16. Everything beyond that is optional and
// announced by a bit in FLAGS.
#define BEEBUS_MIN_FRAME_LEN     6

// Bytes drained from the transport per loop() call. Bounds worst-case time in
// processIncoming() when a node is flooded, so the application loop still runs.
// Leftover bytes stay buffered in the transport and are taken next call.
//
// Not the only bound: draining also stops when the assembled-frame queue is
// full, since this many bytes holds many more control frames than that queue
// has slots. Both limits leave the surplus in the transport rather than
// dropping it.
#define BEEBUS_MAX_RX_BYTES_PER_LOOP  1024

// --- Section header ---
// A section is [LEN16][DEST][PDU...], where LEN counts DEST plus the PDU. The
// Modbus quantity limits cap a PDU at 254 bytes, so the high bits of LEN are
// spare and carry per-section flags.
//
//     15    14   13 12   11 10  9    8 .. 0
//    | ACK | RSP | resvd | PDU TYPE |  LENGTH |
//
#define BEEBUS_SECTION_LEN_MASK   0x01FF  // Bits 0..8 are the length

// Which kind of PDU the section carries, the way an EtherType or an IP protocol
// number says what a frame is carrying. Modbus is 0. A receiver that does not
// know a value drops that section and keeps walking.
#define BEEBUS_SECTION_PDU_MASK   0x0E00
#define BEEBUS_SECTION_PDU_SHIFT  9

#define BEEBUS_PDU_MODBUS         0
#define BEEBUS_PDU_TOPIC          1

// The sender wants this operation acknowledged. The two bytes at the end of the
// section are then the sender's operation ID rather than PDU content, which the
// responder echoes back in its acknowledgement.
#define BEEBUS_SECTION_ACK_REQUESTED 0x8000
#define BEEBUS_ACK_ID_BYTES          2

// This section answers a request rather than making one. Stated because it
// cannot reliably be inferred: a Modbus read request and a read response share
// their function code, and for one length they share their shape as well, so a
// receiver guessing from structure alone misreads a request for 256..511 coils
// as a response and silently never executes it.
//
// A receiver that does not know this bit ignores it, and a sender that does not
// set it leaves the receiver on the structural fallback below, so a ring may
// hold both versions while it is being upgraded.
#define BEEBUS_SECTION_RESPONSE      0x4000

// Acknowledgements to one node share a section: an operation ID and a status
// each, the count implied by the section length.
#define BEEBUS_ACK_ENTRY_BYTES       3
#define BEEBUS_MAX_ACKS_PER_SECTION  32

// How many operations and responses a node can have queued, unless
// BeeBusConfig::maxPendingOps / maxPendingResponses say otherwise (any depth
// to 255). The default on a host; a board's is 16.
#define BEEBUS_MAX_PENDING_OPS          50
#define BEEBUS_MAX_PENDING_RESPONSES    50

// Attempts a retry counter can express. The counters are single bytes, so a
// configured maxRetries above this would be compared against a counter that
// wraps before reaching it: the comparison would never come true, a silent
// successor would never be removed and a lost operation would never report a
// timeout. Read every configured retry budget through here.
#define BEEBUS_MAX_RETRIES              255

inline uint8_t bb_retryLimit(uint32_t configured) {
    return configured > BEEBUS_MAX_RETRIES ? (uint8_t)BEEBUS_MAX_RETRIES
                                           : (uint8_t)configured;
}

// --- Protocol states ---

enum class BeeBusState : uint8_t {
    Offline,         // Not taking part; begin() has run but connect() has not
    Listening,       // Listening for an existing ring before speaking
    Joining,         // Heard a ring, waiting to be solicited
    Requesting,      // Answering a solicitation with a join request
    Member,          // In the ring, not currently holding the token
    HoldingToken,    // Holding the token
    PassingToken,    // Token sent, waiting for the successor to confirm
    Leaving,         // Announcing departure before going offline
};

// Short name for a state, for logging.
inline const char* bb_stateName(BeeBusState state) {
    switch (state) {
        case BeeBusState::Offline:      return "Offline";
        case BeeBusState::Listening:    return "Listening";
        case BeeBusState::Joining:      return "Joining";
        case BeeBusState::Requesting:   return "Requesting";
        case BeeBusState::Member:       return "Member";
        case BeeBusState::HoldingToken: return "HoldingToken";
        case BeeBusState::PassingToken: return "PassingToken";
        case BeeBusState::Leaving:      return "Leaving";
    }
    return "Unknown";
}

// --- Frame header ---
//
//     FLAG | SRC | NEXT | FLAGS | [fields FLAGS asks for] | sections | CRC16
//
// SRC is the sender. NEXT is the node the token is handed to, or 0 when the
// frame carries no token. FLAGS says what else the frame carries; each set bit
// that needs a value pulls one byte in, in bit order.
#define BEEBUS_HDR_SOLICIT        0x01   // Pulls in two bytes: round, window
#define BEEBUS_HDR_MEMBER_JOINED  0x02   // Pulls in one byte: the node that joined
#define BEEBUS_HDR_MEMBER_LEFT    0x04   // Pulls in one byte: the node that left
#define BEEBUS_HDR_HAS_SECTIONS   0x08   // PDU sections follow the header
#define BEEBUS_HDR_TURN           0x10   // Alternating bit: which turn this is
#define BEEBUS_HDR_REPEAT         0x20   // This frame repeats one already sent

// TURN and REPEAT carry no value byte. TURN toggles each time a node takes the
// token afresh and holds across retransmissions, so a successor can tell a new
// turn from a repeat of the one it already took. REPEAT says the sender has sent
// this frame before, separating a retransmission from a medium duplicate.

// Offsets of the three fixed header bytes, past the frame marker at 0.
#define BEEBUS_HDR_SRC            1
#define BEEBUS_HDR_NEXT           2
#define BEEBUS_HDR_FLAGS          3
#define BEEBUS_HDR_FIXED_LEN      4

/**
 * A frame header, as built or as parsed. sectionStart is where the sections
 * begin, which depends on how many optional fields the flags pulled in.
 */
struct BeeBusHeader {
    uint8_t srcNodeID = 0;
    uint8_t successorID = 0;
    uint8_t flags = 0;

    uint8_t solicitRound = 0;     // Valid when BEEBUS_HDR_SOLICIT is set
    uint8_t solicitWindow = 0;    // Valid when BEEBUS_HDR_SOLICIT is set
    uint8_t joinedNodeID = 0;     // Valid when BEEBUS_HDR_MEMBER_JOINED is set
    uint8_t leftNodeID = 0;       // Valid when BEEBUS_HDR_MEMBER_LEFT is set

    uint16_t sectionStart = 0;    // Parsing only: first byte past the header

    bool has(uint8_t bit) const { return (flags & bit) != 0; }

    void solicit(uint8_t round, uint8_t window) {
        flags |= BEEBUS_HDR_SOLICIT;
        solicitRound = round;
        solicitWindow = window;
    }
    void joined(uint8_t nodeID) { flags |= BEEBUS_HDR_MEMBER_JOINED; joinedNodeID = nodeID; }
    void left(uint8_t nodeID)   { flags |= BEEBUS_HDR_MEMBER_LEFT; leftNodeID = nodeID; }
};

// Usable node IDs. 0 means "none" in the header fields, so an addressable node
// is 1..254. BEEBUS_MAX_KNOWN_NODES bounds the fixed-size membership arrays.
#define BEEBUS_NODE_ID_MIN             1
#define BEEBUS_NODE_ID_MAX             254

// A section destination meaning every node at once. Outside the addressable
// range, so it cannot collide with a real node, and only a topic section may
// carry it: a broadcast nothing acknowledges suits a value being reported and
// not a register being written.
#define BEEBUS_NODE_BROADCAST          255
#define BEEBUS_MAX_KNOWN_NODES         254

// Silence reported for a node that has never been heard from at all, which is
// not the same as one that has gone quiet.
#define BEEBUS_SILENCE_NEVER           0xFFFFFFFFu

// --- Gap solicitation ---
//
// A token holder setting BEEBUS_HDR_SOLICIT is saying:
//
//     Anybody whose ID lies strictly between SRC and NEXT may answer now.
//     Successor: wait out the response window before using this token.
//
// NEXT is the smallest member above SRC, so the gap needs no field of its own,
// and the members' gaps tile the whole address space.
//
// The two bytes the flag pulls in are the round number and the response window.
//
// The window is stated rather than derived. Both sides have to agree on it or
// an answer lands after the solicitor has stopped listening, and a figure each
// works out for itself from its own measurements cannot be relied on to match:
// a node that has just powered up has timed nothing like as much as the ring
// it is joining. Stating it makes the agreement structural, and leaves the
// value free to be measured.
//
// The round number is what makes the contention narrow. A node answers a given
// round once; whoever speaks first in a round wins it and the rest stand down
// until the next. So each round admits one node and the set still contending
// is strictly smaller, however the addresses fall.

// Rotations between a member's invitations once the line has shown it carries
// an announcement. A node that wants in says so and is invited on the turn
// after, so this is the sweep that catches an announcement nobody heard, not
// the way nodes normally arrive. Every member invites its own gap and the whole
// ring holds off for each one, so inviting on every rotation costs a large ring
// most of its rotation to find nobody.
#define BEEBUS_SOLICIT_BACKUP_ROTATIONS 6

// How far the response window may widen while answers keep colliding in it, as
// a multiplier. The window is slot space for the answers a gap sends back, and
// how much is needed depends on how many are waiting and how long the medium
// takes to carry one, neither of which a node can know before it asks. Doubling
// on a collision and halving on a quiet round finds it, in the way any
// contention window is found.
#define BEEBUS_SOLICIT_SPREAD_MAX    16

// Turns remembered for spotting a second copy of one. Long enough that other
// nodes speaking between a frame and its duplicate does not hide it, short
// enough that the memory expires well inside a rotation.
#define BEEBUS_TURN_HISTORY          8

// Bytes a control frame occupies on the wire, rounded up for escaping. Used to
// price a solicitation and its answer against the measured line rate, so the
// response window is never shorter than the frames it is waiting for.
#define BEEBUS_CONTROL_FRAME_BYTES   12

// Bytes outside any frame that one response window may hold and still be read
// as two nodes answering at once. A collision is bounded: the answers destroy
// each other and the gap goes quiet. An RS485 pair with nothing driving it is
// not, and it delivers noise for as long as the connector is out. Several
// answers' worth, so escaping and a few contenders still fit, and far below
// what a floating line produces in even one window.
#define BEEBUS_CONTENTION_NOISE_MAX_BYTES  (BEEBUS_CONTROL_FRAME_BYTES * 8)

// Whether nodeID lies strictly between `from` and `to`, going upward and
// wrapping at the top of the address space.
inline bool bb_inGap(uint8_t nodeID, uint8_t from, uint8_t to) {
    if (nodeID == 0 || nodeID == from || nodeID == to) return false;
    if (from == to) return true;                       // A ring of one: everyone else
    if (from < to)  return nodeID > from && nodeID < to;
    return nodeID > from || nodeID < to;               // Wrapped
}

// The response window as one header byte: a three bit exponent and a five bit
// mantissa, so 1 ms resolution up to 31 ms and proportional resolution to about
// four seconds. A wired segment needs the fine end and a store-and-forward
// radio the coarse one. Encoding rounds up, so a stated window is never shorter
// than the one that was asked for.
inline uint8_t bb_encodeWindow(bb_time_t ms) {
    if (ms == 0) return 0;
    uint8_t exponent = 0;
    while (((ms + ((bb_time_t)1 << exponent) - 1) >> exponent) > 31) {
        if (exponent >= 7) return 0xFF;
        exponent++;
    }
    const bb_time_t mantissa = (ms + ((bb_time_t)1 << exponent) - 1) >> exponent;
    return (uint8_t)((exponent << 5) | mantissa);
}

inline bb_time_t bb_decodeWindow(uint8_t code) {
    return (bb_time_t)(code & 0x1F) << (code >> 5);
}

// The next usable node ID above this one, wrapping. 0 and 255 are not
// addressable, so the range closes on itself at 254.
inline uint8_t bb_nextNodeID(uint8_t nodeID) {
    return nodeID >= BEEBUS_NODE_ID_MAX ? BEEBUS_NODE_ID_MIN : (uint8_t)(nodeID + 1);
}

// Whether the gap between two members holds no address at all, in which case
// there is nobody to solicit and no window worth waiting out.
inline bool bb_gapIsEmpty(uint8_t from, uint8_t to) {
    if (from == to) return false;            // A ring of one owns everything
    return bb_nextNodeID(from) == to;
}

// --- Operation result ---

// What happens to an output when the node that commanded it stops answering. A
// safe state is a property of the plant, so it is stated per address.
// HoldLastValue is the default: driving an output is an action, and doing it
// unasked is its own hazard.
enum class BeeBusFailsafe : uint8_t {
    HoldLastValue = 0,   // Leave it exactly as the lost node left it
    ResetToValue,        // Drive it to a stated safe value
};

// Whether a write should be acknowledged by the node that carries it out.
// Unconfirmed goes out with the token and is forgotten; Confirmed stays queued
// until acknowledged, retrying, and reports failure.
//
// The same choice is the QoS of the topic layer, and has those names too:
// Live (best effort, newest wins) is Unconfirmed; Reliable is Confirmed for a
// one-off write or publish, and for a subscription an acknowledged stream that
// repairs what was lost with the current value.
enum class BeeBusDelivery : uint8_t {
    Unconfirmed = 0,
    Confirmed,
    Live = Unconfirmed,
    Reliable = Confirmed,
};
typedef BeeBusDelivery BeeBusQoS;

// Outcome of a data operation. Use bb_accepted() for the common check.
enum class BeeBusResult : uint8_t {
    Queued = 0,       // Accepted; the result arrives on a later token
    Immediate,        // Completed now; the value is already in your variable

    NotInitialised,   // begin() has not been called
    InvalidArgument,  // Null pointer, zero count, or a quantity Modbus disallows
    UnknownNode,      // The destination is not currently on the network
    DuplicateNodeID,  // This node's own ID is in use elsewhere, so it stood down
    NotMapped,        // A local address has no variable bound to it
    Duplicate,        // An identical operation is already pending
    QueueFull,        // The operation queue is at capacity
    Timeout,          // Confirmed write: no acknowledgement after every retry
    NodeLost,         // The destination left the network before it was delivered
    Superseded,       // A newer write to the same address replaced it before it went out
    Cancelled,        // Discarded by clearPendingOps() or a restart
    Refused,          // The destination rejected it; the exact code arrives via onError

    // Topic layer
    TypeMismatch,     // The value's type differs from the binding at the other end
    NotSubscribed,    // No subscription to that topic to act on
    BufferFull,       // A store-and-forward ring overwrote its oldest sample
};

// True when an operation was accepted, immediately or for a later token.
inline bool bb_accepted(BeeBusResult result) {
    return result == BeeBusResult::Queued || result == BeeBusResult::Immediate;
}

// Short name for logging.
inline const char* bb_resultName(BeeBusResult result) {
    switch (result) {
        case BeeBusResult::Queued:          return "Queued";
        case BeeBusResult::Immediate:       return "Immediate";
        case BeeBusResult::NotInitialised:  return "NotInitialised";
        case BeeBusResult::InvalidArgument: return "InvalidArgument";
        case BeeBusResult::UnknownNode:     return "UnknownNode";
        case BeeBusResult::DuplicateNodeID: return "DuplicateNodeID";
        case BeeBusResult::NotMapped:       return "NotMapped";
        case BeeBusResult::Duplicate:       return "Duplicate";
        case BeeBusResult::QueueFull:       return "QueueFull";
        case BeeBusResult::Timeout:         return "Timeout";
        case BeeBusResult::NodeLost:        return "NodeLost";
        case BeeBusResult::Superseded:      return "Superseded";
        case BeeBusResult::Cancelled:       return "Cancelled";
        case BeeBusResult::Refused:         return "Refused";
        case BeeBusResult::TypeMismatch:    return "TypeMismatch";
        case BeeBusResult::NotSubscribed:   return "NotSubscribed";
        case BeeBusResult::BufferFull:      return "BufferFull";
    }
    return "Unknown";
}

// One element of a caller's array. A stride of 0 means packed, which is what a
// plain C array is; anything else is the gap the caller's own layout puts
// between elements.
template <typename T>
inline T* bb_elemAt(void* base, uint16_t index, uint16_t stride) {
    return stride == 0 ? static_cast<T*>(base) + index
                       : reinterpret_cast<T*>(static_cast<uint8_t*>(base) +
                                              (size_t)index * stride);
}

template <typename T>
inline const T* bb_elemAt(const void* base, uint16_t index, uint16_t stride) {
    return stride == 0 ? static_cast<const T*>(base) + index
                       : reinterpret_cast<const T*>(static_cast<const uint8_t*>(base) +
                                                    (size_t)index * stride);
}

// Members of BeeBusResult. A consumer mirroring the enum asserts against this,
// so a member added on one side and not the other is a build error.
#define BEEBUS_RESULT_COUNT 17

// Where a request has got to, for a caller that polls rather than registering
// a completion handler.
enum class BeeBusRequestState : uint8_t {
    Idle,      // Nothing outstanding and nothing waiting to be read
    Pending,   // Queued or in flight
    Done,      // Completed successfully
    Failed     // Completed, and did not succeed
};

// --- Register types ---

enum BeeBusRegisterType {
    MB_OUTPUT_COIL,             // Digital outputs (writable)
    MB_HOLDING_REGISTER,        // Analog outputs (writable)
    MB_INPUT_STATUS,            // Digital inputs (read-only)
    MB_INPUT_REGISTER           // Analog inputs (read-only)
};

// Whether a table holds single bits rather than 16-bit registers.
inline bool bb_tableIsBits(BeeBusRegisterType table) {
    return table == MB_OUTPUT_COIL || table == MB_INPUT_STATUS;
}

// Registers a value of this wire type occupies, or 0 for one that has no
// fixed register image (a string, or a type the wire does not know).
//
//     BOOL                      1 register, 0 or 1
//     I8, U8, I16, U16          1 register, sign- or zero-extended
//     I32, U32, F32             2 registers, HIGH word first
//     I64, U64, F64             4 registers, HIGH word first
//
// Word order is big-endian, the same as the bytes inside a register and the
// same as a string's characters, so a 32-bit value reads left to right on a
// Modbus tool set to "ABCD" (also called big-endian or "high word first"), and
// a 64-bit one on "ABCDEFGH". A REAL or LREAL is its IEEE-754 bit pattern.
// Strings are spans, not values: see addString.
inline uint8_t bb_registersOf(uint8_t type) {
    switch (type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL:
        case BEEBUS_T_I8:
        case BEEBUS_T_U8:
        case BEEBUS_T_I16:
        case BEEBUS_T_U16:  return 1;
        case BEEBUS_T_I32:
        case BEEBUS_T_U32:
        case BEEBUS_T_F32:  return 2;
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64:  return 4;
    }
    return 0;
}

// Whether a value of this wire type belongs in this table. A bit table holds
// BOOLs and nothing else. A register table holds every fixed-width type, BOOL
// included as 0 or 1, because a block of settings that mixes switches and
// numbers is one run of holding registers to every HMI and SCADA. A string is
// packed into registers, so it goes with the register tables too.
inline bool bb_tableAccepts(BeeBusRegisterType table, uint8_t type) {
    const bool bits = (type & BEEBUS_T_MASK) == BEEBUS_T_BOOL;
    if (bb_tableIsBits(table)) return bits;
    return bb_registersOf(type) != 0 || bb_topicTypeIsString(type);
}

// One register of a value's image. `word` 0 is the first (high) register.
// memcpy rather than a cast, so the value need not be aligned.
inline uint16_t bb_valueRegister(const void* variable, uint8_t type, uint8_t word) {
    switch (type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL: { bool v; memcpy(&v, variable, 1); return v ? 1 : 0; }
        case BEEBUS_T_I8:   { int8_t v; memcpy(&v, variable, 1); return (uint16_t)(int16_t)v; }
        case BEEBUS_T_U8:   { uint8_t v; memcpy(&v, variable, 1); return v; }
        case BEEBUS_T_I16:
        case BEEBUS_T_U16:  { uint16_t v; memcpy(&v, variable, 2); return v; }
        case BEEBUS_T_I32:
        case BEEBUS_T_U32:
        case BEEBUS_T_F32: {
            uint32_t bits;
            memcpy(&bits, variable, 4);
            return word == 0 ? (uint16_t)(bits >> 16) : (uint16_t)(bits & 0xFFFFu);
        }
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64: {
            uint64_t bits;
            memcpy(&bits, variable, 8);
            return word < 4 ? (uint16_t)(bits >> (16u * (3u - word))) : 0;
        }
    }
    return 0;
}

// Writes one register of a value's image back into the value. False, with the
// value untouched, when the register holds a number the type cannot: 300 or -1
// for a USINT, 200 for a SINT. A BOOL takes 0 as FALSE and any other number as
// TRUE, which is what a Modbus master writing "1" expects. `changed`, when
// given, says whether the value moved.
inline bool bb_setValueRegister(void* variable, uint8_t type, uint8_t word,
                                uint16_t reg, bool* changed) {
    bool moved = false;
    switch (type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL: {
            bool old; memcpy(&old, variable, 1);
            const bool v = (reg != 0);
            moved = (old != v);
            memcpy(variable, &v, 1);
            break;
        }
        case BEEBUS_T_I8: {
            const int16_t s = (int16_t)reg;
            if (s < -128 || s > 127) return false;
            int8_t old; memcpy(&old, variable, 1);
            const int8_t v = (int8_t)s;
            moved = (old != v);
            memcpy(variable, &v, 1);
            break;
        }
        case BEEBUS_T_U8: {
            if (reg > 255u) return false;
            uint8_t old; memcpy(&old, variable, 1);
            const uint8_t v = (uint8_t)reg;
            moved = (old != v);
            memcpy(variable, &v, 1);
            break;
        }
        case BEEBUS_T_I16:
        case BEEBUS_T_U16: {
            uint16_t old; memcpy(&old, variable, 2);
            moved = (old != reg);
            memcpy(variable, &reg, 2);
            break;
        }
        case BEEBUS_T_I32:
        case BEEBUS_T_U32:
        case BEEBUS_T_F32: {
            uint32_t bits; memcpy(&bits, variable, 4);
            const uint32_t old = bits;
            if (word == 0) bits = (bits & 0x0000FFFFu) | ((uint32_t)reg << 16);
            else           bits = (bits & 0xFFFF0000u) | (uint32_t)reg;
            moved = (old != bits);
            memcpy(variable, &bits, 4);
            break;
        }
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64: {
            if (word > 3) return false;
            uint64_t bits; memcpy(&bits, variable, 8);
            const uint64_t old = bits;
            const unsigned shift = 16u * (3u - word);
            bits = (bits & ~((uint64_t)0xFFFFu << shift)) | ((uint64_t)reg << shift);
            moved = (old != bits);
            memcpy(variable, &bits, 8);
            break;
        }
        default:
            return false;
    }
    if (changed) *changed = moved;
    return true;
}

// Whether a peer can write this table. There is no write function code for the
// two input tables.
inline bool bb_tableIsWritable(BeeBusRegisterType table) {
    return table == MB_OUTPUT_COIL || table == MB_HOLDING_REGISTER;
}

// --- Errors and events ---

enum class BeeBusError : uint8_t {
    // Framing and link
    CrcError,           // CRC check failed on a received frame
    FrameError,         // Frame structure could not be parsed
    BufferOverflow,     // Buffer capacity exceeded

    // Ring membership
    MemberJoined,       // A peer entered the ring
    MemberLeft,         // A peer left or was removed
    NetworkTimeout,     // This node stopped hearing the ring at all
    DuplicateNodeID,    // Another node is using our ID; we have stood down

    // Token ring
    DuplicateToken,     // Bus activity while holding the token

    // Operations
    InvalidRequest,     // Malformed request
    OperationError,     // Operation could not be processed
    OperationTimeout,   // Operation gave up waiting

    // Modbus
    ModbusError,        // Unclassified Modbus failure
    InvalidFunction,    // Function code not supported
    InvalidAddress,     // Register address not mapped
    DeviceFailure,      // Remote device reported a failure

    // Lifecycle
    StateChange,        // Protocol state changed
    Unknown,            // Unclassified
};

// Short name for an error or event, for logging.
inline const char* bb_errorName(BeeBusError error) {
    switch (error) {
        case BeeBusError::CrcError:          return "CrcError";
        case BeeBusError::FrameError:        return "FrameError";
        case BeeBusError::BufferOverflow:    return "BufferOverflow";
        case BeeBusError::MemberJoined:      return "MemberJoined";
        case BeeBusError::MemberLeft:        return "MemberLeft";
        case BeeBusError::NetworkTimeout:    return "NetworkTimeout";
        case BeeBusError::DuplicateNodeID:   return "DuplicateNodeID";
        case BeeBusError::DuplicateToken:    return "DuplicateToken";
        case BeeBusError::InvalidRequest:    return "InvalidRequest";
        case BeeBusError::OperationError:    return "OperationError";
        case BeeBusError::OperationTimeout:  return "OperationTimeout";
        case BeeBusError::ModbusError:       return "ModbusError";
        case BeeBusError::InvalidFunction:   return "InvalidFunction";
        case BeeBusError::InvalidAddress:    return "InvalidAddress";
        case BeeBusError::DeviceFailure:     return "DeviceFailure";
        case BeeBusError::StateChange:       return "StateChange";
        case BeeBusError::Unknown:           return "Unknown";
    }
    return "Unknown";
}

// --- Modbus function codes ---

#define MB_FC_READ_COILS            0x01    // Read coil status
#define MB_FC_READ_DISCRETE_INPUTS  0x02    // Read discrete inputs
#define MB_FC_READ_HOLDING_REGISTERS 0x03   // Read holding registers
#define MB_FC_READ_INPUT_REGISTERS  0x04    // Read input registers
#define MB_FC_WRITE_SINGLE_COIL     0x05    // Write single coil
#define MB_FC_WRITE_SINGLE_REGISTER 0x06    // Write single register
#define MB_FC_WRITE_MULTIPLE_COILS  0x0F    // Write multiple coils
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10 // Write multiple registers

// Acknowledgement, carrying the operation ID being acknowledged and a status.
// From the Modbus user-defined range 0x41..0x48, so it cannot collide with a
// standard code. Not a write code: the section classifier treats those as
// requests, and the acknowledgement would re-execute the write on the requester.
#define MB_FC_ACK                   0x46

// Status carried by an acknowledgement. Anything else is a Modbus exception
// code explaining why the operation was refused.
#define MB_ACK_OK                   0x00

// --- Modbus exception codes ---

#define MB_EX_ILLEGAL_FUNCTION      0x01    // Function code not supported
#define MB_EX_ILLEGAL_DATA_ADDRESS  0x02    // Register address invalid
#define MB_EX_ILLEGAL_DATA_VALUE    0x03    // Data value invalid
#define MB_EX_SLAVE_DEVICE_FAILURE  0x04    // Device failure occurred

// Short name for an exception code, for logging.
inline const char* bb_exceptionName(uint8_t exceptionCode) {
    switch (exceptionCode) {
        case MB_EX_ILLEGAL_FUNCTION:     return "illegal function";
        case MB_EX_ILLEGAL_DATA_ADDRESS: return "illegal data address";
        case MB_EX_ILLEGAL_DATA_VALUE:   return "illegal data value";
        case MB_EX_SLAVE_DEVICE_FAILURE: return "slave device failure";
    }
    return "unknown exception";
}

// Maps an exception code onto the result an operation's completion handler
// receives. Coarser than bb_exceptionToError, which carries the exact reason.
inline BeeBusResult bb_exceptionToResult(uint8_t exceptionCode) {
    switch (exceptionCode) {
        case MB_EX_ILLEGAL_DATA_ADDRESS: return BeeBusResult::NotMapped;
        case MB_EX_ILLEGAL_DATA_VALUE:   return BeeBusResult::InvalidArgument;
    }
    return BeeBusResult::Refused;
}

// Maps an exception code onto the error the application's handler receives.
inline BeeBusError bb_exceptionToError(uint8_t exceptionCode) {
    switch (exceptionCode) {
        case MB_EX_ILLEGAL_FUNCTION:     return BeeBusError::InvalidFunction;
        case MB_EX_ILLEGAL_DATA_ADDRESS: return BeeBusError::InvalidAddress;
        case MB_EX_ILLEGAL_DATA_VALUE:   return BeeBusError::InvalidRequest;
        case MB_EX_SLAVE_DEVICE_FAILURE: return BeeBusError::DeviceFailure;
    }
    return BeeBusError::ModbusError;
}

// --- Data structures ---

// One length-prefixed section within a received frame. `start` indexes the
// destination node ID and `end` is one past the last PDU byte, so the PDU is
// [start + 1, end).
struct BeeBusSection {
    uint16_t start = 0;
    uint16_t end = 0;
    bool ackRequested = false;

    // Whether the sender said this section answers a request. False also when
    // the sender leaves the bit clear, so it is grounds for taking the response
    // path and not for ruling it out.
    bool declaredResponse = false;

    // What kind of PDU the section carries, from the header. Modbus is 0.
    uint8_t pduType = BEEBUS_PDU_MODBUS;
};

// A Modbus request or response.
struct BeeBusModbusRequest {
    uint8_t function;                   // Modbus function code
    uint16_t startAddr;                 // Starting register address
    uint16_t quantity;                  // Number of registers/coils
    BeeBusBytes data;                   // Data payload
    bool isResponse = false;            // Response flag
};

// What a read's result buffer holds. Bits and registers are implied by the
// function code; text is not, because a string read is an ordinary register
// read whose buffer is characters.
enum class BeeBusResultKind : uint8_t {
    Native = 0,
    Text,
};

// How an operation reports its outcome: a plain function and its context (the
// node or protocol that queued it), called with the operation's own node,
// address (or topic) and function (or opcode). No allocation and no copy of a
// closure; what a closure would have captured is in the operation already.
struct BeeBusCompletion {
    void (*fn)(void* ctx, uint8_t nodeID, uint16_t address, uint8_t function,
               BeeBusResult result) = nullptr;
    void* ctx = nullptr;
    explicit operator bool() const { return fn != nullptr; }
    void operator()(uint8_t nodeID, uint16_t address, uint8_t function,
                    BeeBusResult result) const {
        if (fn) fn(ctx, nodeID, address, function, result);
    }
};

inline BeeBusCompletion bb_completion(void (*fn)(void*, uint8_t, uint16_t, uint8_t, BeeBusResult),
                                      void* ctx) {
    BeeBusCompletion completion;
    completion.fn = fn;
    completion.ctx = ctx;
    return completion;
}

struct BeeBusPendingModbusOp {
    uint8_t destNodeID;                 // Target node ID
    uint8_t sourceNodeID;               // Source node ID

    // Which PDU family req carries. Modbus is 0, so everything queued by the
    // register path keeps the behaviour it always had.
    //
    // A topic operation reuses these fields rather than adding a second set:
    // req.function is the opcode, req.startAddr the topic ID, and req.data the
    // whole PDU, already encoded.
    uint8_t pduType = BEEBUS_PDU_MODBUS;

    // Set on an operation that must not be replaced by a later one to the same
    // address. A store-and-forward flush is a history rather than a current
    // value, and collapsing it would deliver the newest sample and drop the
    // reason it was recorded.
    bool noSupersede = false;

    BeeBusModbusRequest req;                  // Request data
    bb_time_t timestamp;                // Queue timestamp
    uint8_t retryCount;                 // Retry counter

    // Destination for a read response. Null for writes, which copy their
    // payload into req.data when queued rather than holding a pointer.
    void* resultPtr;

    // What resultPtr points at, so the response is unpacked into the right type.
    BeeBusResultKind resultKind = BeeBusResultKind::Native;

    // Bytes available at resultPtr. Needed only where the length the wire
    // carries is not implied by the quantity: a register read writes exactly
    // two bytes per register, but a text read copies whatever byte count the
    // response declares, and an odd one is half a register more than the
    // quantity accounts for. Bounding that copy in registers and performing it
    // in bytes is how a peer overruns the caller's buffer. Set for a text read;
    // zero elsewhere, where arraySize is the bound.
    uint16_t resultCapacity = 0;

    bool isArray;                       // Array operation flag
    uint16_t arraySize;                 // Array size if applicable

    // Set once the request has been transmitted. A read, or a write awaiting
    // acknowledgement, stays queued until its answer arrives or it times out,
    // and must not be sent again meanwhile.
    bool awaitingResponse = false;

    // Identifies this operation on the wire when an acknowledgement is asked
    // for, from a per-node counter. Destination, function and address do not
    // identify it: a write can be in flight while one superseding it is queued.
    uint16_t opId = 0;
    uint16_t resultStride = 0;          // Gap between the caller's elements, 0 when packed

    // Whether the destination should acknowledge this operation. Only writes
    // set it; a read is answered by its response either way.
    bool wantsAck = false;

    // Called when the operation reaches a conclusion: Immediate when it
    // succeeded, or a failure result when it was refused or gave up. Optional.
    BeeBusCompletion onComplete;
};

// True for the function codes that expect a response carrying data.
inline bool bb_isReadFunction(uint8_t functionCode) {
    const uint8_t base = functionCode & 0x7F;
    return base == MB_FC_READ_COILS ||
           base == MB_FC_READ_DISCRETE_INPUTS ||
           base == MB_FC_READ_HOLDING_REGISTERS ||
           base == MB_FC_READ_INPUT_REGISTERS;
}

// Whether an operation is waiting for something to come back, so it stays
// queued after transmission instead of being retired. The two families answer
// differently and the function code alone cannot be asked: 0x02 is a Modbus
// discrete-input read and a topic READ.
inline bool bb_opExpectsResponse(const BeeBusPendingModbusOp& op) {
    if (op.req.isResponse) return false;
    return op.pduType == BEEBUS_PDU_TOPIC ? bb_topicExpectsReply(op.req.function)
                                          : bb_isReadFunction(op.req.function);
}

// Whether a topic write carries a request ID already given it, which means it
// has been on the wire: BEEBUS_WRITE_F_REQUEST set and the ID not 0. The ID
// follows the flags byte, after the opcode and the topic ID's varint (then the
// writer's session and the owner's).
inline bool bb_topicRequestNumbered(const BeeBusPendingModbusOp& op) {
    if (op.pduType != BEEBUS_PDU_TOPIC || op.req.function != BEEBUS_TOP_WRITE) return false;
    const size_t size = op.req.data.size();
    if (size < 2) return false;
    const size_t flagsAt = (op.req.data[1] & 0x80) ? 3 : 2;
    if (flagsAt + 2 >= size || (op.req.data[flagsAt] & BEEBUS_WRITE_F_REQUEST) == 0) return false;
    return op.req.data[flagsAt + 1] != 0 || op.req.data[flagsAt + 2] != 0;
}

// Maps a topic refusal onto the result its completion handler receives.
inline BeeBusResult bb_topicReasonToResult(uint8_t reason) {
    switch (reason) {
        case BEEBUS_TOP_OK:            return BeeBusResult::Immediate;
        case BEEBUS_TOP_ERR_NO_TOPIC:  return BeeBusResult::NotMapped;
        case BEEBUS_TOP_ERR_TYPE:      return BeeBusResult::TypeMismatch;
        case BEEBUS_TOP_ERR_MALFORMED: return BeeBusResult::InvalidArgument;
        case BEEBUS_TOP_ERR_NO_ROOM:   return BeeBusResult::QueueFull;
        // The owner restarted since: what became of the request is not known.
        case BEEBUS_TOP_ERR_STALE:     return BeeBusResult::Cancelled;
    }
    return BeeBusResult::Refused;
}

// And onto the error the application's handler receives.
inline BeeBusError bb_topicReasonToError(uint8_t reason) {
    switch (reason) {
        case BEEBUS_TOP_ERR_NO_TOPIC:  return BeeBusError::InvalidAddress;
        case BEEBUS_TOP_ERR_TYPE:      return BeeBusError::InvalidRequest;
        case BEEBUS_TOP_ERR_READ_ONLY: return BeeBusError::InvalidRequest;
        case BEEBUS_TOP_ERR_MALFORMED: return BeeBusError::InvalidRequest;
        case BEEBUS_TOP_ERR_NO_ROOM:   return BeeBusError::OperationError;
    }
    return BeeBusError::OperationError;
}

// True for the bit-oriented reads, whose responses carry a byte count rather
// than a bit count.
inline bool bb_isBitReadFunction(uint8_t functionCode) {
    const uint8_t base = functionCode & 0x7F;
    return base == MB_FC_READ_COILS || base == MB_FC_READ_DISCRETE_INPUTS;
}

// A response queued until this node next holds the token.
struct BeeBusPendingResponse {
    uint8_t pduType = BEEBUS_PDU_MODBUS;   // Which family `response` is in
    BeeBusModbusRequest response;             // Response data
    uint8_t destNodeID;                 // Response destination
    uint8_t sourceNodeID;               // Response source
    bb_time_t timestamp;                // Queue timestamp
};

// Identifies an outstanding read, for matching a response to its request.
struct BeeBusPendingReadKey {
    uint8_t nodeID;                     // Target node ID
    uint8_t functionCode;               // Modbus function code
    uint16_t address;                   // Register address

    bool operator<(const BeeBusPendingReadKey& other) const {
        if (nodeID != other.nodeID) return nodeID < other.nodeID;
        if (functionCode != other.functionCode) return functionCode < other.functionCode;
        return address < other.address;
    }
};

// What an outstanding read is waiting for.
struct BeeBusPendingReadInfo {
    uint16_t quantity;                  // Number of items requested
    bb_time_t timestamp;                // Request timestamp
    uint8_t retryCount;                 // Retry counter
};

// --- Callback types ---

typedef void (*BeeBusErrorHandler)(BeeBusError error, const char* msg);

// A peer wrote a new value into this node's data map. Raised once the value has
// been stored, so a handler reading the bound variable sees it, and only when
// the value actually changed. `fromNodeID` is the peer that wrote it. A string
// raises one event for the whole string rather than one per register.
typedef std::function<void(uint16_t address, bool value, uint8_t fromNodeID)>
        BeeBusCoilChangedHandler;
typedef std::function<void(uint16_t address, int16_t value, uint8_t fromNodeID)>
        BeeBusHregChangedHandler;
typedef std::function<void(uint16_t address, const char* text, uint8_t fromNodeID)>
        BeeBusStringChangedHandler;

// An address was driven to its safe value because the node that last wrote it
// left the network. Without this an output driven to safe is indistinguishable
// from one deliberately commanded.
typedef std::function<void(BeeBusRegisterType type, uint16_t address, uint8_t lostNodeID)>
        BeeBusFailsafeHandler;

// A peer entered or left the ring.
typedef std::function<void(uint8_t nodeID)> BeeBusMemberHandler;

// --- Topic events ---

// A peer published a topic this node has registered from it. Raised once the
// value has been stored, so a handler reading the bound variable sees it.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value)>
        BeeBusTopicChangedHandler;

// A peer wrote one of this node's own topics. Separate from a publish, because
// the two carry the same three arguments and mean opposite directions.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value)>
        BeeBusTopicWrittenHandler;

// The string forms of the two above, for topics bound to a text buffer, whose
// value an eight byte BeeBusScalar cannot carry.
//
// Needed rather than reading the bound variable inside the scalar handler: a
// store-and-forward flush writes the variable only for the newest sample but
// raises the handler for every one, so a handler logging fault history would
// otherwise record the newest message once per entry.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, const char* text,
                           uint16_t length)>
        BeeBusTopicStringChangedHandler;
typedef std::function<void(uint8_t nodeID, uint16_t topicID, const char* text,
                           uint16_t length)>
        BeeBusTopicStringWrittenHandler;

// Outcome of a topic operation this node issued: a publish, a read, a write, a
// subscription. `opcode` is the BEEBUS_TOP_* it answers, because more than one
// operation may be outstanding on one topic and "which of them finished" is the
// question a caller is asking.
//
// The register operations report the same way, through onReadComplete and
// onWriteComplete. Without it an application could see a request accepted but
// never learn whether it arrived, which is the difference between a BUSY that
// clears and one that hangs.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, uint8_t opcode,
                           BeeBusResult result)>
    BeeBusTopicCompleteHandler;

// Publishes of a topic went missing. The sequence on every publish is what
// makes an event-driven network honest: without it, nothing distinguishes a
// value that has not changed from a publisher that has stopped being heard.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, uint8_t missed)>
        BeeBusTopicGapHandler;

// One entry of a peer's topic list arrived, during commissioning.
typedef std::function<void(uint8_t nodeID, const BeeBusTopicEntry& entry)>
        BeeBusTopicListHandler;
