#pragma once

// Self-contained, so the codec and its tests need nothing else.
#include "BeeBusPlatform.h"

#include <string.h>

// --- Topic layer ---
//
// A topic is a value addressed as (nodeID, topicID). The frame carries the
// node, so topic IDs are per node. Every value carries one byte of type, which
// the receiver checks against its own binding.

// Names cost a pointer per local topic and a fixed buffer per discovered one.
// Compiling them out drops name lookup, the list exchange and wildcards.
#ifndef BEEBUS_TOPIC_NAMES
#define BEEBUS_TOPIC_NAMES 1
#endif

// Topic IDs are varints: 0..127 in one byte, 0..32767 in two.
//
// 0 is not usable. It is the null pointer constant, so a literal 0 would be
// ambiguous between the ID and name forms of every call, and reserving it also
// gives the wire a "no topic" value, as node ID 0 is "no node".
#define BEEBUS_TOPIC_ID_MIN        1
#define BEEBUS_TOPIC_ID_MAX        32767

// Including the terminator: 255 characters, all the single length byte in
// front of a name on the wire can say. Nothing holds a name in a buffer of this
// size: a node keeps its own topics' names where the application put them, and
// a peer's at their own length.
#define BEEBUS_TOPIC_NAME_MAX      256

// --- Value types ---
//
// Deduced from the bound variable. Fixed-width only: `int` is 16 bit on AVR and
// 32 bit on ESP32, so binding one would change the wire type per target.
#define BEEBUS_T_BOOL              0
#define BEEBUS_T_I8                1
#define BEEBUS_T_U8                2
#define BEEBUS_T_I16               3
#define BEEBUS_T_U16               4
#define BEEBUS_T_I32               5
#define BEEBUS_T_U32               6
#define BEEBUS_T_F32               7

// Variable-length text, carrying its own length on the wire. Not fixed width,
// so it has no entry in bb_topicTypeSize and validity is asked of
// bb_topicTypeIsValid instead.
#define BEEBUS_T_STRING            8

// Eight bytes, big-endian like every other width. Numbered after STRING so no
// code a 4-byte node knows moves; such a node refuses these as malformed.
#define BEEBUS_T_I64               9
#define BEEBUS_T_U64               10
#define BEEBUS_T_F64               11
#define BEEBUS_T_COUNT             12

// Characters one string PDU carries, set by the single length byte before them.
#define BEEBUS_TOPIC_STRING_MAX    255

// Set on the type byte when a 24-bit age in seconds follows the value. A flag
// rather than a second opcode, so any publish can be aged.
#define BEEBUS_T_AGED              0x80
#define BEEBUS_T_MASK              0x3F

// Set on the type byte of a value (PUBLISH, READ_REPLY, SUBACK) sent under QoS
// RELIABLE: its sequence number is then the sender's stream to this receiver,
// shared by every topic between the two, not the topic's own.
#define BEEBUS_T_RELIABLE          0x40

// Wire size of a fixed-width type, or 0 for one that has no fixed size. A
// string reports 0, so this cannot be used on its own to ask whether a type is
// known.
inline uint8_t bb_topicTypeSize(uint8_t type) {
    switch (type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL:
        case BEEBUS_T_I8:
        case BEEBUS_T_U8:   return 1;
        case BEEBUS_T_I16:
        case BEEBUS_T_U16:  return 2;
        case BEEBUS_T_I32:
        case BEEBUS_T_U32:
        case BEEBUS_T_F32:  return 4;
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64:  return 8;
    }
    return 0;
}

inline bool bb_topicTypeIsString(uint8_t type) {
    return (type & BEEBUS_T_MASK) == BEEBUS_T_STRING;
}

// Whether the wire carries this type at all. The one test a binding or a build
// should use; bb_topicTypeSize answers a narrower question.
inline bool bb_topicTypeIsValid(uint8_t type) {
    return bb_topicTypeSize(type) != 0 || bb_topicTypeIsString(type);
}

inline const char* bb_topicTypeName(uint8_t type) {
    switch (type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL:   return "bool";
        case BEEBUS_T_I8:     return "i8";
        case BEEBUS_T_U8:     return "u8";
        case BEEBUS_T_I16:    return "i16";
        case BEEBUS_T_U16:    return "u16";
        case BEEBUS_T_I32:    return "i32";
        case BEEBUS_T_U32:    return "u32";
        case BEEBUS_T_F32:    return "f32";
        case BEEBUS_T_STRING: return "str";
        case BEEBUS_T_I64:    return "i64";
        case BEEBUS_T_U64:    return "u64";
        case BEEBUS_T_F64:    return "f64";
    }
    return "?";
}

// What a caller polls instead of registering for onTopicChanged / onTopicGap.
// Both counters are the node's running totals, so a caller compares them with
// what it saw last and two callers cannot take each other's edges.
struct BeeBusTopicStream {
    uint16_t received = 0;   // Accepted publishes, wrapping
    uint16_t missed = 0;     // Publishes the sequence says never arrived, saturating
};

// --- A value of any topic type ---
//
// Twelve bytes whatever the type, so topic and slot records are a fixed size.
// `bits` holds the value widened without interpretation (the low 32 bits of an
// 8-byte type); `high` holds the upper 32 bits of an 8-byte type and is 0 for
// every other; `type` says how to read it back. Two 32-bit words rather than a
// uint64_t, so the record stays 4-byte aligned and a topic costs 4 bytes more,
// not 8.
struct BeeBusScalar {
    uint32_t bits = 0;
    uint32_t high = 0;
    uint8_t type = BEEBUS_T_BOOL;
};

// The whole 64-bit image of a value: `high` above `bits`.
inline uint64_t bb_scalarBits64(const BeeBusScalar& value) {
    return ((uint64_t)value.high << 32) | value.bits;
}

inline void bb_scalarSetBits64(BeeBusScalar& value, uint64_t bits) {
    value.bits = (uint32_t)bits;
    value.high = (uint32_t)(bits >> 32);
}

// IEEE 754 reinterpretation, through memcpy rather than a union or a cast so it
// is defined behaviour.
inline uint32_t bb_floatBits(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline float bb_bitsFloat(uint32_t bits) {
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// F64 is IEEE 754 binary64. A target whose double is narrower (8-bit AVR,
// which this library does not support) gets no F64 binding, below.
inline double bb_bitsDouble(uint64_t bits) {
    double value = 0.0;
    if (sizeof(double) == sizeof(bits)) memcpy(&value, &bits, sizeof(bits));
    return value;
}

inline uint64_t bb_doubleBits(double value) {
    uint64_t bits = 0;
    if (sizeof(double) == sizeof(bits)) memcpy(&bits, &value, sizeof(bits));
    return bits;
}

// The value as a real number, whatever its type, so one deadband comparison
// path serves every type.
inline float bb_scalarToFloat(const BeeBusScalar& value) {
    switch (value.type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL: return value.bits ? 1.0f : 0.0f;
        case BEEBUS_T_I8:   return (float)(int8_t)(uint8_t)value.bits;
        case BEEBUS_T_U8:   return (float)(uint8_t)value.bits;
        case BEEBUS_T_I16:  return (float)(int16_t)(uint16_t)value.bits;
        case BEEBUS_T_U16:  return (float)(uint16_t)value.bits;
        case BEEBUS_T_I32:  return (float)(int32_t)value.bits;
        case BEEBUS_T_U32:  return (float)value.bits;
        case BEEBUS_T_F32:  return bb_bitsFloat(value.bits);
        case BEEBUS_T_I64:  return (float)(int64_t)bb_scalarBits64(value);
        case BEEBUS_T_U64:  return (float)bb_scalarBits64(value);
        case BEEBUS_T_F64:  return (float)bb_bitsDouble(bb_scalarBits64(value));
    }
    return 0.0f;
}

inline bool bb_topicTypeIsWide(uint8_t type) {
    return bb_topicTypeSize(type) == 8;
}

// The value as a double, exact for every type but the 64-bit integers past
// 2^53. For range checks, where a float would round a 32-bit limit.
inline double bb_scalarToDouble(const BeeBusScalar& value) {
    switch (value.type & BEEBUS_T_MASK) {
        case BEEBUS_T_I32: return (double)(int32_t)value.bits;
        case BEEBUS_T_U32: return (double)value.bits;
        case BEEBUS_T_I64: return (double)(int64_t)bb_scalarBits64(value);
        case BEEBUS_T_U64: return (double)bb_scalarBits64(value);
        case BEEBUS_T_F64: return bb_bitsDouble(bb_scalarBits64(value));
    }
    return (double)bb_scalarToFloat(value);
}

// How far apart two values of one type are, for a deadband. In double for an
// 8-byte type, whose difference a float would lose; in float for the rest,
// which the FPU of a small target does in hardware.
inline float bb_scalarDistance(const BeeBusScalar& a, const BeeBusScalar& b) {
    if (bb_topicTypeIsWide(a.type)) {
        double x, y;
        switch (a.type & BEEBUS_T_MASK) {
            case BEEBUS_T_I64:
                x = (double)(int64_t)bb_scalarBits64(a);
                y = (double)(int64_t)bb_scalarBits64(b);
                break;
            case BEEBUS_T_U64:
                x = (double)bb_scalarBits64(a);
                y = (double)bb_scalarBits64(b);
                break;
            default:
                x = bb_bitsDouble(bb_scalarBits64(a));
                y = bb_bitsDouble(bb_scalarBits64(b));
                break;
        }
        const double moved = x - y;
        return (float)(moved < 0.0 ? -moved : moved);
    }
    const float moved = bb_scalarToFloat(a) - bb_scalarToFloat(b);
    return moved < 0.0f ? -moved : moved;
}

// Compares the bits, not the float: a float comparison would lose a one-bit
// change in a 32-bit integer.
inline bool bb_scalarEqual(const BeeBusScalar& a, const BeeBusScalar& b) {
    return a.type == b.type && a.bits == b.bits && a.high == b.high;
}

// Reads a bound variable. The type is the binding's, not inferred.
//
// Fixed-width types only. A string has no representation here and is read
// through the topic map, which holds its text.
inline BeeBusScalar bb_scalarLoad(uint8_t type, const void* variable) {
    BeeBusScalar value;
    value.type = (uint8_t)(type & BEEBUS_T_MASK);
    if (!variable) return value;

    switch (value.type) {
        case BEEBUS_T_BOOL: value.bits = *(const bool*)variable ? 1u : 0u; break;
        case BEEBUS_T_I8:   value.bits = (uint32_t)(uint8_t)*(const int8_t*)variable; break;
        case BEEBUS_T_U8:   value.bits = (uint32_t)*(const uint8_t*)variable; break;
        case BEEBUS_T_I16:  value.bits = (uint32_t)(uint16_t)*(const int16_t*)variable; break;
        case BEEBUS_T_U16:  value.bits = (uint32_t)*(const uint16_t*)variable; break;
        case BEEBUS_T_I32:  value.bits = (uint32_t)*(const int32_t*)variable; break;
        case BEEBUS_T_U32:  value.bits = *(const uint32_t*)variable; break;
        case BEEBUS_T_F32:  value.bits = bb_floatBits(*(const float*)variable); break;
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64: {
            // Through memcpy: the bits are the value whichever of the three it
            // is, and an 8-byte member need not be 8-byte aligned.
            uint64_t bits;
            memcpy(&bits, variable, sizeof(bits));
            bb_scalarSetBits64(value, bits);
            break;
        }
    }
    return value;
}

// Writes a bound variable. The caller has already checked the types agree.
// Fixed-width types only, as bb_scalarLoad above.
inline void bb_scalarStore(const BeeBusScalar& value, void* variable) {
    if (!variable) return;

    switch (value.type & BEEBUS_T_MASK) {
        case BEEBUS_T_BOOL: *(bool*)variable = value.bits != 0; break;
        case BEEBUS_T_I8:   *(int8_t*)variable = (int8_t)(uint8_t)value.bits; break;
        case BEEBUS_T_U8:   *(uint8_t*)variable = (uint8_t)value.bits; break;
        case BEEBUS_T_I16:  *(int16_t*)variable = (int16_t)(uint16_t)value.bits; break;
        case BEEBUS_T_U16:  *(uint16_t*)variable = (uint16_t)value.bits; break;
        case BEEBUS_T_I32:  *(int32_t*)variable = (int32_t)value.bits; break;
        case BEEBUS_T_U32:  *(uint32_t*)variable = value.bits; break;
        case BEEBUS_T_F32:  *(float*)variable = bb_bitsFloat(value.bits); break;
        case BEEBUS_T_I64:
        case BEEBUS_T_U64:
        case BEEBUS_T_F64: {
            const uint64_t bits = bb_scalarBits64(value);
            memcpy(variable, &bits, sizeof(bits));
            break;
        }
    }
}

// --- Compile-time type deduction ---
//
// An unsupported type fails to compile rather than binding something the wire
// cannot carry.
template <typename T>
struct BeeBusTypeOf {
    static const bool supported = false;
    static const uint8_t value = BEEBUS_T_COUNT;
};

#define BEEBUS_DECLARE_TOPIC_TYPE(cppType, wireType)      \
    template <> struct BeeBusTypeOf<cppType> {            \
        static const bool supported = true;               \
        static const uint8_t value = wireType;            \
    }

BEEBUS_DECLARE_TOPIC_TYPE(bool,     BEEBUS_T_BOOL);
BEEBUS_DECLARE_TOPIC_TYPE(int8_t,   BEEBUS_T_I8);
BEEBUS_DECLARE_TOPIC_TYPE(uint8_t,  BEEBUS_T_U8);
BEEBUS_DECLARE_TOPIC_TYPE(int16_t,  BEEBUS_T_I16);
BEEBUS_DECLARE_TOPIC_TYPE(uint16_t, BEEBUS_T_U16);
BEEBUS_DECLARE_TOPIC_TYPE(int32_t,  BEEBUS_T_I32);
BEEBUS_DECLARE_TOPIC_TYPE(uint32_t, BEEBUS_T_U32);
BEEBUS_DECLARE_TOPIC_TYPE(float,    BEEBUS_T_F32);
BEEBUS_DECLARE_TOPIC_TYPE(int64_t,  BEEBUS_T_I64);
BEEBUS_DECLARE_TOPIC_TYPE(uint64_t, BEEBUS_T_U64);
#if !defined(__SIZEOF_DOUBLE__) || __SIZEOF_DOUBLE__ == 8
BEEBUS_DECLARE_TOPIC_TYPE(double,   BEEBUS_T_F64);
#endif

#undef BEEBUS_DECLARE_TOPIC_TYPE

// What every topic template asserts, said once.
#define BEEBUS_TOPIC_TYPE_MESSAGE \
    "Bind a fixed-width type: bool, int8_t, uint8_t, int16_t, uint16_t, " \
    "int32_t, uint32_t, int64_t, uint64_t, float or double"

// Builds a scalar from a value of a supported type.
template <typename T>
inline BeeBusScalar bb_scalarOf(T value) {
    return bb_scalarLoad(BeeBusTypeOf<T>::value, &value);
}

// --- Opcodes ---
//
// Numbered from zero: the section header states the PDU family before any of
// these is read, so they cannot be confused with a Modbus function code.
#define BEEBUS_TOP_PUBLISH         0x00
#define BEEBUS_TOP_WRITE           0x01
#define BEEBUS_TOP_READ            0x02
#define BEEBUS_TOP_READ_REPLY      0x03
#define BEEBUS_TOP_SUBSCRIBE       0x04
#define BEEBUS_TOP_SUBACK          0x05
#define BEEBUS_TOP_UNSUBSCRIBE     0x06
#define BEEBUS_TOP_LIST_REQUEST    0x07
#define BEEBUS_TOP_LIST_REPLY      0x08
#define BEEBUS_TOP_REGISTER        0x09
#define BEEBUS_TOP_ERROR           0x0A
// Protocol version 2: the reliable topic layer.
#define BEEBUS_TOP_STREAM_ACK      0x0B   // What a receiver has of a sender's RELIABLE stream
#define BEEBUS_TOP_RESULT          0x0C   // What the owner did with a write that asked
#define BEEBUS_TOP_SESSION         0x0D   // A node's session, announced once after it joins

inline const char* bb_topicOpcodeName(uint8_t opcode) {
    switch (opcode) {
        case BEEBUS_TOP_PUBLISH:      return "PUBLISH";
        case BEEBUS_TOP_WRITE:        return "WRITE";
        case BEEBUS_TOP_READ:         return "READ";
        case BEEBUS_TOP_READ_REPLY:   return "READ_REPLY";
        case BEEBUS_TOP_SUBSCRIBE:    return "SUBSCRIBE";
        case BEEBUS_TOP_SUBACK:       return "SUBACK";
        case BEEBUS_TOP_UNSUBSCRIBE:  return "UNSUBSCRIBE";
        case BEEBUS_TOP_LIST_REQUEST: return "LIST_REQUEST";
        case BEEBUS_TOP_LIST_REPLY:   return "LIST_REPLY";
        case BEEBUS_TOP_REGISTER:     return "REGISTER";
        case BEEBUS_TOP_ERROR:        return "ERROR";
        case BEEBUS_TOP_STREAM_ACK:   return "STREAM_ACK";
        case BEEBUS_TOP_RESULT:       return "RESULT";
        case BEEBUS_TOP_SESSION:      return "SESSION";
    }
    return "?";
}

// Whether an opcode asks for an answer, so the operation stays queued after
// transmission. The topic family's bb_isReadFunction.
inline bool bb_topicExpectsReply(uint8_t opcode) {
    return opcode == BEEBUS_TOP_READ ||
           opcode == BEEBUS_TOP_SUBSCRIBE ||
           opcode == BEEBUS_TOP_LIST_REQUEST;
}

// --- Refusal reasons, carried by ERROR and SUBACK ---
#define BEEBUS_TOP_OK              0x00
#define BEEBUS_TOP_ERR_NO_TOPIC    0x01   // Nothing bound at that ID
#define BEEBUS_TOP_ERR_TYPE        0x02   // Value type differs from the binding
#define BEEBUS_TOP_ERR_READ_ONLY   0x03   // Bound, but peers may not write it
#define BEEBUS_TOP_ERR_MALFORMED   0x04   // PDU did not parse
#define BEEBUS_TOP_ERR_NO_ROOM     0x05   // No subscriber slot or contract left
#define BEEBUS_TOP_ERR_REFUSED     0x06   // The owner refused the value (range, validate)
#define BEEBUS_TOP_ERR_STALE       0x07   // A request made to an earlier instance of the owner

inline const char* bb_topicReasonName(uint8_t reason) {
    switch (reason) {
        case BEEBUS_TOP_OK:            return "ok";
        case BEEBUS_TOP_ERR_NO_TOPIC:  return "no such topic";
        case BEEBUS_TOP_ERR_TYPE:      return "type mismatch";
        case BEEBUS_TOP_ERR_READ_ONLY: return "topic is read only";
        case BEEBUS_TOP_ERR_MALFORMED: return "malformed topic PDU";
        case BEEBUS_TOP_ERR_NO_ROOM:   return "no room for another subscriber";
        case BEEBUS_TOP_ERR_REFUSED:   return "value refused by the owner";
        case BEEBUS_TOP_ERR_STALE:     return "request made to an earlier instance";
    }
    return "unknown reason";
}

// --- Write flags (protocol version 2), the byte after a WRITE's topic ID ---
// REQUEST: a request ID, the writer's session and the session of the owner it
// is made to (0 when not known) follow, and the owner applies that ID once
// however often it arrives. Set on every confirmed write.
// RESULT: the owner answers with a RESULT once it has decided.
#define BEEBUS_WRITE_F_REQUEST     0x01
#define BEEBUS_WRITE_F_RESULT      0x02

// --- Subscribe flags (protocol version 2) ---
#define BEEBUS_SUB_Q_RELIABLE      0x01

// --- What the owner did with a write (RESULT) ---
//
// ACCEPTED: stored as sent (for a command: taken). CHANGED: the owner kept a
// different value (clamped, or put back), which the RESULT carries. REFUSED:
// the owner would not take it and kept the value the RESULT carries. The rest
// say why nothing was done.
#define BEEBUS_OUTCOME_ACCEPTED    0x00
#define BEEBUS_OUTCOME_CHANGED     0x01
#define BEEBUS_OUTCOME_REFUSED     0x02
#define BEEBUS_OUTCOME_READ_ONLY   0x03
#define BEEBUS_OUTCOME_NO_TOPIC    0x04
#define BEEBUS_OUTCOME_TYPE        0x05
#define BEEBUS_OUTCOME_BUSY        0x06   // No room to hold the request: try again
#define BEEBUS_OUTCOME_COUNT       0x07

inline const char* bb_outcomeName(uint8_t outcome) {
    switch (outcome) {
        case BEEBUS_OUTCOME_ACCEPTED:  return "accepted";
        case BEEBUS_OUTCOME_CHANGED:   return "changed";
        case BEEBUS_OUTCOME_REFUSED:   return "refused";
        case BEEBUS_OUTCOME_READ_ONLY: return "read only";
        case BEEBUS_OUTCOME_NO_TOPIC:  return "no such topic";
        case BEEBUS_OUTCOME_TYPE:      return "type mismatch";
        case BEEBUS_OUTCOME_BUSY:      return "busy";
    }
    return "?";
}

// --- STREAM_ACK ---
// The low four bits of its flags byte: how many bytes of missing-mask follow
// (0..8), each bit one sequence number below the highest received.
#define BEEBUS_ACK_MASK_BYTES_MAX  8

// --- Per-topic flags, carried by LIST_REPLY and REGISTER ---
#define BEEBUS_TOPIC_F_WRITABLE    0x01   // Peers may write it
#define BEEBUS_TOPIC_F_BUFFERED    0x02   // Publishes go through a ring

// --- Per-topic flags this node keeps to itself (BeeBusLocalTopic::localFlags) ---
// QUEUED: a peer's write is not stored into the variable; it is only handed to
// the written handlers, which queue it for the program to take (one a scan).
#define BEEBUS_TOPIC_L_QUEUED      0x01

// A commissioning list entry. The name is present only when the request asked
// for it, and is "" otherwise. It points at the node's own copy, not into the
// entry: a local topic's name for as long as the topic is bound, a discovered
// one's until the peer's list is fetched again or cleared, and a received
// page's (onTopicList) for the length of the handler call. Copy it to keep it.
struct BeeBusTopicEntry {
    uint16_t topicID = 0;
    uint8_t type = 0;
    uint8_t flags = 0;
    const char* name = "";
};

// A list page is capped in bytes rather than entries, so it still leaves room
// in the frame for other sections.
#define BEEBUS_TOPIC_LIST_PAGE_BYTES  200
#define BEEBUS_TOPIC_LIST_MAX_ENTRIES 32

// Buffered samples released per token.
#define BEEBUS_TOPIC_FLUSH_PER_TURN   4

// --- Data quality ---
//
// Whether a value can be acted on.
enum class BeeBusQuality : uint8_t {
    Unknown = 0,   // Never received
    Good,          // Within the contract, publisher in the ring
    Uncertain,     // Late, within twice the contract
    Bad,           // Beyond that, or the publisher has left the ring
};

// `age` is how long ago the value arrived, `maxInterval` what the publisher
// undertook to report within. An interval of 0 promises nothing, so membership
// is the whole answer.
inline BeeBusQuality bb_topicQuality(bool received, bool publisherAlive,
                                     bb_time_t age, bb_time_t maxInterval) {
    if (!received) return BeeBusQuality::Unknown;
    if (!publisherAlive) return BeeBusQuality::Bad;
    if (maxInterval == 0) return BeeBusQuality::Good;
    if (age <= maxInterval) return BeeBusQuality::Good;
    if (age <= maxInterval * 2) return BeeBusQuality::Uncertain;
    return BeeBusQuality::Bad;
}

inline const char* bb_qualityName(BeeBusQuality quality) {
    switch (quality) {
        case BeeBusQuality::Unknown:   return "Unknown";
        case BeeBusQuality::Good:      return "Good";
        case BeeBusQuality::Uncertain: return "Uncertain";
        case BeeBusQuality::Bad:       return "Bad";
    }
    return "Unknown";
}

// --- Reporting interval ---
//
// Bit 15 clear counts deciseconds, 0.1 s to 54 minutes; bit 15 set counts
// seconds, 1 s to 9.1 hours. 0 is report by exception.
#define BEEBUS_INTERVAL_SECONDS    0x8000
#define BEEBUS_INTERVAL_MAX        0x7FFF

// Rounds up, so a decoded interval never promises sooner than it was set.
inline uint16_t bb_encodeInterval(bb_time_t milliseconds) {
    if (milliseconds == 0) return 0;

    const bb_time_t deciseconds = (milliseconds + 99) / 100;
    if (deciseconds <= BEEBUS_INTERVAL_MAX) {
        return (uint16_t)deciseconds;
    }

    bb_time_t seconds = (milliseconds + 999) / 1000;
    if (seconds > BEEBUS_INTERVAL_MAX) seconds = BEEBUS_INTERVAL_MAX;
    return (uint16_t)(seconds | BEEBUS_INTERVAL_SECONDS);
}

inline bb_time_t bb_decodeInterval(uint16_t coded) {
    if (coded == 0) return 0;
    const bb_time_t count = (bb_time_t)(coded & BEEBUS_INTERVAL_MAX);
    return (coded & BEEBUS_INTERVAL_SECONDS) ? count * 1000u : count * 100u;
}

// --- Age ---
//
// 24 bits of seconds, 194 days, so the field outlives bb_millis()'s 49.7 day
// wrap.
#define BEEBUS_AGE_MAX             0x00FFFFFFu
