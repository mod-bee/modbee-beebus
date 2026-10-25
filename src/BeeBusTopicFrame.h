#pragma once

// For the big-endian helpers, shared with the rest of the wire.
#include "BeeBusTypes.h"
#include "BeeBusTopicTypes.h"

// A topic's text as it arrived: up to BEEBUS_TOPIC_STRING_MAX bytes, held in
// place and always terminated.
class BeeBusText {
public:
    BeeBusText() : _len(0) { _text[0] = '\0'; }
    void assign(const char* text, size_t length) {
        if (length > BEEBUS_TOPIC_STRING_MAX) length = BEEBUS_TOPIC_STRING_MAX;
        if (length) memcpy(_text, text, length);
        _len = (uint16_t)length;
        _text[_len] = '\0';
    }
    const char* data() const { return _text; }
    const char* c_str() const { return _text; }
    size_t size() const { return _len; }
    bool empty() const { return _len == 0; }
    bool operator==(const char* other) const { return other && strcmp(_text, other) == 0; }
private:
    char _text[BEEBUS_TOPIC_STRING_MAX + 1];
    uint16_t _len;
};

// The entries of one list reply: a page is at most BEEBUS_TOPIC_LIST_MAX_ENTRIES.
// An entry built here points at its topic's own name; one parsed from a PDU
// points at a copy held in `_names`, which every name in one received PDU fits
// (each costs its length byte on the wire and its terminator here).
class BeeBusTopicEntryList {
public:
    BeeBusTopicEntryList() : _count(0), _used(0) {}
    BeeBusTopicEntryList(const BeeBusTopicEntryList&) = delete;
    BeeBusTopicEntryList& operator=(const BeeBusTopicEntryList&) = delete;
    size_t size() const { return _count; }
    bool empty() const { return _count == 0; }
    const BeeBusTopicEntry& operator[](size_t i) const { return _entries[i]; }
    BeeBusTopicEntry& operator[](size_t i) { return _entries[i]; }
    bool push_back(const BeeBusTopicEntry& entry) {
        if (_count >= BEEBUS_TOPIC_LIST_MAX_ENTRIES) return false;
        _entries[_count++] = entry;
        return true;
    }
    // The entry, with `length` characters of name copied in.
    bool push_back(const BeeBusTopicEntry& entry, const uint8_t* name, uint8_t length) {
        if ((size_t)_used + length + 1 > sizeof(_names)) return false;
        char* copy = _names + _used;
        memcpy(copy, name, length);
        copy[length] = '\0';
        if (!push_back(entry)) return false;
        _entries[_count - 1].name = copy;
        _used = (uint16_t)(_used + length + 1);
        return true;
    }
    void clear() { _count = 0; _used = 0; }
private:
    BeeBusTopicEntry _entries[BEEBUS_TOPIC_LIST_MAX_ENTRIES];
    char _names[BEEBUS_MAX_RX_BUFFER];
    uint8_t _count;
    uint16_t _used;
};

// A parsed topic PDU. Which fields carry anything depends on the opcode; the
// rest keep their defaults.
struct BeeBusTopicPdu {
    // Back to a freshly made PDU, field by field: the text and the entries are
    // held in place and large, so a whole new one is not built to clear them.
    void reset() {
        opcode = 0; topicID = 0; value = BeeBusScalar(); seq = 0; ageSeconds = 0;
        aged = false; reliable = false; hasValue = false; flags = 0; requestID = 0;
        session = 0; peerSession = 0; missing = 0; text.assign("", 0); deadband = 0.0f; interval = 0;
        status = 0; cursor = 0; maxEntries = 0; wantNames = false; more = false;
        hasNames = false; nextCursor = 0; entries.clear();
    }

    uint8_t opcode = 0;
    uint16_t topicID = 0;

    // PUBLISH, WRITE, READ_REPLY, SUBACK (when hasValue), RESULT
    BeeBusScalar value;
    uint8_t seq = 0;          // STREAM_ACK: the highest sequence received
    uint32_t ageSeconds = 0;
    bool aged = false;
    bool reliable = false;    // The seq is the sender's RELIABLE stream to us
    bool hasValue = false;    // SUBACK: the current value travels with it

    // SUBSCRIBE (QoS), WRITE (BEEBUS_WRITE_F_*), STREAM_ACK (mask bytes)
    uint8_t flags = 0;
    // WRITE with BEEBUS_WRITE_F_REQUEST, RESULT
    uint16_t requestID = 0;
    // SUBSCRIBE, SUBACK, WRITE with a request, SESSION: the sender's session
    uint16_t session = 0;
    // WRITE with a request: the session of the owner it was made to, 0 unknown
    uint16_t peerSession = 0;
    // STREAM_ACK: bit i set = sequence (seq - 1 - i) never arrived
    uint64_t missing = 0;

    // The value when it is a string, `value.type` being BEEBUS_T_STRING. Held
    // rather than pointed at: a pointer into the receive buffer would be
    // zero-copy but would dangle for anything that kept it. Held in place, so
    // nothing is allocated.
    BeeBusText text;

    // SUBSCRIBE
    float deadband = 0.0f;
    uint16_t interval = 0;

    // SUBACK, ERROR; RESULT: the outcome (BEEBUS_OUTCOME_*)
    uint8_t status = 0;

    // LIST_REQUEST
    uint16_t cursor = 0;
    uint8_t maxEntries = 0;
    bool wantNames = false;

    // LIST_REPLY
    bool more = false;
    bool hasNames = false;
    uint16_t nextCursor = 0;

    // LIST_REPLY and REGISTER
    BeeBusTopicEntryList entries;
};

// Codec for the topic PDU family. Pure functions over byte buffers, holding no
// state and knowing nothing about nodes or maps.
class BeeBusTopicFrame {
public:
    // --- Varint ---
    //
    // 0nnnnnnn is 0..127 in one byte; 1nnnnnnn nnnnnnnn is the low seven bits
    // of the high byte followed by eight more, giving 0..32767. The short form
    // is canonical: the long form of a value under 128 is refused, so a topic
    // has exactly one encoding.
    static uint8_t packVarint(uint8_t* out, uint16_t available, uint16_t value);
    static uint8_t unpackVarint(const uint8_t* in, uint16_t available, uint16_t& value);
    // False when the value has no encoding, having appended nothing.
    static bool appendVarint(BeeBusBytes& out, uint16_t value);

    // --- Scalars ---
    // Big-endian. Returns bytes moved, or 0 if the type is unknown or the
    // buffer too small.
    static uint8_t packScalar(uint8_t* out, uint16_t available, const BeeBusScalar& value);
    static uint8_t unpackScalar(const uint8_t* in, uint16_t available, uint8_t type,
                                BeeBusScalar& value);
    static void appendScalar(BeeBusBytes& out, const BeeBusScalar& value);

    // --- Strings ---
    // One length byte, then that many characters, with no terminator. The
    // length byte is what caps a value at BEEBUS_TOPIC_STRING_MAX.
    static void appendText(BeeBusBytes& out, const char* text, uint8_t length);

    // --- Age ---
    // 24 bits of seconds, big-endian. Saturates rather than wrapping, since a
    // wrapped age would read as fresh.
    static void appendAge(BeeBusBytes& out, uint32_t seconds);
    static uint32_t unpackAge(const uint8_t* in);

    // --- Parsing ---
    //
    // Strict in both directions: a PDU that runs out of bytes is refused, and
    // so is one with bytes left over. A trailing byte means the two ends
    // disagree about the layout.
    static bool parse(const uint8_t* pdu, uint16_t length, BeeBusTopicPdu& out);

    // --- Building ---
    // Each appends a complete PDU. False when a field cannot be expressed.
    // `typeFlags` is ORed into the type byte: BEEBUS_T_RELIABLE.
    static bool buildPublish(BeeBusBytes& out, uint8_t opcode, uint16_t topicID,
                             uint8_t seq, const BeeBusScalar& value,
                             bool aged, uint32_t ageSeconds, uint8_t typeFlags = 0);
    // `flags` is BEEBUS_WRITE_F_*; the request ID and session travel only
    // with BEEBUS_WRITE_F_REQUEST.
    static bool buildWrite(BeeBusBytes& out, uint16_t topicID,
                           const BeeBusScalar& value, uint8_t flags = 0,
                           uint16_t requestID = 0, uint16_t session = 0,
                           uint16_t peerSession = 0);

    // The string forms of the two above. Kept apart from the scalar builders
    // rather than folded in, so neither path can silently accept the other's
    // value and emit a PDU nothing will parse.
    static bool buildPublishString(BeeBusBytes& out, uint8_t opcode,
                                   uint16_t topicID, uint8_t seq,
                                   const char* text, uint8_t length,
                                   bool aged, uint32_t ageSeconds, uint8_t typeFlags = 0);
    static bool buildWriteString(BeeBusBytes& out, uint16_t topicID,
                                 const char* text, uint8_t length, uint8_t flags = 0,
                                 uint16_t requestID = 0, uint16_t session = 0,
                                 uint16_t peerSession = 0);
    static bool buildRead(BeeBusBytes& out, uint16_t topicID);
    // `qos` is BEEBUS_SUB_Q_*; `session` the subscriber's.
    static bool buildSubscribe(BeeBusBytes& out, uint16_t topicID,
                               float deadband, uint16_t interval,
                               uint8_t qos = 0, uint16_t session = 0);
    // A refusal, or an acceptance without a value (a buffered topic, whose
    // values come out of its ring).
    static bool buildSuback(BeeBusBytes& out, uint16_t topicID,
                            uint8_t status, uint16_t session = 0, uint8_t type = 0);
    // An acceptance, carrying the topic's current value (a string topic's
    // `text` when the value's type is BEEBUS_T_STRING) under `seq`.
    static bool buildSubackValue(BeeBusBytes& out, uint16_t topicID, uint16_t session,
                                 uint8_t seq, const BeeBusScalar& value,
                                 const char* text, uint8_t length, uint8_t typeFlags);
    // The owner's answer to a write that asked: outcome and the value kept.
    static bool buildResult(BeeBusBytes& out, uint16_t topicID, uint16_t requestID,
                            uint8_t outcome, const BeeBusScalar& value,
                            const char* text, uint8_t length);
    // `missingBytes` of `missing` (low byte first) follow the highest seq.
    static bool buildStreamAck(BeeBusBytes& out, uint8_t highest, uint64_t missing,
                               uint8_t missingBytes);
    static bool buildSession(BeeBusBytes& out, uint16_t session);
    static bool buildUnsubscribe(BeeBusBytes& out, uint16_t topicID);
    static bool buildListRequest(BeeBusBytes& out, uint16_t cursor,
                                 uint8_t maxEntries, bool wantNames);
    static bool buildListReply(BeeBusBytes& out, bool more, bool withNames,
                               uint16_t nextCursor,
                               const BeeBusTopicEntryList& entries);
    static bool buildRegister(BeeBusBytes& out, const BeeBusTopicEntry& entry);
    static bool buildError(BeeBusBytes& out, uint16_t topicID, uint8_t reason);

    // Bytes one entry occupies, so a page can be filled to a byte budget
    // without building it first.
    static uint16_t entrySize(const BeeBusTopicEntry& entry, bool withNames);
};
