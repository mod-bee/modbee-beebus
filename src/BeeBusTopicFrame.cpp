#include "BeeBusTopicFrame.h"
#include "BeeBusNoExcept.h"

namespace {

// A cursor over a PDU being parsed. Every read is bounds checked, so parse()
// expresses the layout and nothing else.
class Reader {
public:
    Reader(const uint8_t* data, uint16_t length) : _data(data), _length(length), _pos(0) {}

    bool byte(uint8_t& out) {
        if (_pos >= _length) return false;
        out = _data[_pos++];
        return true;
    }

    bool be16(uint16_t& out) {
        if ((uint16_t)(_pos + 2) > _length) return false;
        out = bb_unpackBE16(&_data[_pos]);
        _pos = (uint16_t)(_pos + 2);
        return true;
    }

    bool varint(uint16_t& out) {
        const uint8_t used = BeeBusTopicFrame::unpackVarint(_data + _pos,
                                                           (uint16_t)(_length - _pos), out);
        if (used == 0) return false;
        _pos = (uint16_t)(_pos + used);
        return true;
    }

    bool scalar(uint8_t type, BeeBusScalar& out) {
        const uint8_t used = BeeBusTopicFrame::unpackScalar(_data + _pos,
                                                           (uint16_t)(_length - _pos),
                                                           type, out);
        if (used == 0) return false;
        _pos = (uint16_t)(_pos + used);
        return true;
    }

    bool age(uint32_t& out) {
        if ((uint16_t)(_pos + 3) > _length) return false;
        out = BeeBusTopicFrame::unpackAge(_data + _pos);
        _pos = (uint16_t)(_pos + 3);
        return true;
    }

    // A type byte and the value it announces, string or scalar, and the age
    // the type byte says follows. What a PUBLISH, a READ_REPLY, an accepting
    // SUBACK and a RESULT all end with.
    bool typedValue(BeeBusTopicPdu& out) {
        uint8_t typeByte = 0;
        if (!byte(typeByte)) return false;
        out.aged = (typeByte & BEEBUS_T_AGED) != 0;
        out.reliable = (typeByte & BEEBUS_T_RELIABLE) != 0;
        if (bb_topicTypeIsString(typeByte)) {
            if (!text(out.text)) return false;
            out.value.type = BEEBUS_T_STRING;
        } else if (!scalar(typeByte, out.value)) {
            return false;
        }
        return !out.aged || age(out.ageSeconds);
    }

    // A length-prefixed value into a string. The length comes off the wire, so
    // the bound is checked in 32 bit arithmetic that cannot itself wrap.
    bool text(BeeBusText& out) {
        uint8_t length = 0;
        if (!byte(length)) return false;
        if ((uint32_t)_pos + length > _length) return false;
        out.assign((const char*)(_data + _pos), length);
        _pos = (uint16_t)(_pos + length);
        return true;
    }

    // A length-prefixed name, added with its entry to `list`, which holds a
    // copy. Refused rather than truncated: half a name would match the wrong
    // topic under a wildcard.
    bool name(BeeBusTopicEntryList& list, const BeeBusTopicEntry& entry) {
        uint8_t length = 0;
        if (!byte(length)) return false;
        if ((uint16_t)(_pos + length) > _length) return false;
        if (!list.push_back(entry, _data + _pos, length)) return false;
        _pos = (uint16_t)(_pos + length);
        return true;
    }

    bool atEnd() const { return _pos == _length; }

private:
    const uint8_t* _data;
    uint16_t _length;
    uint16_t _pos;
};

// One length byte, then the characters without a terminator.
bool appendName(BeeBusBytes& out, const char* name) {
    const size_t length = name ? strnlen(name, BEEBUS_TOPIC_NAME_MAX) : 0;
    if (length >= BEEBUS_TOPIC_NAME_MAX) return false;

    out.push_back((uint8_t)length);
    out.insert(out.end(), name, name + length);
    return true;
}

} // namespace

// --- Varint ---
uint8_t BeeBusTopicFrame::packVarint(uint8_t* out, uint16_t available, uint16_t value) {
    if (!out || value > BEEBUS_TOPIC_ID_MAX) return 0;

    if (value < 0x80) {
        if (available < 1) return 0;
        out[0] = (uint8_t)value;
        return 1;
    }

    if (available < 2) return 0;
    out[0] = (uint8_t)(0x80 | (value >> 8));
    out[1] = (uint8_t)(value & 0xFF);
    return 2;
}

uint8_t BeeBusTopicFrame::unpackVarint(const uint8_t* in, uint16_t available, uint16_t& value) {
    if (!in || available < 1) return 0;

    if ((in[0] & 0x80) == 0) {
        value = in[0];
        return 1;
    }

    if (available < 2) return 0;

    const uint16_t decoded = (uint16_t)(((uint16_t)(in[0] & 0x7F) << 8) | in[1]);

    // Not this encoding. Accepting it would let one topic arrive under two
    // different byte sequences.
    if (decoded < 0x80) return 0;

    value = decoded;
    return 2;
}

bool BeeBusTopicFrame::appendVarint(BeeBusBytes& out, uint16_t value) {
    uint8_t bytes[2];
    const uint8_t used = packVarint(bytes, sizeof(bytes), value);
    if (used == 0) {
        return false;   // Out of range: appending nothing would emit a PDU
    }                   // nothing can parse, under cover of having worked.
    out.insert(out.end(), bytes, bytes + used);
    return true;
}

// --- Scalars ---
uint8_t BeeBusTopicFrame::packScalar(uint8_t* out, uint16_t available,
                                     const BeeBusScalar& value) {
    const uint8_t size = bb_topicTypeSize(value.type);
    if (!out || size == 0 || available < size) return 0;

    switch (size) {
        case 1:
            out[0] = (uint8_t)value.bits;
            break;
        case 2:
            bb_packBE16(out, (uint16_t)value.bits);
            break;
        case 4:
            bb_packBE16(out, (uint16_t)(value.bits >> 16));
            bb_packBE16(out + 2, (uint16_t)(value.bits & 0xFFFF));
            break;
        default:
            // Eight bytes, high word first, as the four-byte types.
            bb_packBE16(out, (uint16_t)(value.high >> 16));
            bb_packBE16(out + 2, (uint16_t)(value.high & 0xFFFF));
            bb_packBE16(out + 4, (uint16_t)(value.bits >> 16));
            bb_packBE16(out + 6, (uint16_t)(value.bits & 0xFFFF));
            break;
    }
    return size;
}

uint8_t BeeBusTopicFrame::unpackScalar(const uint8_t* in, uint16_t available, uint8_t type,
                                       BeeBusScalar& value) {
    const uint8_t size = bb_topicTypeSize(type);
    if (!in || size == 0 || available < size) return 0;

    value.type = (uint8_t)(type & BEEBUS_T_MASK);
    value.high = 0;

    switch (size) {
        case 1:
            value.bits = in[0];
            break;
        case 2:
            value.bits = bb_unpackBE16(in);
            break;
        case 4:
            value.bits = ((uint32_t)bb_unpackBE16(in) << 16) | bb_unpackBE16(in + 2);
            break;
        default:
            value.high = ((uint32_t)bb_unpackBE16(in) << 16) | bb_unpackBE16(in + 2);
            value.bits = ((uint32_t)bb_unpackBE16(in + 4) << 16) | bb_unpackBE16(in + 6);
            break;
    }
    return size;
}

void BeeBusTopicFrame::appendScalar(BeeBusBytes& out, const BeeBusScalar& value) {
    uint8_t bytes[8];
    const uint8_t used = packScalar(bytes, sizeof(bytes), value);
    out.insert(out.end(), bytes, bytes + used);
}

// --- Strings ---
void BeeBusTopicFrame::appendText(BeeBusBytes& out, const char* text, uint8_t length) {
    out.push_back(length);
    if (length > 0) {
        out.insert(out.end(), text, text + length);
    }
}

// --- Age ---
void BeeBusTopicFrame::appendAge(BeeBusBytes& out, uint32_t seconds) {
    if (seconds > BEEBUS_AGE_MAX) seconds = BEEBUS_AGE_MAX;
    out.push_back((uint8_t)(seconds >> 16));
    out.push_back((uint8_t)(seconds >> 8));
    out.push_back((uint8_t)seconds);
}

uint32_t BeeBusTopicFrame::unpackAge(const uint8_t* in) {
    return ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | in[2];
}

// --- Parsing ---
bool BeeBusTopicFrame::parse(const uint8_t* pdu, uint16_t length, BeeBusTopicPdu& out) {
    if (!pdu || length < 1) return false;

    out.reset();
    Reader reader(pdu, length);

    if (!reader.byte(out.opcode)) return false;

    switch (out.opcode) {
        case BEEBUS_TOP_PUBLISH:
        case BEEBUS_TOP_READ_REPLY:
            if (!reader.varint(out.topicID)) return false;
            if (!reader.byte(out.seq)) return false;
            if (!reader.typedValue(out)) return false;
            break;

        case BEEBUS_TOP_WRITE: {
            uint8_t typeByte = 0;
            if (!reader.varint(out.topicID)) return false;
            if (!reader.byte(out.flags)) return false;
            if (out.flags & BEEBUS_WRITE_F_REQUEST) {
                if (!reader.be16(out.requestID) || !reader.be16(out.session) ||
                    !reader.be16(out.peerSession)) {
                    return false;
                }
            }
            if (!reader.byte(typeByte)) return false;
            // A write is never aged or streamed: those bits mean nothing here.
            if (typeByte & (BEEBUS_T_AGED | BEEBUS_T_RELIABLE)) return false;
            if (bb_topicTypeIsString(typeByte)) {
                if (!reader.text(out.text)) return false;
                out.value.type = BEEBUS_T_STRING;
            } else if (!reader.scalar(typeByte, out.value)) {
                return false;
            }
            break;
        }

        case BEEBUS_TOP_RESULT:
            if (!reader.varint(out.topicID)) return false;
            if (!reader.be16(out.requestID)) return false;
            if (!reader.byte(out.status)) return false;
            if (out.status >= BEEBUS_OUTCOME_COUNT) return false;
            if (!reader.typedValue(out)) return false;
            break;

        case BEEBUS_TOP_STREAM_ACK: {
            if (!reader.byte(out.seq)) return false;
            if (!reader.byte(out.flags)) return false;
            const uint8_t bytes = (uint8_t)(out.flags & 0x0F);
            if (bytes > BEEBUS_ACK_MASK_BYTES_MAX) return false;
            for (uint8_t i = 0; i < bytes; i++) {
                uint8_t part = 0;
                if (!reader.byte(part)) return false;
                out.missing |= (uint64_t)part << (8 * i);
            }
            break;
        }

        case BEEBUS_TOP_SESSION:
            if (!reader.be16(out.session)) return false;
            break;

        case BEEBUS_TOP_READ:
        case BEEBUS_TOP_UNSUBSCRIBE:
            if (!reader.varint(out.topicID)) return false;
            break;

        case BEEBUS_TOP_SUBSCRIBE: {
            uint16_t high = 0;
            uint16_t low = 0;
            if (!reader.varint(out.topicID)) return false;
            if (!reader.be16(high) || !reader.be16(low)) return false;
            out.deadband = bb_bitsFloat(((uint32_t)high << 16) | low);
            if (!reader.be16(out.interval)) return false;
            if (!reader.byte(out.flags)) return false;
            if (!reader.be16(out.session)) return false;
            break;
        }

        case BEEBUS_TOP_SUBACK:
            if (!reader.varint(out.topicID)) return false;
            if (!reader.byte(out.status)) return false;
            if (!reader.be16(out.session)) return false;
            // An acceptance carries the current value, so the first value
            // cannot be lost on its own; a buffered topic's comes out of its
            // ring instead, and its answer says none follows.
            if (out.status == BEEBUS_TOP_OK) {
                if (!reader.byte(out.flags)) return false;
                out.hasValue = (out.flags & 0x01) != 0;
                if (out.hasValue) {
                    if (!reader.byte(out.seq)) return false;
                    if (!reader.typedValue(out)) return false;
                } else {
                    // The type alone, so the two ends still agree on it first.
                    uint8_t type = 0;
                    if (!reader.byte(type)) return false;
                    out.value.type = (uint8_t)(type & BEEBUS_T_MASK);
                }
            }
            break;

        case BEEBUS_TOP_LIST_REQUEST: {
            uint8_t limit = 0;
            if (!reader.varint(out.cursor)) return false;
            if (!reader.byte(limit)) return false;

            // The names flag rides in the top bit of the count.
            out.wantNames = (limit & 0x80) != 0;
            out.maxEntries = (uint8_t)(limit & 0x7F);
            break;
        }

        case BEEBUS_TOP_LIST_REPLY: {
            uint8_t flags = 0;
            uint8_t count = 0;
            if (!reader.byte(flags)) return false;
            if (!reader.byte(count)) return false;
            if (!reader.varint(out.nextCursor)) return false;

            out.more = (flags & 0x01) != 0;
            out.hasNames = (flags & 0x02) != 0;

            for (uint8_t i = 0; i < count; i++) {
                BeeBusTopicEntry entry;
                if (!reader.varint(entry.topicID)) return false;
                if (!reader.byte(entry.type)) return false;
                if (!reader.byte(entry.flags)) return false;
                // More than a page holds is not something this build sends,
                // and is refused rather than partly read.
                if (out.hasNames ? !reader.name(out.entries, entry)
                                 : !out.entries.push_back(entry)) {
                    return false;
                }
            }
            break;
        }

        case BEEBUS_TOP_REGISTER: {
            BeeBusTopicEntry entry;
            if (!reader.varint(entry.topicID)) return false;
            if (!reader.byte(entry.type)) return false;
            if (!reader.byte(entry.flags)) return false;
            if (!reader.name(out.entries, entry)) return false;
            out.topicID = entry.topicID;
            break;
        }

        case BEEBUS_TOP_ERROR:
            if (!reader.varint(out.topicID)) return false;
            if (!reader.byte(out.status)) return false;
            break;

        default:
            return false;
    }

    // Bytes left over mean the sender used a layout this build does not have.
    return reader.atEnd();
}

// --- Building ---
bool BeeBusTopicFrame::buildPublish(BeeBusBytes& out, uint8_t opcode,
                                    uint16_t topicID, uint8_t seq,
                                    const BeeBusScalar& value,
                                    bool aged, uint32_t ageSeconds, uint8_t typeFlags) {
    if (topicID > BEEBUS_TOPIC_ID_MAX || bb_topicTypeSize(value.type) == 0) return false;
    if (opcode != BEEBUS_TOP_PUBLISH && opcode != BEEBUS_TOP_READ_REPLY) return false;

    out.push_back(opcode);
    (void)appendVarint(out, topicID);
    out.push_back(seq);
    out.push_back((uint8_t)((aged ? (value.type | BEEBUS_T_AGED) : value.type) |
                            (typeFlags & BEEBUS_T_RELIABLE)));
    appendScalar(out, value);
    if (aged) appendAge(out, ageSeconds);
    return true;
}

namespace {

// The flags byte of a WRITE, and the request that travels with it.
void appendWriteHead(BeeBusBytes& out, uint16_t topicID, uint8_t flags,
                     uint16_t requestID, uint16_t session, uint16_t peerSession) {
    out.push_back(BEEBUS_TOP_WRITE);
    (void)BeeBusTopicFrame::appendVarint(out, topicID);
    out.push_back(flags);
    if (flags & BEEBUS_WRITE_F_REQUEST) {
        uint8_t field[6];
        bb_packBE16(field, requestID);
        bb_packBE16(field + 2, session);
        bb_packBE16(field + 4, peerSession);
        out.insert(out.end(), field, field + sizeof(field));
    }
}

// A type byte and its value, string or scalar.
bool appendTypedValue(BeeBusBytes& out, const BeeBusScalar& value, const char* text,
                      uint8_t length, uint8_t typeFlags) {
    if (bb_topicTypeIsString(value.type)) {
        if (!text && length > 0) return false;
        out.push_back((uint8_t)(BEEBUS_T_STRING | (typeFlags & BEEBUS_T_RELIABLE)));
        BeeBusTopicFrame::appendText(out, text, length);
        return true;
    }
    if (bb_topicTypeSize(value.type) == 0) return false;
    out.push_back((uint8_t)(value.type | (typeFlags & BEEBUS_T_RELIABLE)));
    BeeBusTopicFrame::appendScalar(out, value);
    return true;
}

}  // namespace

bool BeeBusTopicFrame::buildWrite(BeeBusBytes& out, uint16_t topicID,
                                  const BeeBusScalar& value, uint8_t flags,
                                  uint16_t requestID, uint16_t session, uint16_t peerSession) {
    if (topicID > BEEBUS_TOPIC_ID_MAX || bb_topicTypeSize(value.type) == 0) return false;

    appendWriteHead(out, topicID, flags, requestID, session, peerSession);
    out.push_back(value.type);
    appendScalar(out, value);
    return true;
}

// The string forms. Separate from the two above rather than folded in, so
// neither path can accept the other's value: bb_topicTypeSize reports 0 for a
// string, which is what makes the scalar builders reject one.
bool BeeBusTopicFrame::buildPublishString(BeeBusBytes& out, uint8_t opcode,
                                          uint16_t topicID, uint8_t seq,
                                          const char* text, uint8_t length,
                                          bool aged, uint32_t ageSeconds, uint8_t typeFlags) {
    if (topicID > BEEBUS_TOPIC_ID_MAX || (!text && length > 0)) return false;
    if (opcode != BEEBUS_TOP_PUBLISH && opcode != BEEBUS_TOP_READ_REPLY) return false;

    out.push_back(opcode);
    (void)appendVarint(out, topicID);
    out.push_back(seq);
    out.push_back((uint8_t)((aged ? (BEEBUS_T_STRING | BEEBUS_T_AGED) : BEEBUS_T_STRING) |
                            (typeFlags & BEEBUS_T_RELIABLE)));
    appendText(out, text, length);
    if (aged) appendAge(out, ageSeconds);
    return true;
}

bool BeeBusTopicFrame::buildWriteString(BeeBusBytes& out, uint16_t topicID,
                                        const char* text, uint8_t length, uint8_t flags,
                                        uint16_t requestID, uint16_t session,
                                        uint16_t peerSession) {
    if (topicID > BEEBUS_TOPIC_ID_MAX || (!text && length > 0)) return false;

    appendWriteHead(out, topicID, flags, requestID, session, peerSession);
    out.push_back(BEEBUS_T_STRING);
    appendText(out, text, length);
    return true;
}

bool BeeBusTopicFrame::buildRead(BeeBusBytes& out, uint16_t topicID) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    out.push_back(BEEBUS_TOP_READ);
    (void)appendVarint(out, topicID);
    return true;
}

bool BeeBusTopicFrame::buildSubscribe(BeeBusBytes& out, uint16_t topicID,
                                      float deadband, uint16_t interval,
                                      uint8_t qos, uint16_t session) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    const uint32_t bits = bb_floatBits(deadband);
    uint8_t field[9];
    bb_packBE16(field, (uint16_t)(bits >> 16));
    bb_packBE16(field + 2, (uint16_t)(bits & 0xFFFF));
    bb_packBE16(field + 4, interval);
    field[6] = qos;
    bb_packBE16(field + 7, session);

    out.push_back(BEEBUS_TOP_SUBSCRIBE);
    (void)appendVarint(out, topicID);
    out.insert(out.end(), field, field + sizeof(field));
    return true;
}

bool BeeBusTopicFrame::buildSuback(BeeBusBytes& out, uint16_t topicID,
                                   uint8_t status, uint16_t session, uint8_t type) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    uint8_t field[2];
    bb_packBE16(field, session);
    out.push_back(BEEBUS_TOP_SUBACK);
    (void)appendVarint(out, topicID);
    out.push_back(status);
    out.insert(out.end(), field, field + sizeof(field));
    if (status == BEEBUS_TOP_OK) {
        out.push_back(0);   // No value follows, only its type
        out.push_back((uint8_t)(type & BEEBUS_T_MASK));
    }
    return true;
}

bool BeeBusTopicFrame::buildSubackValue(BeeBusBytes& out, uint16_t topicID, uint16_t session,
                                        uint8_t seq, const BeeBusScalar& value,
                                        const char* text, uint8_t length, uint8_t typeFlags) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    const size_t start = out.size();
    uint8_t field[2];
    bb_packBE16(field, session);
    out.push_back(BEEBUS_TOP_SUBACK);
    (void)appendVarint(out, topicID);
    out.push_back(BEEBUS_TOP_OK);
    out.insert(out.end(), field, field + sizeof(field));
    out.push_back(0x01);   // The value follows
    out.push_back(seq);
    if (!appendTypedValue(out, value, text, length, typeFlags)) {
        out.resize(start);
        return false;
    }
    return true;
}

bool BeeBusTopicFrame::buildResult(BeeBusBytes& out, uint16_t topicID, uint16_t requestID,
                                   uint8_t outcome, const BeeBusScalar& value,
                                   const char* text, uint8_t length) {
    if (topicID > BEEBUS_TOPIC_ID_MAX || outcome >= BEEBUS_OUTCOME_COUNT) return false;

    const size_t start = out.size();
    uint8_t field[2];
    bb_packBE16(field, requestID);
    out.push_back(BEEBUS_TOP_RESULT);
    (void)appendVarint(out, topicID);
    out.insert(out.end(), field, field + sizeof(field));
    out.push_back(outcome);
    if (!appendTypedValue(out, value, text, length, 0)) {
        out.resize(start);
        return false;
    }
    return true;
}

bool BeeBusTopicFrame::buildStreamAck(BeeBusBytes& out, uint8_t highest, uint64_t missing,
                                      uint8_t missingBytes) {
    if (missingBytes > BEEBUS_ACK_MASK_BYTES_MAX) return false;

    out.push_back(BEEBUS_TOP_STREAM_ACK);
    out.push_back(highest);
    out.push_back(missingBytes);
    for (uint8_t i = 0; i < missingBytes; i++) {
        out.push_back((uint8_t)(missing >> (8 * i)));
    }
    return true;
}

bool BeeBusTopicFrame::buildSession(BeeBusBytes& out, uint16_t session) {
    uint8_t field[2];
    bb_packBE16(field, session);
    out.push_back(BEEBUS_TOP_SESSION);
    out.insert(out.end(), field, field + sizeof(field));
    return true;
}

bool BeeBusTopicFrame::buildUnsubscribe(BeeBusBytes& out, uint16_t topicID) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    out.push_back(BEEBUS_TOP_UNSUBSCRIBE);
    (void)appendVarint(out, topicID);
    return true;
}

bool BeeBusTopicFrame::buildListRequest(BeeBusBytes& out, uint16_t cursor,
                                        uint8_t maxEntries, bool wantNames) {
    if (cursor > BEEBUS_TOPIC_ID_MAX || maxEntries == 0 || maxEntries > 0x7F) return false;

    out.push_back(BEEBUS_TOP_LIST_REQUEST);
    (void)appendVarint(out, cursor);
    out.push_back((uint8_t)(wantNames ? (maxEntries | 0x80) : maxEntries));
    return true;
}

bool BeeBusTopicFrame::buildListReply(BeeBusBytes& out, bool more, bool withNames,
                                      uint16_t nextCursor,
                                      const BeeBusTopicEntryList& entries) {
    if (nextCursor > BEEBUS_TOPIC_ID_MAX || entries.size() > 0xFF) return false;

    // Built aside and only handed over once it is whole, so false means `out`
    // was not touched rather than "do not look at what is in `out`". A builder
    // that can fail partway through an entry list would otherwise leave a
    // half-formed PDU behind for a caller that only checked the flag.
    BeeBusBytes pdu;
    pdu.push_back(BEEBUS_TOP_LIST_REPLY);
    pdu.push_back((uint8_t)((more ? 0x01 : 0x00) | (withNames ? 0x02 : 0x00)));
    pdu.push_back((uint8_t)entries.size());
    if (!appendVarint(pdu, nextCursor)) return false;

    for (size_t i = 0; i < entries.size(); i++) {
        const BeeBusTopicEntry& entry = entries[i];
        if (entry.topicID > BEEBUS_TOPIC_ID_MAX) return false;

        if (!appendVarint(pdu, entry.topicID)) return false;
        pdu.push_back(entry.type);
        pdu.push_back(entry.flags);
        if (withNames && !appendName(pdu, entry.name)) return false;
    }

    out.insert(out.end(), pdu.begin(), pdu.end());
    return true;
}

bool BeeBusTopicFrame::buildRegister(BeeBusBytes& out, const BeeBusTopicEntry& entry) {
    if (entry.topicID > BEEBUS_TOPIC_ID_MAX) return false;

    BeeBusBytes pdu;
    pdu.push_back(BEEBUS_TOP_REGISTER);
    if (!appendVarint(pdu, entry.topicID)) return false;
    pdu.push_back(entry.type);
    pdu.push_back(entry.flags);
    if (!appendName(pdu, entry.name)) return false;

    out.insert(out.end(), pdu.begin(), pdu.end());
    return true;
}

bool BeeBusTopicFrame::buildError(BeeBusBytes& out, uint16_t topicID, uint8_t reason) {
    if (topicID > BEEBUS_TOPIC_ID_MAX) return false;

    out.push_back(BEEBUS_TOP_ERROR);
    (void)appendVarint(out, topicID);
    out.push_back(reason);
    return true;
}

uint16_t BeeBusTopicFrame::entrySize(const BeeBusTopicEntry& entry, bool withNames) {
    uint16_t size = (uint16_t)((entry.topicID < 0x80 ? 1 : 2) + 2);
    if (withNames) size = (uint16_t)(size + 1 + (entry.name ? strlen(entry.name) : 0));
    return size;
}
