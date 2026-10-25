#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// The application-facing half of the topic layer. Every call resolves into the
// topic map or into BeeBusProtocol's outbound helpers.

// --- Topics this node owns ---
bool BeeBusAPI::addTopicRaw(uint16_t topicID, uint8_t type, void* variable, bool writable,
                            const char* name, uint16_t textCapacity) {
    BEEBUS_GUARD();
    if (!_protocol || !variable) return false;
    return _protocol->getTopicMap().addLocal(topicID, type, variable, writable, name,
                                             textCapacity);
}

bool BeeBusAPI::rebindTopic(uint16_t topicID, void* variable) {
    BEEBUS_GUARD();
    if (!_protocol || !variable) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;
    map.localAt(index).var = variable;
    return true;
}

bool BeeBusAPI::rebindRemoteTopic(uint8_t nodeID, uint16_t topicID, void* variable) {
    BEEBUS_GUARD();
    if (!_protocol || !variable) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;
    map.remoteAt(index).var = variable;
    return true;
}

// --- Text topics ---
//
// Every one of these forwards to the same raw binding as a scalar does, with
// the buffer's capacity; the type is stated rather than deduced.
#if BEEBUS_TOPIC_NAMES
bool BeeBusAPI::addTopic(const char* name, uint16_t topicID, char* text, uint16_t capacity) {
    BEEBUS_GUARD();
    return addTopicRaw(topicID, BEEBUS_T_STRING, (void*)text, true, name, capacity);
}

bool BeeBusAPI::addInputTopic(const char* name, uint16_t topicID, char* text,
                              uint16_t capacity) {
    BEEBUS_GUARD();
    return addTopicRaw(topicID, BEEBUS_T_STRING, (void*)text, false, name, capacity);
}

bool BeeBusAPI::registerTopic(uint8_t nodeID, const char* name, uint16_t topicID,
                              char* text, uint16_t capacity) {
    BEEBUS_GUARD();
    return registerTopicRaw(nodeID, topicID, BEEBUS_T_STRING, (void*)text, name, capacity);
}
#endif

bool BeeBusAPI::addTopic(uint16_t topicID, char* text, uint16_t capacity) {
    BEEBUS_GUARD();
    return addTopicRaw(topicID, BEEBUS_T_STRING, (void*)text, true, nullptr, capacity);
}

bool BeeBusAPI::addInputTopic(uint16_t topicID, char* text, uint16_t capacity) {
    BEEBUS_GUARD();
    return addTopicRaw(topicID, BEEBUS_T_STRING, (void*)text, false, nullptr, capacity);
}

bool BeeBusAPI::registerTopic(uint8_t nodeID, uint16_t topicID, char* text, uint16_t capacity) {
    BEEBUS_GUARD();
    return registerTopicRaw(nodeID, topicID, BEEBUS_T_STRING, (void*)text, nullptr, capacity);
}

bool BeeBusAPI::removeTopic(uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getTopicMap().removeLocal(topicID);
}

void BeeBusAPI::reserveTopics(uint16_t localCount, uint16_t remoteCount) {
    BEEBUS_GUARD();
    if (!_protocol) return;
    _protocol->getTopicMap().reserveLocal(localCount);
    _protocol->getTopicMap().reserveRemote(remoteCount);
}

// --- Topics belonging to a peer ---
bool BeeBusAPI::registerTopicRaw(uint8_t nodeID, uint16_t topicID, uint8_t type,
                                 void* variable, const char* name, uint16_t textCapacity) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getTopicMap().addRemote(nodeID, topicID, type, variable, name,
                                              textCapacity);
}

bool BeeBusAPI::unregisterTopic(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getTopicMap().removeRemote(nodeID, topicID);
}

// --- Publishing ---
// Capture a sample into a buffered topic's ring. Shared by the three publishes,
// which differ only in where an unbuffered one would have sent it.
BeeBusResult BeeBusAPI::captureForBuffer(uint16_t index, uint16_t topicID,
                                         const BeeBusScalar& value) {
    BEEBUS_GUARD();
    BeeBusTopicMap& map = _protocol->getTopicMap();
    if (bb_topicTypeIsString(map.localAt(index).last.type)) {
        return map.captureStringSample(index, bb_millis()) ? BeeBusResult::Queued
                                                           : BeeBusResult::BufferFull;
    }
    map.localAt(index).last = value;
    return map.pushSample(topicID, value, bb_millis()) ? BeeBusResult::Queued
                                                       : BeeBusResult::BufferFull;
}

BeeBusResult BeeBusAPI::publishAll(uint16_t topicID, BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    // Refused at the call rather than queued: a broadcast is addressed to the
    // ring, nothing acknowledges it, and a confirmed operation that can never
    // retire would sit in the queue until it timed out.
    if (delivery == BeeBusDelivery::Confirmed) return BeeBusResult::InvalidArgument;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;

    const BeeBusScalar value = bb_scalarLoad(map.localAt(index).last.type,
                                             map.localAt(index).var);

    // A buffered topic is published from its ring and nowhere else, so a live
    // value cannot arrive ahead of the history behind it.
    if (map.localAt(index).bufferDepth != 0) {
        const BeeBusResult captured = captureForBuffer(index, topicID, value);
        // Nothing was queued, so nothing will retire and report. The caller is
        // told here instead: from its side the operation IS finished — the
        // sample is held and the protocol owns delivery from now on. Without
        // this a caller waiting on the completion would wait for ever.
        _protocol->raiseTopicComplete(BEEBUS_NODE_BROADCAST, topicID,
                                      BEEBUS_TOP_PUBLISH, captured);
        return captured;
    }

    return _protocol->sendTopicValue(BEEBUS_NODE_BROADCAST, index,
                                     BEEBUS_TOP_PUBLISH, value, bb_millis(), delivery);
}

BeeBusResult BeeBusAPI::publishTo(uint8_t nodeID, uint16_t topicID,
                                  BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;

    const BeeBusScalar value = bb_scalarLoad(map.localAt(index).last.type,
                                             map.localAt(index).var);

    if (map.localAt(index).bufferDepth != 0) {
        const BeeBusResult captured = captureForBuffer(index, topicID, value);
        _protocol->raiseTopicComplete(nodeID, topicID, BEEBUS_TOP_PUBLISH, captured);
        return captured;
    }

    return _protocol->sendTopicValue(nodeID, index, BEEBUS_TOP_PUBLISH, value, bb_millis(),
                                     delivery);
}

// --- Reading and writing a peer's topic ---
BeeBusResult BeeBusAPI::readTopic(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    // The reply lands in the bound variable, so without one it has nowhere to
    // go.
    if (_protocol->getTopicMap().remoteIndex(nodeID, topicID) == BEEBUS_TOPIC_NONE) {
        return BeeBusResult::NotMapped;
    }

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildRead(pdu, topicID)) {
        return BeeBusResult::InvalidArgument;
    }

    BeeBusCompletion onComplete;
    if (_readCompleteHandler) {
        onComplete = bb_completion(&BeeBusAPI::readCompleted, this);
    }

    return _protocol->queueTopicOp(nodeID, BEEBUS_TOP_READ, topicID, pdu,
                                   BeeBusDelivery::Unconfirmed, false, onComplete);
}

BeeBusResult BeeBusAPI::queueTopicWrite(uint8_t nodeID, uint16_t topicID,
                                        const BeeBusBytes& pdu,
                                        BeeBusDelivery delivery, bool wantResult) {
    BEEBUS_GUARD();
    BeeBusCompletion onComplete;
    if (wantResult) {
        // The result tracker learns when the owner has it, then the caller.
        onComplete = bb_completion(&BeeBusAPI::resultWriteCompleted, this);
    } else if (_writeCompleteHandler) {
        onComplete = bb_completion(&BeeBusAPI::writeCompleted, this);
    }

    const BeeBusResult result = _protocol->queueTopicOp(nodeID, BEEBUS_TOP_WRITE, topicID, pdu,
                                                        delivery, false, onComplete);
    if (wantResult && bb_accepted(result)) {
        // The value asked for, kept so the decision can be asked for again:
        // a scalar from the PDU (its last bytes after the type byte); a
        // string is not asked again.
        BeeBusTopicPdu& parsed = _protocol->scratchPdu();
        BeeBusScalar asked;
        if (BeeBusTopicFrame::parse(pdu.data(), (uint16_t)pdu.size(), parsed)) asked = parsed.value;
        _protocol->noteResultWanted(nodeID, topicID, asked);
    }
    return result;
}

void BeeBusAPI::resultWriteCompleted(void* ctx, uint8_t nodeID, uint16_t address,
                                     uint8_t function, BeeBusResult result) {
    BeeBusAPI* api = static_cast<BeeBusAPI*>(ctx);
    if (api->_protocol) api->_protocol->noteResultWriteDone(nodeID, address, result);
    writeCompleted(ctx, nodeID, address, function, result);
}

namespace {

// A write is a request (applied once, numbered as it leaves) when it is
// confirmed or asks for a result; a result is asked for only with a request.
uint8_t writeFlags(BeeBusDelivery delivery, bool wantResult) {
    uint8_t flags = 0;
    if (delivery == BeeBusDelivery::Confirmed || wantResult) flags |= BEEBUS_WRITE_F_REQUEST;
    if (wantResult) flags |= BEEBUS_WRITE_F_RESULT;
    return flags;
}

}  // namespace

BeeBusResult BeeBusAPI::writeTopicRaw(uint8_t nodeID, uint16_t topicID,
                                      const BeeBusScalar& value, BeeBusDelivery delivery,
                                      bool wantResult) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    // A result travels back on its own; the request it answers has to arrive,
    // so it is confirmed.
    if (wantResult) delivery = BeeBusDelivery::Confirmed;

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildWrite(pdu, topicID, value, writeFlags(delivery, wantResult), 0,
                                      _protocol->session())) {
        return BeeBusResult::InvalidArgument;
    }
    return queueTopicWrite(nodeID, topicID, pdu, delivery, wantResult);
}

BeeBusResult BeeBusAPI::writeTopicText(uint8_t nodeID, uint16_t topicID, const char* text,
                                       size_t length, BeeBusDelivery delivery, bool wantResult) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!text) return BeeBusResult::InvalidArgument;

    // Refused at the call rather than truncated. The caller passed this text
    // deliberately and can be told it will not fit, which is not true of a
    // bound variable the publish scan happens to read.
    if (length > BEEBUS_TOPIC_STRING_MAX) return BeeBusResult::InvalidArgument;

    if (wantResult) delivery = BeeBusDelivery::Confirmed;

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildWriteString(pdu, topicID, text, (uint8_t)length,
                                            writeFlags(delivery, wantResult), 0,
                                            _protocol->session())) {
        return BeeBusResult::InvalidArgument;
    }
    return queueTopicWrite(nodeID, topicID, pdu, delivery, wantResult);
}

BeeBusResult BeeBusAPI::writeTopic(uint8_t nodeID, uint16_t topicID, const char* text,
                                   BeeBusDelivery delivery, bool wantResult) {
    BEEBUS_GUARD();
    return writeTopicText(nodeID, topicID, text, text ? strlen(text) : 0, delivery, wantResult);
}

// --- Text of a bound topic ---
const char* BeeBusAPI::topicText(uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return "";

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return "";

    const char* text = map.localString(index);
    return text ? text : "";
}

const char* BeeBusAPI::topicText(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return "";

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return "";
    if ((map.remoteAt(index).flags & BEEBUS_REMOTE_F_RECEIVED) == 0) return "";

    const char* text = map.remoteString(index);
    return text ? text : "";
}

// --- Subscriptions ---
BeeBusResult BeeBusAPI::subscribe(uint8_t nodeID, uint16_t topicID, float deadband,
                                  bb_time_t maxIntervalMs, BeeBusQoS qos) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;

    // Recorded as it will travel, so quality is judged against what the
    // publisher was asked for rather than against the unrounded request.
    const bb_time_t agreed = bb_decodeInterval(bb_encodeInterval(maxIntervalMs));

    const uint8_t contractId = map.internContract(deadband, agreed);
    if (contractId == BEEBUS_SUB_NONE) return BeeBusResult::QueueFull;

    map.remoteAt(index).contractId = contractId;
    map.remoteAt(index).flags =
        (uint8_t)(map.remoteAt(index).flags &
                  ~(BEEBUS_REMOTE_F_SUBSCRIBED | BEEBUS_REMOTE_F_RELIABLE));
    if (qos == BeeBusQoS::Reliable) {
        map.remoteAt(index).flags =
            (uint8_t)(map.remoteAt(index).flags | BEEBUS_REMOTE_F_RELIABLE);
    }

    return _protocol->issueSubscribe(nodeID, topicID);
}

BeeBusResult BeeBusAPI::unsubscribe(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    if (map.remoteAt(index).contractId == BEEBUS_SUB_NONE) {
        return BeeBusResult::NotSubscribed;
    }

    map.remoteAt(index).contractId = BEEBUS_SUB_NONE;
    map.remoteAt(index).flags =
        (uint8_t)(map.remoteAt(index).flags & ~BEEBUS_REMOTE_F_SUBSCRIBED);

    // This node's last reference to those terms may have just gone.
    map.releaseUnusedContracts();

    // Including one still waiting for the peer to arrive.
    _protocol->dropDeferredSubscribe(nodeID, topicID);

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildUnsubscribe(pdu, topicID)) {
        return BeeBusResult::InvalidArgument;
    }
    return _protocol->queueTopicOp(nodeID, BEEBUS_TOP_UNSUBSCRIBE, topicID, pdu,
                                   BeeBusDelivery::Unconfirmed, false, BeeBusCompletion());
}

// --- Commissioning ---
BeeBusResult BeeBusAPI::fetchTopics(uint8_t nodeID, bool withNames) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

#if !BEEBUS_TOPIC_NAMES
    withNames = false;
#endif

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildListRequest(pdu, 0, BEEBUS_TOPIC_LIST_MAX_ENTRIES,
                                            withNames)) {
        return BeeBusResult::InvalidArgument;
    }
    return _protocol->queueTopicOp(nodeID, BEEBUS_TOP_LIST_REQUEST, 0, pdu,
                                   BeeBusDelivery::Unconfirmed, false, BeeBusCompletion());
}

BeeBusResult BeeBusAPI::publishTopicList(uint8_t nodeID, bool withNames) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

#if !BEEBUS_TOPIC_NAMES
    withNames = false;
#endif

    // The first page only: it carries the cursor for the next, which the peer
    // asks for itself.
    BeeBusBytes pdu;
    uint16_t firstID = 0;
    if (!_protocol->buildTopicListPage(0, BEEBUS_TOPIC_LIST_MAX_ENTRIES, withNames,
                                       pdu, firstID)) {
        return BeeBusResult::InvalidArgument;
    }
    return _protocol->queueTopicOp(nodeID, BEEBUS_TOP_LIST_REPLY, firstID, pdu,
                                   BeeBusDelivery::Unconfirmed, true, BeeBusCompletion());
}

BeeBusResult BeeBusAPI::registerTopicWith(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;

    BeeBusTopicEntry entry;
    entry.topicID = topicID;
    entry.type = map.localAt(index).last.type;
    entry.flags = map.localAt(index).flags;
#if BEEBUS_TOPIC_NAMES
    if (map.localAt(index).name) entry.name = map.localAt(index).name;
#endif

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildRegister(pdu, entry)) {
        return BeeBusResult::InvalidArgument;
    }
    return _protocol->queueTopicOp(nodeID, BEEBUS_TOP_REGISTER, topicID, pdu,
                                   BeeBusDelivery::Unconfirmed, false, BeeBusCompletion());
}

// --- Values and freshness ---
bool BeeBusAPI::getTopic(uint16_t topicID, BeeBusScalar& value) {
    BEEBUS_GUARD();
    if (!_protocol) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;

    // A string has no scalar form, and bb_scalarLoad has no case for one, so
    // reporting success here would hand a generic caller a confident zero for
    // every text topic. Refused instead, which sends it to topicText().
    if (bb_topicTypeIsString(map.localAt(index).last.type)) {
        return false;
    }

    value = bb_scalarLoad(map.localAt(index).last.type, map.localAt(index).var);
    return true;
}

bool BeeBusAPI::getRemoteTopic(uint8_t nodeID, uint16_t topicID, BeeBusScalar& value) {
    BEEBUS_GUARD();
    if (!_protocol) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;
    if ((map.remoteAt(index).flags & BEEBUS_REMOTE_F_RECEIVED) == 0) return false;

    // As above: a text topic is read through topicText(), not through here.
    if (bb_topicTypeIsString(map.remoteAt(index).type)) return false;

    value = bb_scalarLoad(map.remoteAt(index).type, map.remoteAt(index).var);
    return true;
}

BeeBusQuality BeeBusAPI::topicQuality(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusQuality::Unknown;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return BeeBusQuality::Unknown;

    const BeeBusRemoteTopic& topic = map.remoteAt(index);
    const bool received = (topic.flags & BEEBUS_REMOTE_F_RECEIVED) != 0;
    const bb_time_t age = received ? bb_elapsed(topic.rxAt, bb_millis()) : 0;

    return bb_topicQuality(received, _protocol->isAlive(nodeID), age,
                           map.contract(topic.contractId).maxInterval);
}

bb_time_t BeeBusAPI::topicAge(uint8_t nodeID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return BEEBUS_SILENCE_NEVER;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return BEEBUS_SILENCE_NEVER;
    if ((map.remoteAt(index).flags & BEEBUS_REMOTE_F_RECEIVED) == 0) {
        return BEEBUS_SILENCE_NEVER;
    }
    return bb_elapsed(map.remoteAt(index).rxAt, bb_millis());
}

bool BeeBusAPI::topicStream(uint8_t nodeID, uint16_t topicID, BeeBusTopicStream& out) {
    BEEBUS_GUARD();
    if (!_protocol) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;

    const BeeBusRemoteTopic& topic = map.remoteAt(index);
    out.received = topic.rxCount;
    out.missed = topic.missedCount;
    return true;
}

bool BeeBusAPI::setTopicQueued(uint16_t topicID, bool queued) {
    BEEBUS_GUARD();
    if (!_protocol) return false;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return false;
    BeeBusLocalTopic& t = map.localAt(index);
    t.localFlags = (uint8_t)(queued ? (t.localFlags | BEEBUS_TOPIC_L_QUEUED)
                                    : (t.localFlags & ~BEEBUS_TOPIC_L_QUEUED));
    return true;
}

uint16_t BeeBusAPI::topicWrites(uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return 0;

    BeeBusTopicMap& map = _protocol->getTopicMap();
    const uint16_t index = map.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return 0;
    return map.localAt(index).writeCount;
}

bb_time_t BeeBusAPI::silenceFor(uint8_t nodeID) {
    BEEBUS_GUARD();
    return _protocol ? _protocol->silenceFor(nodeID) : BEEBUS_SILENCE_NEVER;
}

bool BeeBusAPI::isAlive(uint8_t nodeID) {
    BEEBUS_GUARD();
    return _protocol ? _protocol->isAlive(nodeID) : false;
}

// --- Store and forward ---
bool BeeBusAPI::bufferTopic(uint16_t topicID, uint8_t depth) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getTopicMap().setBuffer(topicID, depth);
}

uint16_t BeeBusAPI::bufferedCount(uint16_t topicID) {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return _protocol->getTopicMap().bufferedCount(topicID);
}

// --- Enumeration ---
uint16_t BeeBusAPI::topicCount() {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return (uint16_t)_protocol->getTopicMap().localCount();
}

bool BeeBusAPI::topicAt(uint16_t index, BeeBusTopicEntry& entry) {
    BEEBUS_GUARD();
    if (!_protocol || index >= _protocol->getTopicMap().localCount()) return false;

    const BeeBusLocalTopic& topic = _protocol->getTopicMap().localAt(index);
    entry = BeeBusTopicEntry();
    entry.topicID = topic.id;
    entry.type = topic.last.type;
    entry.flags = topic.flags;
#if BEEBUS_TOPIC_NAMES
    if (topic.name) entry.name = topic.name;
#endif
    return true;
}

uint16_t BeeBusAPI::discoveredCount(uint8_t nodeID) {
    BEEBUS_GUARD();
#if BEEBUS_TOPIC_NAMES
    if (!_protocol) return 0;

    uint16_t count = 0;
    const BeeBusTopicMap& map = _protocol->getTopicMap();
    for (size_t i = 0; i < map.discoveredCount(); i++) {
        uint32_t key = 0;
        map.discoveredAt(i, key);
        if ((uint8_t)(key >> 16) == nodeID) count++;
    }
    return count;
#else
    (void)nodeID;
    return 0;
#endif
}

bool BeeBusAPI::discoveredAt(uint8_t nodeID, uint16_t index, BeeBusTopicEntry& entry) {
    BEEBUS_GUARD();
#if BEEBUS_TOPIC_NAMES
    if (!_protocol) return false;

    uint16_t seen = 0;
    const BeeBusTopicMap& map = _protocol->getTopicMap();
    for (size_t i = 0; i < map.discoveredCount(); i++) {
        uint32_t key = 0;
        const BeeBusDiscoveredTopic& found = map.discoveredAt(i, key);
        if ((uint8_t)(key >> 16) != nodeID) continue;
        if (seen++ != index) continue;

        entry = BeeBusTopicEntry();
        entry.topicID = (uint16_t)(key & 0xFFFF);
        entry.type = found.type;
        entry.flags = found.flags;
        if (found.name) entry.name = found.name;
        return true;
    }
    return false;
#else
    (void)nodeID;
    (void)index;
    (void)entry;
    return false;
#endif
}

// --- By name ---
//
// Each resolves the name and forwards. A local name is looked up among this
// node's own topics, a remote one among the peer bindings this node made and
// then among whatever fetchTopics discovered.
#if BEEBUS_TOPIC_NAMES
uint16_t BeeBusAPI::topicIdOf(const char* name) {
    BEEBUS_GUARD();
    if (!_protocol) return BEEBUS_TOPIC_NONE;
    return _protocol->getTopicMap().localByName(name);
}

uint16_t BeeBusAPI::remoteTopicIdOf(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    if (!_protocol) return BEEBUS_TOPIC_NONE;
    return _protocol->getTopicMap().remoteByName(nodeID, name);
}

BeeBusResult BeeBusAPI::publishAll(const char* name, BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return publishAll(topicID, delivery);
}

BeeBusResult BeeBusAPI::publishTo(uint8_t nodeID, const char* name, BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return publishTo(nodeID, topicID, delivery);
}

BeeBusResult BeeBusAPI::readTopic(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return readTopic(nodeID, topicID);
}

BeeBusResult BeeBusAPI::subscribe(uint8_t nodeID, const char* name, float deadband,
                                  bb_time_t maxIntervalMs, BeeBusQoS qos) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return subscribe(nodeID, topicID, deadband, maxIntervalMs, qos);
}

BeeBusResult BeeBusAPI::unsubscribe(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return unsubscribe(nodeID, topicID);
}

BeeBusResult BeeBusAPI::registerTopicWith(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return registerTopicWith(nodeID, topicID);
}

bool BeeBusAPI::removeTopic(const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    return topicID != BEEBUS_TOPIC_NONE && removeTopic(topicID);
}

bool BeeBusAPI::unregisterTopic(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    return topicID != BEEBUS_TOPIC_NONE && unregisterTopic(nodeID, topicID);
}

bool BeeBusAPI::getTopic(const char* name, BeeBusScalar& value) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    return topicID != BEEBUS_TOPIC_NONE && getTopic(topicID, value);
}

bool BeeBusAPI::getRemoteTopic(uint8_t nodeID, const char* name, BeeBusScalar& value) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    return topicID != BEEBUS_TOPIC_NONE && getRemoteTopic(nodeID, topicID, value);
}

BeeBusQuality BeeBusAPI::topicQuality(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusQuality::Unknown;
    return topicQuality(nodeID, topicID);
}

bb_time_t BeeBusAPI::topicAge(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BEEBUS_SILENCE_NEVER;
    return topicAge(nodeID, topicID);
}

BeeBusResult BeeBusAPI::writeTopic(uint8_t nodeID, const char* name, const char* text,
                                   BeeBusDelivery delivery, bool wantResult) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
    return writeTopic(nodeID, topicID, text, delivery, wantResult);
}

const char* BeeBusAPI::topicText(const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    return topicID == BEEBUS_TOPIC_NONE ? "" : topicText(topicID);
}

const char* BeeBusAPI::topicText(uint8_t nodeID, const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = remoteTopicIdOf(nodeID, name);
    return topicID == BEEBUS_TOPIC_NONE ? "" : topicText(nodeID, topicID);
}

bool BeeBusAPI::bufferTopic(const char* name, uint8_t depth) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    return topicID != BEEBUS_TOPIC_NONE && bufferTopic(topicID, depth);
}

uint16_t BeeBusAPI::bufferedCount(const char* name) {
    BEEBUS_GUARD();
    const uint16_t topicID = topicIdOf(name);
    return topicID == BEEBUS_TOPIC_NONE ? 0 : bufferedCount(topicID);
}

BeeBusResult BeeBusAPI::subscribeAll(uint8_t nodeID, const char* pattern, float deadband,
                                     bb_time_t maxIntervalMs, BeeBusQoS qos) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!pattern) return BeeBusResult::InvalidArgument;

    BeeBusTopicMap& map = _protocol->getTopicMap();

    // Every match takes the same terms, so they are interned once.
    const bb_time_t agreed = bb_decodeInterval(bb_encodeInterval(maxIntervalMs));
    const uint8_t contractId = map.internContract(deadband, agreed);
    if (contractId == BEEBUS_SUB_NONE) return BeeBusResult::QueueFull;

    uint16_t matched = 0;
    for (size_t i = 0; i < map.discoveredCount(); i++) {
        uint32_t key = 0;
        const BeeBusDiscoveredTopic& found = map.discoveredAt(i, key);
        if ((uint8_t)(key >> 16) != nodeID) continue;
        if (!found.name || !bb_globMatch(pattern, found.name)) continue;

        // A match still needs a variable to land in, so an unregistered topic
        // is passed over rather than half subscribed.
        const uint16_t topicID = (uint16_t)(key & 0xFFFF);
        const uint16_t index = map.remoteIndex(nodeID, topicID);
        if (index == BEEBUS_TOPIC_NONE) continue;

        map.remoteAt(index).contractId = contractId;
        map.remoteAt(index).flags =
            (uint8_t)((map.remoteAt(index).flags &
                       ~(BEEBUS_REMOTE_F_SUBSCRIBED | BEEBUS_REMOTE_F_RELIABLE)) |
                      (qos == BeeBusQoS::Reliable ? BEEBUS_REMOTE_F_RELIABLE : 0));

        // Deferred rather than queued: an expansion can be far larger than the
        // operation queue, and is paced over the following tokens.
        _protocol->deferSubscribe(nodeID, topicID);
        matched++;
    }

    return matched > 0 ? BeeBusResult::Queued : BeeBusResult::NotMapped;
}
#endif

// --- Topic events ---
void BeeBusAPI::onTopicChanged(BeeBusTopicChangedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicChanged(handler);
}

void BeeBusAPI::onTopicWritten(BeeBusTopicWrittenHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicWritten(handler);
}

void BeeBusAPI::onTopicStringChanged(BeeBusTopicStringChangedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicStringChanged(handler);
}

void BeeBusAPI::onTopicStringWritten(BeeBusTopicStringWrittenHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicStringWritten(handler);
}

void BeeBusAPI::onTopicGap(BeeBusTopicGapHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicGap(handler);
}

void BeeBusAPI::onTopicComplete(BeeBusTopicCompleteHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicComplete(handler);
}

void BeeBusAPI::onTopicList(BeeBusTopicListHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicList(handler);
}

// --- Results and layer 3 ---
void BeeBusAPI::onTopicResult(BeeBusTopicResultHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicResult(handler);
}

void BeeBusAPI::topicApplied(uint16_t topicID, uint16_t settleMs, bool command) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->noteTopicApplied(topicID, settleMs, command);
}

bool BeeBusAPI::topicResult(uint8_t nodeID, uint16_t topicID, uint8_t& outcome,
                            BeeBusScalar& kept, uint16_t& count) {
    BEEBUS_GUARD();
    return _protocol && _protocol->topicResult(nodeID, topicID, outcome, kept, count);
}

void BeeBusAPI::onTopicDecided(BeeBusTopicDecidedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onTopicDecided(handler);
}

bool BeeBusAPI::serveRaw(uint16_t topicID, uint8_t type, void* variable,
                         const BeeBusServe& terms, const char* name, uint16_t textCapacity) {
    BEEBUS_GUARD();
    if (!_protocol || !variable) return false;
    return _protocol->serve(topicID, type, variable, terms, name, textCapacity);
}

bool BeeBusAPI::serve(uint16_t topicID, char* text, uint16_t capacity, const BeeBusServe& terms) {
    BEEBUS_GUARD();
    return serveRaw(topicID, BEEBUS_T_STRING, (void*)text, terms, nullptr, capacity);
}

bool BeeBusAPI::unserve(uint16_t topicID) {
    BEEBUS_GUARD();
    return _protocol && _protocol->unserve(topicID);
}

bool BeeBusAPI::bindRaw(uint8_t ownerID, uint16_t topicID, uint8_t type, void* variable,
                        const BeeBusBindTerms& terms, const char* name, uint16_t textCapacity) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->bind(ownerID, topicID, type, variable, terms, name, textCapacity);
}

bool BeeBusAPI::bind(uint8_t ownerID, uint16_t topicID, char* text, uint16_t capacity,
                     const BeeBusBindTerms& terms) {
    BEEBUS_GUARD();
    return bindRaw(ownerID, topicID, BEEBUS_T_STRING, (void*)text, terms, nullptr, capacity);
}

bool BeeBusAPI::unbind(uint8_t ownerID, uint16_t topicID) {
    BEEBUS_GUARD();
    return _protocol && _protocol->unbind(ownerID, topicID);
}

BeeBusBindState BeeBusAPI::bindState(uint8_t ownerID, uint16_t topicID) {
    BEEBUS_GUARD();
    return _protocol ? _protocol->bindState(ownerID, topicID) : BeeBusBindState::Offline;
}

bool BeeBusAPI::bindResult(uint8_t ownerID, uint16_t topicID, uint8_t& outcome,
                           BeeBusScalar& kept) {
    BEEBUS_GUARD();
    return _protocol && _protocol->bindResult(ownerID, topicID, outcome, kept);
}

void BeeBusAPI::bindChanged(uint8_t ownerID, uint16_t topicID) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->bindChanged(ownerID, topicID);
}

void BeeBusAPI::onBindState(BeeBusBindStateHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) _protocol->onBindState(handler);
}

uint16_t BeeBusAPI::session() {
    BEEBUS_GUARD();
    return _protocol ? _protocol->session() : 0;
}
