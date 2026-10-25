#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// The reliable topic layer: RELIABLE streams, sessions, write requests and
// their results. What each piece is for is in BeeBusStreams.h.

namespace {

// Where a value PDU keeps its sequence byte and its type byte: a PUBLISH right
// after the topic ID, an accepting SUBACK after its status and session. False
// when the PDU is not one that carries a streamed value.
bool valueBytes(const uint8_t* pdu, uint16_t length, uint16_t& topicID,
                uint16_t& seqAt, uint16_t& typeAt) {
    if (length < 2) return false;
    const uint8_t used = BeeBusTopicFrame::unpackVarint(pdu + 1, (uint16_t)(length - 1), topicID);
    if (used == 0) return false;
    uint16_t at = (uint16_t)(1 + used);
    if (pdu[0] == BEEBUS_TOP_SUBACK) {
        if (at + 3 >= length || pdu[at] != BEEBUS_TOP_OK || (pdu[at + 3] & 0x01) == 0) {
            return false;
        }
        at = (uint16_t)(at + 4);   // status, session, flags
    } else if (pdu[0] != BEEBUS_TOP_PUBLISH) {
        return false;
    }
    seqAt = at;
    typeAt = (uint16_t)(at + 1);
    return typeAt < length && (pdu[typeAt] & BEEBUS_T_RELIABLE) != 0;
}

// Text is compared by a digest where a written value is held for a read-back:
// a result needs to know whether the text changed, not what it was.
uint32_t textDigest(const char* text, size_t length) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < length; i++) {
        hash = (hash ^ (uint8_t)text[i]) * 16777619u;
    }
    return hash;
}

// How many bytes of a missing-mask are worth sending: up to its highest set bit.
uint8_t maskBytes(uint64_t missing) {
    uint8_t bytes = 0;
    while (missing != 0) {
        bytes++;
        missing >>= 8;
    }
    return bytes;
}

}  // namespace

// --- Peers ---
BeeBusPeer* BeeBusProtocol::peer(uint8_t nodeID, bool create) {
    size_t lo = 0;
    size_t hi = _peers.size();
    while (lo < hi) {
        const size_t mid = (lo + hi) / 2;
        if (_peers[mid].node < nodeID) lo = mid + 1;
        else hi = mid;
    }
    if (lo < _peers.size() && _peers[lo].node == nodeID) return &_peers[lo];
    if (!create || nodeID == 0 || nodeID == _nodeID || nodeID == BEEBUS_NODE_BROADCAST) {
        return nullptr;
    }

    // A record per peer ever heard from, so a table that grows at most to the
    // number of node IDs; it grows as peers appear, a few at a time.
    if (_peers.full() && !_peers.grow(_peers.capacity() + 4)) return nullptr;
    BeeBusPeer fresh;
    fresh.node = nodeID;
    if (!_peers.insertAt(lo, fresh)) return nullptr;
    return &_peers[lo];
}

void BeeBusProtocol::notePeerSession(uint8_t nodeID, uint16_t session) {
    if (session == 0) return;
    BeeBusPeer* known = peer(nodeID, true);
    if (!known) return;
    if (known->session != 0 && known->session != session) {
        BEEBUS_DEBUG_PROTOCOL("SESSION: Node %d restarted (%04X -> %04X)",
            nodeID, (unsigned)known->session, (unsigned)session);
        peerRestarted(nodeID);
        known = peer(nodeID, false);
        if (!known) return;
    }
    known->session = session;
}

void BeeBusProtocol::peerRestarted(uint8_t nodeID) {
    reportError(BeeBusError::MemberJoined, "Peer restarted");

    // What the old instance held of ours went with it, and what we held of
    // its (its subscriptions to us, its topics' sequence) describes nobody.
    // The bindings this application made stay, as on a departure.
    _topicMap.removeNode(nodeID);

    if (BeeBusPeer* known = peer(nodeID, false)) {
        const uint16_t session = known->session;
        *known = BeeBusPeer();
        known->node = nodeID;
        known->session = session;
    }

    for (size_t i = 0; i < _results.size();) {
        if (_results[i].writer == nodeID) _results.eraseAt(i);
        else i++;
    }
    for (size_t i = 0; i < _sentResults.size();) {
        if (_sentResults[i].writer == nodeID) _sentResults.eraseAt(i);
        else i++;
    }

    // Answers still queued for the old instance would reach the new one, whose
    // requests are numbered afresh, and be taken for answers to those.
    for (;;) {
        uint16_t stale = 0;
        bool found = false;
        const BeeBusList<BeeBusPendingModbusOp>& pending = _operations.getPendingOps();
        for (size_t i = 0; i < pending.size() && !found; i++) {
            const BeeBusPendingModbusOp& op = pending[i];
            if (op.destNodeID == nodeID && op.pduType == BEEBUS_PDU_TOPIC &&
                op.req.function == BEEBUS_TOP_RESULT) {
                stale = op.opId;
                found = true;
            }
        }
        if (!found) break;
        _operations.removePendingOperation(nodeID, stale, BeeBusResult::Cancelled);
    }

    // A decision owed by the old instance will not come from the new one, and
    // asking it again would be asking a stranger.
    for (size_t i = 0; i < _resultRecords.size(); i++) {
        BeeBusResultRecord& record = _resultRecords[i];
        if (record.node != nodeID || (record.flags & BEEBUS_RESULT_F_WAITING) == 0) continue;
        record.outcome = BEEBUS_OUTCOME_NONE;
        record.count = (uint16_t)(record.count + 1);
        record.flags = 0;
    }

    // Binds to it start over: its values are a new instance's, and a change
    // that was with the old one is not with this one.
    for (size_t i = 0; i < _bindings.size(); i++) {
        BeeBusBinding& bound = _bindings[i];
        if ((uint8_t)(bound.key >> 16) != nodeID) continue;
        bound.flags = (uint8_t)(bound.flags &
                                ~(BEEBUS_BIND_F_VALUE | BEEBUS_BIND_F_ACKED | BEEBUS_BIND_F_RESEND));
        setBindState(bound, isMember(nodeID) ? BeeBusBindState::Syncing
                                             : BeeBusBindState::Offline);
    }

    // What we subscribe from it is asked for again; our session reaches the
    // new instance with that.
    resubscribeTo(nodeID);
}

void BeeBusProtocol::silentCompletion(void*, uint8_t, uint16_t, uint8_t, BeeBusResult) {}

// --- Numbering what leaves ---
void BeeBusProtocol::stampTopicSection(uint8_t destNodeID, uint8_t* pdu, uint16_t length,
                                       BeeBusModbusRequest* stored) {
    if (!pdu || length < 2 || destNodeID == BEEBUS_NODE_BROADCAST) return;

    // A RESULT carries the owner's value as it leaves, retry or not: it then
    // arrives as the newest word from the owner on that topic, ahead of
    // anything queued after it. A string keeps the text it was built with
    // (its length is already in the frame).
    if (pdu[0] == BEEBUS_TOP_RESULT) {
        uint16_t topicID = 0;
        const uint8_t used = BeeBusTopicFrame::unpackVarint(pdu + 1, (uint16_t)(length - 1),
                                                            topicID);
        const uint16_t typeAt = (uint16_t)(1 + used + 3);   // request ID, outcome
        if (used == 0 || typeAt >= length) return;
        const uint8_t type = (uint8_t)(pdu[typeAt] & BEEBUS_T_MASK);
        const uint8_t size = bb_topicTypeSize(type);
        const uint16_t index = _topicMap.localIndex(topicID);
        if (size == 0 || index == BEEBUS_TOPIC_NONE || typeAt + 1 + size > length) return;
        const BeeBusLocalTopic& topic = _topicMap.localAt(index);
        if (topic.last.type != type) return;
        const BeeBusScalar now = bb_scalarLoad(type, topic.var);
        BeeBusTopicFrame::packScalar(&pdu[typeAt + 1], size, now);
        if (stored && stored->data.size() >= (size_t)typeAt + 1 + size) {
            BeeBusTopicFrame::packScalar(&stored->data[typeAt + 1], size, now);
        }
        return;
    }

    // A write that asked to be applied once: its request ID, given the first
    // time it goes out and kept for its retries.
    if (pdu[0] == BEEBUS_TOP_WRITE) {
        uint16_t topicID = 0;
        const uint8_t used = BeeBusTopicFrame::unpackVarint(pdu + 1, (uint16_t)(length - 1),
                                                            topicID);
        const uint16_t flagsAt = (uint16_t)(1 + used);
        if (used == 0 || flagsAt + 7 > length) return;
        if ((pdu[flagsAt] & BEEBUS_WRITE_F_REQUEST) == 0) return;

        // The owner's session as known now, unless the request already has
        // one: an owner that has restarted since refuses it rather than take a
        // retry of a request its old instance applied for a new one.
        const uint16_t ownerAt = (uint16_t)(flagsAt + 5);
        if (bb_unpackBE16(&pdu[ownerAt]) == 0) {
            const BeeBusPeer* owner = peer(destNodeID, false);
            if (owner && owner->session != 0) {
                bb_packBE16(&pdu[ownerAt], owner->session);
                if (stored && stored->data.size() >= (size_t)ownerAt + 2) {
                    bb_packBE16(&stored->data[ownerAt], owner->session);
                }
            }
        }

        const uint16_t idAt = (uint16_t)(flagsAt + 1);
        uint16_t requestID = bb_unpackBE16(&pdu[idAt]);
        if (requestID == 0) {
            BeeBusPeer* to = peer(destNodeID, true);
            if (!to) return;
            requestID = to->nextRequest;
            to->nextRequest = (uint16_t)(to->nextRequest + 1);
            if (to->nextRequest == 0) to->nextRequest = 1;   // 0 is "not yet numbered"
            bb_packBE16(&pdu[idAt], requestID);
            if (stored && stored->data.size() >= (size_t)idAt + 2) {
                bb_packBE16(&stored->data[idAt], requestID);
            }
            // A bind's change answers to the newest request only: a retry of an
            // older one keeps its own ID and is not taken for it. So does a
            // caller polling topicResult.
            if (BeeBusBinding* bound = binding(destNodeID, topicID)) {
                if (bound->state == BeeBusBindState::Pending) bound->requestID = requestID;
            }
            if ((pdu[flagsAt] & BEEBUS_WRITE_F_RESULT) != 0) {
                if (BeeBusResultRecord* record = resultRecord(destNodeID, topicID, false)) {
                    if (record->flags & BEEBUS_RESULT_F_WAITING) record->requestID = requestID;
                }
            }
        }
        return;
    }

    // A RELIABLE value: the next number on the stream to that peer.
    uint16_t topicID = 0;
    uint16_t seqAt = 0;
    uint16_t typeAt = 0;
    if (!valueBytes(pdu, length, topicID, seqAt, typeAt)) return;

    BeeBusPeer* to = peer(destNodeID, true);
    if (!to) return;
    if (bb_peerInFlight(*to) == 0) to->txQuiet = 0;
    const uint8_t seq = to->txNext;
    to->txNext = (uint8_t)(to->txNext + 1);
    if (to->txQueued > 0) to->txQueued--;
    pdu[seqAt] = seq;

    const uint16_t index = _topicMap.localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE) return;
    const uint16_t slot = _topicMap.slotFor(index, destNodeID, false);
    if (slot != BEEBUS_TOPIC_NONE) _topicMap.noteStreamSent(slot, seq);
}

// --- Receiving a stream ---
bool BeeBusProtocol::acceptStream(uint8_t srcNodeID, uint8_t seq) {
    BeeBusPeer* from = peer(srcNodeID, true);
    if (!from) return true;   // Nothing to track it with: take it, as LIVE would

    bool fresh = true;
    if ((from->flags & BEEBUS_PEER_RX_STARTED) == 0) {
        from->rxHigh = seq;
        from->rxMissing = 0;
        from->flags = (uint8_t)(from->flags | BEEBUS_PEER_RX_STARTED);
    } else {
        const uint8_t ahead = (uint8_t)(seq - from->rxHigh);
        if (ahead == 0 || ahead >= 128) {
            // A copy, or older than what is held. Held back even when it is a
            // number marked missing (a medium that reorders): the sender
            // answers a missing number with the topic's current value, which
            // is newer than this one.
            fresh = false;
        } else {
            uint64_t missing = ahead >= 64 ? 0 : (from->rxMissing << ahead);
            const uint8_t skipped = (uint8_t)(ahead - 1);
            missing |= skipped >= 64 ? ~(uint64_t)0 : (((uint64_t)1 << skipped) - 1);
            from->rxMissing = missing;
            from->rxHigh = seq;
        }
    }

    from->flags = (uint8_t)(from->flags | BEEBUS_PEER_ACK_DUE);
    refreshStreamAck(srcNodeID);
    return fresh;
}

void BeeBusProtocol::refreshStreamAck(uint8_t nodeID) {
    BeeBusPeer* from = peer(nodeID, false);
    if (!from || (from->flags & BEEBUS_PEER_ACK_DUE) == 0) return;

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildStreamAck(pdu, from->rxHigh, from->rxMissing,
                                          maskBytes(from->rxMissing))) {
        return;
    }

    // One answer per peer, rewritten while it waits for the token: it says
    // where the stream stands when it leaves, not a history.
    for (auto& queued : _operations.getPendingResponses()) {
        if (queued.pduType != BEEBUS_PDU_TOPIC || queued.destNodeID != nodeID) continue;
        if (queued.response.function != BEEBUS_TOP_STREAM_ACK) continue;
        queued.response.data.assign(pdu.data(), pdu.size());
        from->flags = (uint8_t)(from->flags & ~BEEBUS_PEER_ACK_DUE);
        return;
    }
    if (queueTopicReply(nodeID, BEEBUS_TOP_STREAM_ACK, 0, pdu)) {
        from = peer(nodeID, false);
        if (from) from->flags = (uint8_t)(from->flags & ~BEEBUS_PEER_ACK_DUE);
    }
}

void BeeBusProtocol::handleStreamAck(uint8_t srcNodeID, const BeeBusTopicPdu& pdu) {
    BeeBusPeer* to = peer(srcNodeID, false);
    if (!to) return;

    // Only what was sent can be acknowledged; anything else belongs to a
    // stream that is not this one.
    const uint8_t sentLast = (uint8_t)(to->txNext - 1);
    if ((uint8_t)(sentLast - pdu.seq) >= 128) return;

    if ((uint8_t)(pdu.seq - to->txAcked) < 128 && pdu.seq != to->txAcked) {
        to->txAcked = pdu.seq;
        to->txQuiet = 0;
    }

    const uint16_t due = _topicMap.streamAcked(srcNodeID, pdu.seq, pdu.missing);
    if (due != 0) {
        BEEBUS_DEBUG_IO("STREAM: Node:%d lost %u value(s), sending them again",
            srcNodeID, (unsigned)due);
    }
}

void BeeBusProtocol::tickStreams() {
    const uint8_t resendAfter =
        bb_setting<uint8_t>(_config.rotationsStreamResend, BEEBUS_ROTATIONS_STREAM_RESEND);

    for (size_t i = 0; i < _peers.size(); i++) {
        BeeBusPeer& peerState = _peers[i];

        // An acknowledgement that could not be queued goes when there is room.
        if (peerState.flags & BEEBUS_PEER_ACK_DUE) refreshStreamAck(peerState.node);

        if (bb_peerInFlight(peerState) == 0) {
            peerState.txQuiet = 0;
            continue;
        }
        if (++peerState.txQuiet < resendAfter) continue;

        // Its acknowledgements stopped: everything it has not confirmed goes
        // again, once, as its current value, and the window opens.
        const uint16_t due = _topicMap.streamResend(peerState.node);
        peerState.txAcked = (uint8_t)(peerState.txNext - 1);
        peerState.txQuiet = 0;
        BEEBUS_DEBUG_IO("STREAM: No acknowledgement from Node:%d, %u value(s) again",
            peerState.node, (unsigned)due);
        (void)due;
    }

    // A bind whose owner has its change but has not decided within the limit,
    // and a write of ours polled for its result the same. The limit covers the
    // owner's whole retry schedule for its RESULT, one attempt per operation
    // timeout, and one timeout more, so a RESULT lost twice still arrives
    // inside it. In milliseconds, from the same estimate the owner times its
    // retries by: counted in rotations it ran out early on a lossy link, where
    // the estimate, and so the owner's timeout, grows.
    const bb_time_t operation = ringTimeout(
        bb_setting<uint8_t>(_config.rotationsOperation, BEEBUS_ROTATIONS_OPERATION),
        _config.operationTimeout, BEEBUS_REFERENCE_OPERATION);
    const bb_time_t resultWait =
        _config.rotationsResult != BEEBUS_AUTO
            ? ringTimeout(_config.rotationsResult, BEEBUS_AUTO, BEEBUS_REFERENCE_OPERATION)
            : operation * ((bb_time_t)bb_retryLimit(_config.maxRetries) + 2u);
    const bb_time_t now = bb_millis();
    for (size_t i = 0; i < _resultRecords.size(); i++) {
        BeeBusResultRecord& record = _resultRecords[i];
        if ((record.flags & BEEBUS_RESULT_F_WAITING) == 0) continue;
        if ((record.flags & BEEBUS_RESULT_F_ACKED) == 0) continue;
        if (bb_elapsed(record.ackedAt, now) <= resultWait) continue;
        // An owner out of the ring is waited for, within an overall limit,
        // rather than counted against the asking.
        const bb_time_t overall = resultWait * (BEEBUS_RESULT_REASKS + 3u);
        if (!isMember(record.node) && bb_elapsed(record.since, now) <= overall) continue;
        // Asked again under the same request ID, a few times, before giving
        // up: the decision may have been lost, not withheld.
        if (record.reasks < BEEBUS_RESULT_REASKS && record.requestID != 0 &&
            !bb_topicTypeIsString(record.asked.type) &&
            reask(record.node, record.topicID, record.requestID, record.asked, nullptr,
                  bb_completion(&BeeBusProtocol::reaskCompleted, this))) {
            BeeBusResultRecord& again = _resultRecords[i];
            again.reasks++;
            again.flags = (uint8_t)(again.flags & ~BEEBUS_RESULT_F_ACKED);
            continue;
        }
        record.outcome = BEEBUS_OUTCOME_NONE;
        record.count = (uint16_t)(record.count + 1);
        record.flags = 0;
    }
    for (size_t i = 0; i < _bindings.size(); i++) {
        BeeBusBinding& bound = _bindings[i];
        if (bound.state != BeeBusBindState::Pending) continue;
        if ((bound.flags & BEEBUS_BIND_F_ACKED) == 0) continue;
        if (bb_elapsed(bound.ackedAt, now) <= resultWait) continue;
        const bool isText = bb_topicTypeIsString(bound.type);
        if (bound.reasks < BEEBUS_RESULT_REASKS && bound.requestID != 0 &&
            reask((uint8_t)(bound.key >> 16), (uint16_t)(bound.key & 0xFFFF), bound.requestID,
                  bound.requested, isText ? (const char*)bound.var : nullptr,
                  bb_completion(&BeeBusProtocol::bindWriteCompleted, this))) {
            BeeBusBinding& again = _bindings[i];
            again.reasks++;
            again.flags = (uint8_t)(again.flags & ~BEEBUS_BIND_F_ACKED);
            continue;
        }
        bound.outcome = BEEBUS_OUTCOME_BUSY;
        copyShadow(bound);
        setBindState(bound, BeeBusBindState::Failed);
    }
}

bool BeeBusProtocol::reask(uint8_t nodeID, uint16_t topicID, uint16_t requestID,
                           const BeeBusScalar& value, const char* text,
                           BeeBusCompletion onComplete) {
    if (!isMember(nodeID)) return false;
    const uint8_t flags = BEEBUS_WRITE_F_REQUEST | BEEBUS_WRITE_F_RESULT;
    BeeBusBytes pdu;
    bool built;
    if (text) {
        size_t length = strlen(text);
        if (length > BEEBUS_TOPIC_STRING_MAX) length = BEEBUS_TOPIC_STRING_MAX;
        built = BeeBusTopicFrame::buildWriteString(pdu, topicID, text, (uint8_t)length, flags,
                                                   requestID, _session);
    } else {
        built = BeeBusTopicFrame::buildWrite(pdu, topicID, value, flags, requestID, _session);
    }
    if (!built) return false;
    return bb_accepted(queueTopicOp(nodeID, BEEBUS_TOP_WRITE, topicID, pdu,
                                    BeeBusDelivery::Confirmed, false, onComplete));
}

void BeeBusProtocol::reaskCompleted(void* ctx, uint8_t nodeID, uint16_t topicID, uint8_t,
                                    BeeBusResult result) {
    static_cast<BeeBusProtocol*>(ctx)->noteResultWriteDone(nodeID, topicID, result);
}

// --- Request windows ---
namespace {

// Whether `id` is new to a window of the last 32 IDs below `high`, recording
// it if so. Behind the highest is another topic's request overtaken by a
// retry: new unless the window has it; past the window, too old to be anything
// but a copy.
bool takeIntoWindow(uint16_t id, uint16_t& high, uint32_t& seen, bool& started) {
    if (!started) {
        high = id;
        seen = 0;
        started = true;
        return true;
    }
    const uint16_t ahead = (uint16_t)(id - high);
    if (ahead == 0) return false;
    if (ahead < 0x8000u) {
        uint32_t moved = ahead >= 32 ? 0 : (seen << ahead);
        if (ahead - 1 < 32) moved |= (uint32_t)1 << (ahead - 1);
        seen = moved;
        high = id;
        return true;
    }
    const uint16_t behind = (uint16_t)(high - id);
    if (behind > 32) return false;
    const uint32_t bit = (uint32_t)1 << (behind - 1);
    if (seen & bit) return false;
    seen |= bit;
    return true;
}

}  // namespace

// --- Write requests, on the owner ---
bool BeeBusProtocol::acceptRequest(uint8_t srcNodeID, uint16_t requestID) {
    BeeBusPeer* from = peer(srcNodeID, true);
    if (!from) return true;
    bool started = (from->flags & BEEBUS_PEER_REQ_STARTED) != 0;
    const bool fresh = takeIntoWindow(requestID, from->reqHigh, from->reqSeen, started);
    if (started) from->flags = (uint8_t)(from->flags | BEEBUS_PEER_REQ_STARTED);
    return fresh;
}

// --- Results, on the owner ---
const BeeBusServed* BeeBusProtocol::served(uint16_t topicID) const {
    size_t lo = 0;
    size_t hi = _served.size();
    while (lo < hi) {
        const size_t mid = (lo + hi) / 2;
        if (_served[mid].topicID < topicID) lo = mid + 1;
        else hi = mid;
    }
    return lo < _served.size() && _served[lo].topicID == topicID ? &_served[lo] : nullptr;
}

bool BeeBusProtocol::holdResult(uint8_t writer, uint16_t topicID, uint16_t requestID,
                                const BeeBusScalar& written, bool applied) {
    if (_results.full()) return false;
    BeeBusPendingResult held;
    held.written = written;
    held.topicID = topicID;
    held.requestID = requestID;
    held.writer = writer;
    held.applied = applied;
    held.appliedAt = bb_millis();
    held.appliedLoop = _loopCount;
    return _results.push_back(held);
}

void BeeBusProtocol::noteTopicApplied(uint16_t topicID, uint16_t settleMs, bool command) {
    for (size_t i = 0; i < _results.size(); i++) {
        BeeBusPendingResult& held = _results[i];
        if (held.topicID != topicID || held.applied) continue;
        held.applied = true;
        held.appliedAt = bb_millis();
        held.appliedLoop = _loopCount;
        held.settleMs = settleMs;
        held.command = command;
        return;
    }
}

void BeeBusProtocol::answerRepeat(uint8_t writer, uint16_t topicID, uint16_t requestID) {
    for (size_t i = 0; i < _sentResults.size(); i++) {
        const BeeBusSentResult& sent = _sentResults[i];
        if (sent.writer == writer && sent.topicID == topicID && sent.requestID == requestID) {
            sendResult(writer, topicID, requestID, sent.outcome, true);
            return;
        }
    }
}

void BeeBusProtocol::sendResult(uint8_t writer, uint16_t topicID, uint16_t requestID,
                                uint8_t outcome, bool repeat) {
    // Remembered whether or not it can go now: a writer this node has
    // dropped for a moment asks again once it is back.
    if (!repeat) {
        if (_sentResults.full()) _sentResults.eraseAt(0);
        BeeBusSentResult sent;
        sent.topicID = topicID;
        sent.requestID = requestID;
        sent.writer = writer;
        sent.outcome = outcome;
        _sentResults.push_back(sent);
    }
    if (!isMember(writer)) return;

    // The value the owner holds now goes with the decision, whatever it is.
    BeeBusScalar value;
    const char* text = "";
    size_t length = 0;
    const uint16_t index = _topicMap.localIndex(topicID);
    if (index != BEEBUS_TOPIC_NONE) {
        const BeeBusLocalTopic& topic = _topicMap.localAt(index);
        if (bb_topicTypeIsString(topic.last.type)) {
            value.type = BEEBUS_T_STRING;
            const char* held = _topicMap.localString(index);
            if (held) {
                text = held;
                const uint16_t cap = topic.textCap;
                length = strnlen(held, cap < BEEBUS_TOPIC_STRING_MAX ? (size_t)cap
                                                                      : (size_t)BEEBUS_TOPIC_STRING_MAX);
            }
        } else {
            value = bb_scalarLoad(topic.last.type, topic.var);
        }
    } else {
        value.type = BEEBUS_T_U8;   // Nothing held: a placeholder the codec can carry
    }

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildResult(pdu, topicID, requestID, outcome, value, text,
                                       (uint8_t)length)) {
        return;
    }
    // Confirmed, and never merged with another: each request has its answer.
    queueTopicOp(writer, BEEBUS_TOP_RESULT, topicID, pdu, BeeBusDelivery::Confirmed, true,
                 bb_completion(&BeeBusProtocol::silentCompletion, this));

    // The record is finished with; the application may now be told, once.
    if (!repeat && _topicDecidedHandler) _topicDecidedHandler(writer, topicID, outcome);
}

void BeeBusProtocol::serviceResults() {
    if (_results.empty()) return;
    const bb_time_t now = bb_millis();

    for (size_t i = 0; i < _results.size();) {
        const BeeBusPendingResult held = _results[i];
        const BeeBusServed* terms = served(held.topicID);
        const bb_time_t settle = held.settleMs ? held.settleMs : (terms ? terms->terms.settleMs : 0);
        const bool command = held.command ||
                             (terms && terms->terms.kind == BeeBusServeKind::Command);

        // Taken, a loop() has run since (so has the program, which calls it),
        // and the settle time is up.
        if (!held.applied || _loopCount == held.appliedLoop ||
            bb_elapsed(held.appliedAt, now) < settle) {
            i++;
            continue;
        }
        _results.eraseAt(i);

        uint8_t outcome = BEEBUS_OUTCOME_ACCEPTED;
        const uint16_t index = _topicMap.localIndex(held.topicID);
        if (index == BEEBUS_TOPIC_NONE) {
            outcome = BEEBUS_OUTCOME_NO_TOPIC;
        } else if (!command) {
            // A command is an event, gone once taken; a value is read back.
            const BeeBusLocalTopic& topic = _topicMap.localAt(index);
            bool same;
            if (bb_topicTypeIsString(topic.last.type)) {
                const char* text = _topicMap.localString(index);
                const size_t length = text ? strnlen(text, topic.textCap) : 0;
                same = textDigest(text ? text : "", length) == held.written.bits;
            } else {
                same = bb_scalarEqual(bb_scalarLoad(topic.last.type, topic.var), held.written);
            }
            outcome = same ? BEEBUS_OUTCOME_ACCEPTED : BEEBUS_OUTCOME_CHANGED;
        }
        sendResult(held.writer, held.topicID, held.requestID, outcome);
    }
}

// --- Results, on the writer ---
BeeBusResultRecord* BeeBusProtocol::resultRecord(uint8_t nodeID, uint16_t topicID, bool create) {
    for (size_t i = 0; i < _resultRecords.size(); i++) {
        if (_resultRecords[i].node == nodeID && _resultRecords[i].topicID == topicID) {
            return &_resultRecords[i];
        }
    }
    if (!create) return nullptr;
    if (_resultRecords.full()) {
        // The oldest finished one goes; one waiting is kept.
        size_t drop = _resultRecords.size();
        for (size_t i = 0; i < _resultRecords.size() && drop == _resultRecords.size(); i++) {
            if ((_resultRecords[i].flags & BEEBUS_RESULT_F_WAITING) == 0) drop = i;
        }
        if (drop == _resultRecords.size()) return nullptr;
        _resultRecords.eraseAt(drop);
    }
    BeeBusResultRecord fresh;
    fresh.node = nodeID;
    fresh.topicID = topicID;
    if (!_resultRecords.push_back(fresh)) return nullptr;
    return &_resultRecords.back();
}

void BeeBusProtocol::noteResultWanted(uint8_t nodeID, uint16_t topicID,
                                      const BeeBusScalar& asked) {
    BeeBusResultRecord* record = resultRecord(nodeID, topicID, true);
    if (!record) return;
    record->flags = BEEBUS_RESULT_F_WAITING;
    record->requestID = 0;
    record->asked = asked;
    record->reasks = 0;
    record->since = 0;
}

void BeeBusProtocol::noteResultWriteDone(uint8_t nodeID, uint16_t topicID, BeeBusResult result) {
    BeeBusResultRecord* record = resultRecord(nodeID, topicID, false);
    if (!record || (record->flags & BEEBUS_RESULT_F_WAITING) == 0) return;
    const bb_time_t now = bb_millis();
    if (result == BeeBusResult::Superseded) return;
    if (result == BeeBusResult::Immediate || (record->requestID != 0 &&
                                              result != BeeBusResult::Refused &&
                                              result != BeeBusResult::NotMapped &&
                                              result != BeeBusResult::TypeMismatch &&
                                              result != BeeBusResult::Cancelled)) {
        // The owner has it - or may have: a request that went out and was
        // then cut off (the owner dropped from the ring for a moment, its
        // acknowledgements lost) is asked about again once it can be, under
        // the same ID, and the owner answers from what it decided.
        if (record->since == 0) record->since = now;
        record->flags = (uint8_t)(record->flags | BEEBUS_RESULT_F_ACKED);
        record->ackedAt = now;
        return;
    }
    // Refused outright, never sent, or made to an owner that has restarted
    // since: no decision will come.
    record->outcome = BEEBUS_OUTCOME_NONE;
    record->count = (uint16_t)(record->count + 1);
    record->flags = 0;
}

bool BeeBusProtocol::topicResult(uint8_t nodeID, uint16_t topicID, uint8_t& outcome,
                                 BeeBusScalar& kept, uint16_t& count) const {
    for (size_t i = 0; i < _resultRecords.size(); i++) {
        const BeeBusResultRecord& record = _resultRecords[i];
        if (record.node != nodeID || record.topicID != topicID) continue;
        outcome = record.outcome;
        kept = record.kept;
        count = record.count;
        return true;
    }
    return false;
}

void BeeBusProtocol::handleResult(uint8_t srcNodeID, const BeeBusTopicPdu& pdu) {
    // A RESULT is a confirmed operation of the owner's: one whose
    // acknowledgement was lost arrives again, and is acknowledged, not told.
    if (BeeBusPeer* from = peer(srcNodeID, true)) {
        bool started = (from->flags & BEEBUS_PEER_RESULT_STARTED) != 0;
        const bool fresh = takeIntoWindow(pdu.requestID, from->resultHigh, from->resultSeen,
                                          started);
        if (started) from->flags = (uint8_t)(from->flags | BEEBUS_PEER_RESULT_STARTED);
        if (!fresh) return;
    }

    // For a caller polling topicResult: the newest request's decision.
    // Only the request it answers: one not yet numbered has not gone, so a
    // RESULT arriving meanwhile is an earlier request's.
    if (BeeBusResultRecord* record = resultRecord(srcNodeID, pdu.topicID, false)) {
        if (record->requestID != 0 && record->requestID == pdu.requestID) {
            record->outcome = pdu.status;
            record->kept = pdu.value;
            record->count = (uint16_t)(record->count + 1);
            record->flags = 0;
        }
    }

    if (BeeBusBinding* bound = binding(srcNodeID, pdu.topicID)) {
        if (bound->state == BeeBusBindState::Pending && bound->requestID != 0 &&
            bound->requestID == pdu.requestID) {
            if (pdu.status == BEEBUS_OUTCOME_BUSY) {
                // No room at the owner for a decision: the change goes again.
                bound->flags = (uint8_t)(bound->flags | BEEBUS_BIND_F_RESEND);
                bound->flags = (uint8_t)(bound->flags & ~BEEBUS_BIND_F_ACKED);
                bound->requestID = 0;
            } else {
                bound->outcome = pdu.status;
                bound->kept = pdu.value;

                // The variable takes the owner's value as the decision left:
                // a RESULT carries the value as it was sent, so it is the
                // newest word from the owner on this topic when it arrives.
                if (bb_topicTypeIsString(bound->type)) {
                    const size_t n = pdu.text.size() < (size_t)bound->textCap - 1
                                         ? pdu.text.size() : (size_t)bound->textCap - 1;
                    memcpy(bound->shadow, pdu.text.data(), n);
                    bound->shadow[n] = '\0';
                } else if (pdu.value.type == bound->type) {
                    bb_scalarStore(pdu.value, bound->shadow);
                }
                bound->flags = (uint8_t)(bound->flags & ~BEEBUS_BIND_F_ACKED);
                copyShadow(*bound);
                setBindState(*bound, pdu.status == BEEBUS_OUTCOME_ACCEPTED
                                         ? BeeBusBindState::InSync
                                         : BeeBusBindState::Refused);
            }
        }
    }

    // The record is finished with; the application may now be told.
    if (_topicResultHandler) {
        _topicResultHandler(srcNodeID, pdu.topicID, pdu.status, pdu.value,
                            bb_topicTypeIsString(pdu.value.type) ? pdu.text.c_str() : "");
    }
}

size_t BeeBusProtocol::streamHeapBytes() const {
    size_t bytes = _peers.capacity() * sizeof(BeeBusPeer) +
                   _results.capacity() * sizeof(BeeBusPendingResult) +
                   _resultRecords.capacity() * sizeof(BeeBusResultRecord) +
                   _served.capacity() * sizeof(BeeBusServed) +
                   _bindings.capacity() * sizeof(BeeBusBinding);
    for (size_t i = 0; i < _bindings.size(); i++) {
        const bool text = bb_topicTypeIsString(_bindings[i].type);
        bytes += text ? (size_t)_bindings[i].textCap * 2 : 8;
    }
    return bytes;
}

uint32_t bb_resultTextDigest(const char* text, size_t length) {
    return textDigest(text, length);
}
