#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// The topic layer's outbound half: queuing PDUs, publishing what subscribers
// are owed, and draining rings. Inbound is BeeBusTopicHandler, storage is
// BeeBusTopicMap.

// --- Queuing ---
BeeBusResult BeeBusProtocol::queueTopicOp(uint8_t destNodeID, uint8_t opcode,
                                          uint16_t topicID,
                                          const BeeBusBytes& pdu,
                                          BeeBusDelivery delivery, bool noSupersede,
                                          BeeBusCompletion onComplete) {
    if (destNodeID == 0 || destNodeID == _nodeID || pdu.empty()) {
        return BeeBusResult::InvalidArgument;
    }
    // A broadcast is addressed to the ring rather than to a member of it, so
    // there is no membership to check; what it needs is a ring to speak on. A
    // node not in one has no destination at all, which is the same answer.
    if (destNodeID == BEEBUS_NODE_BROADCAST) {
        if (!isRingParticipant()) {
            return _duplicateNodeID ? BeeBusResult::DuplicateNodeID
                                    : BeeBusResult::UnknownNode;
        }
    } else if (!isMember(destNodeID)) {
        return _duplicateNodeID ? BeeBusResult::DuplicateNodeID
                                : BeeBusResult::UnknownNode;
    }

    BeeBusPendingModbusOp op;
    op.pduType = BEEBUS_PDU_TOPIC;
    op.noSupersede = noSupersede;
    op.destNodeID = destNodeID;
    op.sourceNodeID = _nodeID;
    op.req.function = opcode;
    op.req.startAddr = topicID;
    op.req.quantity = 0;
    op.req.data.assign(pdu.data(), pdu.size());
    op.req.isResponse = false;
    op.timestamp = bb_millis();
    op.retryCount = 0;
    op.resultPtr = nullptr;
    op.isArray = false;
    op.arraySize = 0;
    op.wantsAck = (delivery == BeeBusDelivery::Confirmed);

    // Every topic operation reports its outcome, and it is arranged here rather
    // than at each of the dozen call sites: one that forgot would leave a
    // caller waiting on a completion that never came, and nothing would say so.
    // A caller with its own handler keeps it — the two are not both wanted.
    op.onComplete = onComplete ? onComplete : bb_completion(&BeeBusProtocol::topicCompleted, this);

    return _operations.addPendingOperation(op, *this);
}

void BeeBusProtocol::topicCompleted(void* ctx, uint8_t nodeID, uint16_t topicID,
                                    uint8_t opcode, BeeBusResult result) {
    static_cast<BeeBusProtocol*>(ctx)->raiseTopicComplete(nodeID, topicID, opcode, result);
}

bool BeeBusProtocol::queueTopicReply(uint8_t destNodeID, uint8_t opcode, uint16_t topicID,
                                     const BeeBusBytes& pdu) {
    if (destNodeID == 0 || pdu.empty()) return false;

    BeeBusPendingResponse reply;
    reply.pduType = BEEBUS_PDU_TOPIC;
    reply.response.function = opcode;
    reply.response.startAddr = topicID;
    reply.response.quantity = 0;
    reply.response.data.assign(pdu.data(), pdu.size());
    reply.response.isResponse = true;
    reply.destNodeID = destNodeID;
    reply.sourceNodeID = _nodeID;
    reply.timestamp = bb_millis();

    return _operations.addPendingResponse(reply);
}

// --- Publishing ---
BeeBusResult BeeBusProtocol::sendTopicValue(uint8_t destNodeID, uint16_t topicIndex,
                                            uint8_t opcode, const BeeBusScalar& value,
                                            bb_time_t sampleAt, BeeBusDelivery delivery,
                                            uint8_t skipped, const char* text) {
    if (topicIndex >= _topicMap.localCount()) {
        return BeeBusResult::NotMapped;
    }

    const uint16_t topicID = _topicMap.localAt(topicIndex).id;

    // Every destination has a stream of its own, so a subscriber whose deadband
    // skipped a change does not read it as a loss. A reader that never
    // subscribed gets a slot too: the sequence belongs to the stream.
    const uint16_t slotIndex = _topicMap.slotFor(topicIndex, destNodeID, true);
    if (slotIndex == BEEBUS_TOPIC_NONE) {
        return BeeBusResult::QueueFull;
    }

    const bb_time_t now = bb_millis();
    bb_time_t heldFor = bb_elapsed(sampleAt, now);

    // Under a second is not worth three bytes of age, and a wrapped clock must
    // not report an old sample as fresh.
    if (heldFor > 0x7FFFFFFFu) heldFor = 0;
    const bool aged = heldFor >= 1000;

    // A live publish to a RELIABLE subscriber goes on this node's stream to
    // it, numbered as it leaves. What is not live (a buffered sample, already
    // confirmed one by one, or a read reply) keeps the topic's own count.
    const bool streamed = opcode == BEEBUS_TOP_PUBLISH && destNodeID != BEEBUS_NODE_BROADCAST &&
                          delivery == BeeBusDelivery::Unconfirmed && skipped == 0 && !text &&
                          (_topicMap.slotAt(slotIndex).flags & BEEBUS_SUB_F_RELIABLE) != 0;
    const uint8_t typeFlags = streamed ? BEEBUS_T_RELIABLE : 0;

    const uint8_t seq = streamed ? 0 : (uint8_t)(_topicMap.slotAt(slotIndex).seq + 1 + skipped);

    BeeBusBytes pdu;
    const char* source = nullptr;

    // A subscribe answer still queued for this destination carries the topic's
    // value, and leaves after any publish (answers follow operations in a
    // frame): it is given this value instead, so the answer is never the older
    // of the two to arrive.
    if (opcode == BEEBUS_TOP_PUBLISH && destNodeID != BEEBUS_NODE_BROADCAST && skipped == 0 &&
        !text && refreshSubackValue(destNodeID, topicIndex, value)) {
        BeeBusSubSlot& held = _topicMap.slotAt(slotIndex);
        held.lastSent = value;
        held.lastSentAt = now;
        held.flags = (uint8_t)(held.flags | BEEBUS_SUB_F_SENT);
        if (bb_topicTypeIsString(value.type)) {
            _topicMap.noteStringSent(topicID, slotIndex, _topicMap.localString(topicIndex));
        }
        _topicMap.localAt(topicIndex).lastPubAt = now;
        return BeeBusResult::Queued;
    }

    if (bb_topicTypeIsString(value.type)) {
        // A buffered sample brings its own text; a live publish reads the bound
        // variable.
        source = text ? text : _topicMap.localString(topicIndex);
        if (!source) {
            return BeeBusResult::NotMapped;
        }

        // The length byte is what bounds a value. A longer one is carried as
        // far as it goes rather than never arriving at all. Bounded by the
        // topic's buffer too, which need not be terminated when full.
        const uint16_t cap = _topicMap.localAt(topicIndex).textCap;
        const size_t length = strnlen(source, cap < BEEBUS_TOPIC_STRING_MAX
                                                  ? (size_t)cap
                                                  : (size_t)BEEBUS_TOPIC_STRING_MAX);

        if (!BeeBusTopicFrame::buildPublishString(pdu, opcode, topicID, seq,
                                                  source, (uint8_t)length,
                                                  aged, heldFor / 1000, typeFlags)) {
            return BeeBusResult::InvalidArgument;
        }
    } else if (!BeeBusTopicFrame::buildPublish(pdu, opcode, topicID, seq, value,
                                               aged, heldFor / 1000, typeFlags)) {
        return BeeBusResult::InvalidArgument;
    }

    if (opcode == BEEBUS_TOP_READ_REPLY) {
        queueTopicReply(destNodeID, opcode, topicID, pdu);
    } else {
        // A publish never supersedes an earlier one: each carries its own
        // sequence, and replacing one would leave a number nothing was sent
        // under.
        const BeeBusResult result = queueTopicOp(destNodeID, opcode, topicID, pdu,
                                                 delivery, true, BeeBusCompletion());
        if (!bb_accepted(result)) {
            return result;
        }
    }

    // Committed only once queued, so a refused publish does not consume a
    // sequence number and open a gap that never happened.
    BeeBusSubSlot& stream = _topicMap.slotAt(slotIndex);
    if (!streamed) stream.seq = seq;
    stream.lastSent = value;
    stream.lastSentAt = now;
    stream.flags = (uint8_t)(stream.flags | BEEBUS_SUB_F_SENT);

    // A string destination keeps which version of the text it has.
    if (source) {
        _topicMap.noteStringSent(topicID, slotIndex, source);
    }

    // A broadcast reached every node holding the topic, subscribers included, so
    // each of their streams has had this value. Said here rather than left to
    // the scan to work out: otherwise every subscriber still reads as due and
    // the same value goes out again, once each, behind the broadcast that
    // already delivered it.
    if (destNodeID == BEEBUS_NODE_BROADCAST && opcode == BEEBUS_TOP_PUBLISH) {
        _topicMap.noteAllSlotsSent(topicIndex, value, now);
    }

    _topicMap.localAt(topicIndex).lastPubAt = now;
    return BeeBusResult::Queued;
}

bool BeeBusProtocol::refreshSubackValue(uint8_t destNodeID, uint16_t topicIndex,
                                        const BeeBusScalar& value) {
    const BeeBusLocalTopic& topic = _topicMap.localAt(topicIndex);
    for (auto& queued : _operations.getPendingResponses()) {
        if (queued.pduType != BEEBUS_PDU_TOPIC || queued.destNodeID != destNodeID) continue;
        if (queued.response.function != BEEBUS_TOP_SUBACK) continue;
        if (queued.response.startAddr != topic.id) continue;

        BeeBusTopicPdu& parsed = _refreshPdu;
        if (!BeeBusTopicFrame::parse(queued.response.data.data(),
                                     (uint16_t)queued.response.data.size(), parsed) ||
            parsed.status != BEEBUS_TOP_OK || !parsed.hasValue) {
            return false;
        }
        const char* text = nullptr;
        size_t length = 0;
        if (bb_topicTypeIsString(topic.last.type)) {
            text = _topicMap.localString(topicIndex);
            if (!text) return false;
            const uint16_t cap = topic.textCap;
            length = strnlen(text, cap < BEEBUS_TOPIC_STRING_MAX ? (size_t)cap
                                                                  : (size_t)BEEBUS_TOPIC_STRING_MAX);
        }
        BeeBusBytes rebuilt;
        if (!BeeBusTopicFrame::buildSubackValue(rebuilt, topic.id, parsed.session, parsed.seq,
                                                value, text, (uint8_t)length,
                                                parsed.reliable ? BEEBUS_T_RELIABLE : 0)) {
            return false;
        }
        queued.response.data.assign(rebuilt.data(), rebuilt.size());
        return true;
    }
    return false;
}

// --- Receiving ---
uint8_t BeeBusProtocol::applyTopicValue(uint8_t srcNodeID, uint16_t index,
                                        const BeeBusTopicPdu& pdu, bool broadcast) {
    BeeBusRemoteTopic& topic = _topicMap.remoteAt(index);
    const uint16_t topicID = (uint16_t)(topic.key & 0xFFFF);

    if (pdu.value.type != topic.type) {
        BEEBUS_DEBUG_IO("TOPIC: Node:%d topic %u is %s, bound here as %s",
            srcNodeID, (unsigned)topicID, bb_topicTypeName(pdu.value.type),
            bb_topicTypeName(topic.type));
        return BEEBUS_TOP_ERR_TYPE;
    }

    // A broadcast is outside the counting, and takes no part in it.
    //
    // A sequence belongs to a stream, and a stream is one publisher counting to
    // one destination: that is what lets a subscriber on a deadband be sent one
    // sample in six without reading five as losses. A frame addressed to the
    // ring has no destination of its own, so its number means nothing here —
    // and the number it does carry belongs to a different counter from the one
    // this record is following. Measured against that counter it would read as
    // a copy the medium held on to, and a published value that arrived intact
    // would be thrown away in silence.
    //
    // So it is neither checked nor recorded. Applied as it stands, and the
    // subscription's own stream carries on undisturbed beside it — which is
    // what keeps the two ways of publishing a topic from interfering. Nothing
    // is given up: a broadcast is unacknowledged by definition, so there is no
    // loss to detect in it.
    const bool haveStream = (topic.flags & BEEBUS_REMOTE_F_SEQ) != 0;
    const uint8_t delta =
        broadcast ? 1 : (uint8_t)(pdu.seq - (haveStream ? topic.lastSeq : 0));

    bool duplicate = false;

    // Half the range behind what was applied is a copy the medium held on to.
    // Only meaningful once a stream is established.
    if (!broadcast && haveStream && delta >= 128) {
        BEEBUS_DEBUG_IO("TOPIC: Stale publish of %u from Node:%d, discarded",
            (unsigned)topicID, srcNodeID);
        return BEEBUS_TOP_OK;
    }

    // Worked out here but raised at the end, with the other events. Every
    // handler below is application code and may bind or unbind a topic, which
    // reallocates the remote vector and invalidates both `topic` and `index`;
    // nothing may call out until this record is finished with.
    uint8_t missed = 0;
    if (delta == 0) {
        duplicate = true;
    } else if (delta >= 2 && pdu.opcode == BEEBUS_TOP_PUBLISH) {
        // A solicited reply is not part of the stream's cadence.
        missed = (uint8_t)(delta - 1);
    }

    // The broadcast leaves the stream where it found it, for the same reason it
    // was not measured against it: the number it carries is another counter's.
    if (!broadcast) {
        topic.lastSeq = pdu.seq;
        topic.flags = (uint8_t)(topic.flags | BEEBUS_REMOTE_F_SEQ);
    }
    topic.flags = (uint8_t)(topic.flags | BEEBUS_REMOTE_F_RECEIVED);

    // Counted here rather than left to the handlers, so a caller that polls can
    // see both without registering anything.
    if (!duplicate) topic.rxCount = (uint16_t)(topic.rxCount + 1);
    if (missed != 0) {
        const uint32_t total = (uint32_t)topic.missedCount + missed;
        topic.missedCount = total > 0xFFFFu ? 0xFFFFu : (uint16_t)total;
    }

    // The age was measured on the publisher's clock and is being subtracted
    // from ours, and the two have no relation: a node that rebooted a moment
    // ago is younger than the history a peer is flushing into it. Clamped at
    // this node's own uptime, because a stamp before that would wrap and read
    // as a time far in the future, at which point every sample in the flush
    // loses to the value already held and the bound variable never moves.
    //
    // Also clamped to half the clock's range, past which a back-dated stamp
    // would land in the future for the ordinary reason.
    const bb_time_t now = bb_millis();
    uint32_t seconds = pdu.aged ? pdu.ageSeconds : 0;
    if (seconds > 2000000u) seconds = 2000000u;

    bb_time_t backdate = (bb_time_t)(seconds * 1000u);
    if (backdate > now) backdate = now;   // No older than this node is
    const bb_time_t sampleAt = now - backdate;

    // Back-dated rather than stored beside the value, so quality, freshness and
    // newest-wins all work unchanged. A flush arrives oldest first, so without
    // this the variable would end up holding the oldest sample.
    const bool isString = bb_topicTypeIsString(topic.type);

    const bool newer = bb_elapsed(topic.rxAt, sampleAt) < 0x80000000u;
    if (newer) {
        topic.rxAt = sampleAt;
        if (isString) {
            _topicMap.setRemoteString(index, pdu.text.data(), (uint16_t)pdu.text.size());
        } else {
            bb_scalarStore(pdu.value, topic.var);
        }
    }

    // The record is finished with, so it is safe to call out. Nothing below may
    // touch `topic` or `index` again.
    if (missed != 0) {
        raiseTopicGap(srcNodeID, topicID, missed);
    }

    // A bound topic's shadow moved: the bind decides what its caller sees.
    if (newer && !_bindings.empty()) {
        bindValueArrived(srcNodeID, topicID);
    }

    // Every entry reaches the handler in arrival order, whether or not it won
    // the variable. A string goes to its own handler as well, carrying the text
    // this entry held rather than whatever the variable ended up with.
    if (!duplicate) {
        raiseTopicChanged(srcNodeID, topicID, pdu.value);
        if (isString) {
            raiseTopicStringChanged(srcNodeID, topicID, pdu.text.data(),
                                    (uint16_t)pdu.text.size());
        }
    }
    return BEEBUS_TOP_OK;
}

// --- Subscriptions ---
BeeBusResult BeeBusProtocol::issueSubscribe(uint8_t nodeID, uint16_t topicID) {
    const uint16_t index = _topicMap.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) {
        return BeeBusResult::NotMapped;
    }

    // A peer that is not in the ring is not a refusal. The terms are already
    // recorded, so the subscription is a standing intent the ring settles when
    // the peer arrives — which is the same thing resubscribeTo() does for a
    // peer that comes back, and there is no reason one that has never been
    // here should be treated differently. Held here rather than left to the
    // caller: a caller that has to watch for the peer itself is one that has
    // to know the protocol's membership rules, and only a caller that polls
    // every scan gets the behaviour at all.
    //
    // A full queue is the same kind of answer: held, and sent as room
    // frees, so an application can subscribe everything it binds at once.
    if (!isMember(nodeID)) {
        deferSubscribe(nodeID, topicID);
        return BeeBusResult::Queued;
    }
    const BeeBusResult result = sendSubscribe(nodeID, topicID);
    if (result == BeeBusResult::QueueFull) {
        deferSubscribe(nodeID, topicID);
        return BeeBusResult::Queued;
    }
    return result;
}

BeeBusResult BeeBusProtocol::sendSubscribe(uint8_t nodeID, uint16_t topicID) {
    const uint16_t index = _topicMap.remoteIndex(nodeID, topicID);
    if (index == BEEBUS_TOPIC_NONE) {
        return BeeBusResult::NotMapped;
    }

    const BeeBusRemoteTopic& topic = _topicMap.remoteAt(index);
    const BeeBusContract terms = _topicMap.contract(topic.contractId);
    const uint8_t qos = (topic.flags & BEEBUS_REMOTE_F_RELIABLE) ? BEEBUS_SUB_Q_RELIABLE : 0;

    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildSubscribe(pdu, topicID, terms.deadband,
                                          bb_encodeInterval(terms.maxInterval), qos, _session)) {
        return BeeBusResult::InvalidArgument;
    }
    return queueTopicOp(nodeID, BEEBUS_TOP_SUBSCRIBE, topicID, pdu,
                        BeeBusDelivery::Unconfirmed, false, BeeBusCompletion());
}

void BeeBusProtocol::dropDeferredSubscribe(uint8_t nodeID, uint16_t topicID) {
    const uint32_t key = ((uint32_t)nodeID << 16) | topicID;

    for (size_t i = 0; i < _deferredSubs.size(); i++) {
        if (_deferredSubs[i] == key) {
            _deferredSubs.eraseAt(i);
            return;
        }
    }
}

void BeeBusProtocol::deferSubscribe(uint8_t nodeID, uint16_t topicID) {
    const uint32_t key = ((uint32_t)nodeID << 16) | topicID;

    for (size_t i = 0; i < _deferredSubs.size(); i++) {
        if (_deferredSubs[i] == key) return;
    }
    // Room for every remote topic, taken as the map grows.
    if (_deferredSubs.full()) {
        const size_t room = _topicMap.remoteCount() > _deferredSubs.size()
                                ? _topicMap.remoteCount() : _deferredSubs.size() + 1;
        _deferredSubs.grow(room);
    }
    _deferredSubs.push_back(key);
}

void BeeBusProtocol::resubscribeTo(uint8_t nodeID) {
    for (size_t i = 0; i < _topicMap.remoteCount(); i++) {
        const BeeBusRemoteTopic& topic = _topicMap.remoteAt(i);
        if ((uint8_t)(topic.key >> 16) != nodeID) continue;
        if (topic.contractId == BEEBUS_SUB_NONE) continue;
        if (topic.flags & BEEBUS_REMOTE_F_SUBSCRIBED) continue;

        // Deferred rather than queued: a peer may carry more subscriptions
        // than the queue holds.
        deferSubscribe(nodeID, (uint16_t)(topic.key & 0xFFFF));
    }
}

// --- Commissioning ---
bool BeeBusProtocol::buildTopicListPage(uint16_t cursor, uint8_t maxEntries, bool withNames,
                                        BeeBusBytes& out, uint16_t& firstID) {
    if (maxEntries == 0) maxEntries = 1;
    if (maxEntries > BEEBUS_TOPIC_LIST_MAX_ENTRIES) maxEntries = BEEBUS_TOPIC_LIST_MAX_ENTRIES;

    BeeBusTopicEntryList entries;
    uint16_t bytes = 0;
    uint16_t nextCursor = 0;
    bool more = false;

    // The cursor is a topic ID rather than a position, so topics added or
    // removed between pages cannot make the walk skip or repeat one.
    for (size_t i = 0; i < _topicMap.localCount(); i++) {
        const BeeBusLocalTopic& topic = _topicMap.localAt(i);
        if (topic.id < cursor) continue;

        BeeBusTopicEntry entry;
        entry.topicID = topic.id;
        entry.type = topic.last.type;
        entry.flags = topic.flags;
#if BEEBUS_TOPIC_NAMES
        if (withNames && topic.name) entry.name = topic.name;
#endif

        // A page is kept to BEEBUS_TOPIC_LIST_PAGE_BYTES, except that its
        // first entry always goes: a long name can be bigger than a page on
        // its own, and refusing it would stall the walk on that topic.
        const uint16_t size = BeeBusTopicFrame::entrySize(entry, withNames);
        if (entries.size() >= maxEntries ||
            (!entries.empty() && (uint16_t)(bytes + size) > BEEBUS_TOPIC_LIST_PAGE_BYTES)) {
            more = true;
            nextCursor = topic.id;
            break;
        }

        bytes = (uint16_t)(bytes + size);
        entries.push_back(entry);
    }

    firstID = entries.empty() ? 0 : entries[0].topicID;
    return BeeBusTopicFrame::buildListReply(out, more, withNames, nextCursor, entries);
}

// --- Scheduling ---
void BeeBusProtocol::serviceTopics() {
    // Subscriptions held back for room. Ahead of the early return below, since
    // a node with no subscribers of its own may still owe these.
    size_t held = 0;
    while (isRingParticipant() && held < _deferredSubs.size() &&
           _operations.getPendingOpCount() < _operations.getPendingOps().capacity() / 2) {
        const uint32_t key = _deferredSubs[held];
        const uint8_t destNodeID = (uint8_t)(key >> 16);

        // A peer that is not in the ring keeps its place. Erasing it would
        // drop the subscription for the one reason it was held — and passed
        // over rather than waited on, so one absent peer does not hold up the
        // subscriptions behind it.
        if (!isMember(destNodeID)) {
            held++;
            continue;
        }

        if (sendSubscribe(destNodeID, (uint16_t)(key & 0xFFFF)) == BeeBusResult::QueueFull) {
            break;
        }
        _deferredSubs.eraseAt(held);
    }

    if (!_topicMap.hasSubscribers() && _topicMap.bufferedTopicCount() == 0) return;

    const bb_time_t now = bb_millis();

    // The scan runs whether or not this node is in a ring: a buffered topic has
    // to record while there is nowhere to send.
    _topicMap.scanDue(now);

    if (!isRingParticipant()) return;
    if (_topicMap.dueCount() == 0) return;

    // RELIABLE values queued to each peer and not yet numbered: with those in
    // flight, what the window holds back. Counted afresh each pass, so nothing
    // that leaves the queue another way can leave the count wrong.
    for (size_t i = 0; i < _peers.size(); i++) _peers[i].txQueued = 0;
    const BeeBusList<BeeBusPendingModbusOp>& pending = _operations.getPendingOps();
    for (size_t i = 0; i < pending.size(); i++) {
        const BeeBusPendingModbusOp& op = pending[i];
        if (op.pduType != BEEBUS_PDU_TOPIC || op.req.function != BEEBUS_TOP_PUBLISH) continue;
        uint16_t topicID = 0;
        const uint8_t used = BeeBusTopicFrame::unpackVarint(op.req.data.data() + 1,
                                                            (uint16_t)(op.req.data.size() - 1),
                                                            topicID);
        const size_t typeAt = (size_t)used + 2;
        if (used == 0 || typeAt >= op.req.data.size()) continue;
        if ((op.req.data[typeAt] & BEEBUS_T_RELIABLE) == 0) continue;
        if (BeeBusPeer* to = peer(op.destNodeID, false)) to->txQueued++;
    }
    // Sequence numbers are compared in half of their eight bits, so no more
    // than 127 may be in flight at once.
    const uint8_t configured = bb_setting<uint8_t>(_config.streamWindow, BEEBUS_STREAM_WINDOW);
    const uint8_t window = configured > 127 ? 127 : configured;

    for (size_t i = 0; i < _topicMap.dueCount(); i++) {
        const uint16_t topicIndex = _topicMap.dueAt(i).topicIndex;
        const BeeBusSubSlot& slot = _topicMap.slotAt(_topicMap.dueAt(i).slotIndex);
        const uint8_t destNodeID = slot.nodeID;
        const bool reliable = (slot.flags & BEEBUS_SUB_F_RELIABLE) != 0;

        if (!isMember(destNodeID)) continue;

        const uint16_t topicID = _topicMap.localAt(topicIndex).id;

        // One publish of a topic to a destination in flight at a time, or a
        // value changing every loop() would fill the queue between two tokens.
        if (_operations.findTopicOp(destNodeID, topicID, BEEBUS_TOP_PUBLISH)) continue;

        // A full window holds a RELIABLE change back, still due: it coalesces
        // with whatever follows and goes as the latest value.
        BeeBusPeer* to = nullptr;
        if (reliable) {
            to = peer(destNodeID, true);
            if (to && (uint16_t)bb_peerInFlight(*to) + to->txQueued >= window) continue;
        }

        const BeeBusScalar value = bb_scalarLoad(_topicMap.localAt(topicIndex).last.type,
                                                 _topicMap.localAt(topicIndex).var);
        if (bb_accepted(sendTopicValue(destNodeID, topicIndex, BEEBUS_TOP_PUBLISH, value, now)) &&
            reliable) {
            to = peer(destNodeID, false);
            if (to) to->txQueued++;
        }
    }
}

void BeeBusProtocol::drainTopicBuffers() {
    // Four gates: this node is taking part, the join churn is over, the queue
    // is under half full, and the topic has a live subscriber. The last is
    // checked per topic below.
    if (_topicMap.bufferedTopicCount() == 0 || !isRingParticipant()) return;
    if (_rotationsSinceJoin < bb_setting<uint8_t>(_config.rotationsFlushSettle, BEEBUS_ROTATIONS_FLUSH_SETTLE)) return;
    // Half the queue this node has, whatever size it was given.
    const size_t halfQueue = _operations.getPendingOps().capacity() / 2;
    if (_operations.getPendingOpCount() >= halfQueue) return;

    uint8_t sent = 0;
    uint8_t destinations[BEEBUS_NODE_ID_MAX];

    for (size_t i = 0; i < _topicMap.bufferedTopicCount() && sent < BEEBUS_TOPIC_FLUSH_PER_TURN; i++) {
        const uint16_t topicID = _topicMap.bufferedTopicAt(i);
        const uint16_t topicIndex = _topicMap.localIndex(topicID);
        if (topicIndex == BEEBUS_TOPIC_NONE) continue;

        const size_t subscribers = _topicMap.subscribersOf(topicIndex, destinations,
                                                           sizeof(destinations));

        // Only peers in the ring: publishing elsewhere consumes the sample and
        // delivers it nowhere.
        size_t live = 0;
        for (size_t d = 0; d < subscribers; d++) {
            if (isMember(destinations[d])) destinations[live++] = destinations[d];
        }
        if (live == 0) continue;

        const bool isString =
            bb_topicTypeIsString(_topicMap.localAt(topicIndex).last.type);

        while (sent < BEEBUS_TOPIC_FLUSH_PER_TURN) {
            BeeBusTopicSample sample;
            const char* text = nullptr;

            // A string sample carries its text out of the ring, so what goes on
            // the wire is what was recorded rather than what the variable holds
            // now.
            if (isString) {
                if (!_topicMap.peekStringSample(topicID, sample, text)) break;
            } else if (!_topicMap.peekSample(topicID, sample)) {
                break;
            }

            // Room for every destination before any is sent, so a sample is
            // never delivered to half the subscribers and repeated to the rest
            // under a different sequence number.
            if (_operations.getPendingOpCount() + live > halfQueue) {
                return;
            }

            // What the ring overwrote is spent as sequence numbers on the
            // first sample out, so the loss arrives marked. A step of 128 or
            // more reads as stale, so a larger loss reports as 126.
            uint16_t dropped = _topicMap.droppedCount(topicID);
            if (dropped > 126) dropped = 126;

            for (size_t d = 0; d < live; d++) {
                sendTopicValue(destinations[d], topicIndex, BEEBUS_TOP_PUBLISH,
                               sample.value, sample.at, BeeBusDelivery::Confirmed,
                               (uint8_t)dropped, isString ? text : nullptr);
            }
            _topicMap.clearDropped(topicID);

            _topicMap.dropSample(topicID);
            sent++;
        }
    }
}
