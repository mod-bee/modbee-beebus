#include "BeeBusGlobal.h"
#include "BeeBusTopicHandler.h"
#include "BeeBusNoExcept.h"

uint8_t BeeBusTopicHandler::process(const uint8_t* pdu, uint16_t length, uint8_t srcNodeID,
                                    bool broadcast, BeeBusTopicPdu& parsed) {
    if (!BeeBusTopicFrame::parse(pdu, length, parsed)) {
        BEEBUS_DEBUG_IO("TOPIC: Malformed PDU from Node:%d (%d bytes)", srcNodeID, length);
        _protocol.reportError(BeeBusError::InvalidRequest, "Malformed topic PDU");
        return BEEBUS_TOP_ERR_MALFORMED;
    }

    BEEBUS_DEBUG_IO("TOPIC: %s topic:%u from Node:%d",
        bb_topicOpcodeName(parsed.opcode), (unsigned)parsed.topicID, srcNodeID);

    switch (parsed.opcode) {
        case BEEBUS_TOP_PUBLISH:
        case BEEBUS_TOP_READ_REPLY:   return handleValue(parsed, srcNodeID, broadcast);
        case BEEBUS_TOP_WRITE:        return handleWrite(parsed, srcNodeID);
        case BEEBUS_TOP_READ:         return handleRead(parsed, srcNodeID);
        case BEEBUS_TOP_SUBSCRIBE:    return handleSubscribe(parsed, srcNodeID);
        case BEEBUS_TOP_SUBACK:       return handleSuback(parsed, srcNodeID);
        case BEEBUS_TOP_UNSUBSCRIBE:  return handleUnsubscribe(parsed, srcNodeID);
        case BEEBUS_TOP_LIST_REQUEST: return handleListRequest(parsed, srcNodeID);
        case BEEBUS_TOP_LIST_REPLY:   return handleListReply(parsed, srcNodeID);
        case BEEBUS_TOP_REGISTER:     return handleRegister(parsed, srcNodeID);
        case BEEBUS_TOP_ERROR:        return handleError(parsed, srcNodeID);
        case BEEBUS_TOP_STREAM_ACK:   return handleStreamAck(parsed, srcNodeID);
        case BEEBUS_TOP_RESULT:       return handleResult(parsed, srcNodeID);
        case BEEBUS_TOP_SESSION:      return handleSession(parsed, srcNodeID);
    }
    return BEEBUS_TOP_ERR_MALFORMED;
}

void BeeBusTopicHandler::refuse(uint8_t destNodeID, uint16_t topicID, uint8_t reason) {
    BeeBusBytes pdu;
    if (!BeeBusTopicFrame::buildError(pdu, topicID, reason)) return;
    _protocol.queueTopicReply(destNodeID, BEEBUS_TOP_ERROR, topicID, pdu);
}

// --- A value arriving from a peer ---
uint8_t BeeBusTopicHandler::handleValue(const BeeBusTopicPdu& pdu, uint8_t srcNodeID,
                                        bool broadcast) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    // A RELIABLE value is counted on the sender's stream to us before anything
    // else, bound here or not, so a number is never left looking lost. A copy
    // is acknowledged again and goes no further.
    const bool streamed = pdu.reliable && !broadcast && pdu.opcode == BEEBUS_TOP_PUBLISH;
    if (streamed && !_protocol.acceptStream(srcNodeID, pdu.seq)) {
        return BEEBUS_TOP_OK;
    }

    const uint16_t index = map.remoteIndex(srcNodeID, pdu.topicID);

    // A streamed value the receiver already holds is a repair that crossed
    // its original, or a re-send after a lost acknowledgement: nothing new,
    // so nothing is applied or raised a second time.
    if (streamed && index != BEEBUS_TOPIC_NONE &&
        (map.remoteAt(index).flags & BEEBUS_REMOTE_F_RECEIVED) != 0 &&
        pdu.value.type == map.remoteAt(index).type) {
        const BeeBusRemoteTopic& held = map.remoteAt(index);
        if (bb_topicTypeIsString(held.type)) {
            const char* text = map.remoteString(index);
            if (text && pdu.text == text) return BEEBUS_TOP_OK;
        } else if (bb_scalarEqual(bb_scalarLoad(held.type, held.var), pdu.value)) {
            return BEEBUS_TOP_OK;
        }
    }

    if (index == BEEBUS_TOPIC_NONE) {
        // Nothing bound here. Not answered: the publisher was right to send it.
        BEEBUS_DEBUG_IO("TOPIC: Value for unregistered topic %u from Node:%d",
            (unsigned)pdu.topicID, srcNodeID);
        return BEEBUS_TOP_OK;
    }

    // A streamed value carries the stream's number, not the topic's: applied
    // outside the topic's own count, as a broadcast is.
    const uint8_t reason = _protocol.applyTopicValue(srcNodeID, index, pdu, broadcast || streamed);
    if (reason != BEEBUS_TOP_OK) {
        refuse(srcNodeID, pdu.topicID, reason);

        // A read whose answer cannot be stored is finished.
        if (pdu.opcode == BEEBUS_TOP_READ_REPLY) {
            _protocol.getOperations().completeTopicOp(srcNodeID, pdu.topicID,
                                                      BEEBUS_TOP_READ,
                                                      bb_topicReasonToResult(reason));
        }
        return reason;
    }

    if (pdu.opcode == BEEBUS_TOP_READ_REPLY) {
        _protocol.getOperations().completeTopicOp(srcNodeID, pdu.topicID, BEEBUS_TOP_READ,
                                                  BeeBusResult::Immediate);
    }
    return BEEBUS_TOP_OK;
}

// --- A peer writing one of our topics ---
uint8_t BeeBusTopicHandler::handleWrite(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    const bool request = (pdu.flags & BEEBUS_WRITE_F_REQUEST) != 0;
    const bool wantResult = request && (pdu.flags & BEEBUS_WRITE_F_RESULT) != 0;

    // A request is applied once. A repeat (its acknowledgement was lost) is
    // acknowledged again and nothing more: its result, if it asked for one, is
    // on its way already as an operation of its own.
    if (request) {
        _protocol.notePeerSession(srcNodeID, pdu.session);
        // Made to an earlier instance of this node, which may have applied it:
        // refused, so the writer knows the outcome is not known, rather than
        // applied a second time by a window that started again.
        if (pdu.peerSession != 0 && pdu.peerSession != _protocol.session()) {
            refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_STALE);
            return BEEBUS_TOP_ERR_STALE;
        }
        if (!_protocol.acceptRequest(srcNodeID, pdu.requestID)) {
            BEEBUS_DEBUG_IO("TOPIC: Repeat of request %u from Node:%d, not applied again",
                (unsigned)pdu.requestID, srcNodeID);
            if (wantResult) _protocol.answerRepeat(srcNodeID, pdu.topicID, pdu.requestID);
            return BEEBUS_TOP_OK;
        }
    }

    uint16_t index = map.localIndex(pdu.topicID);
    if (index == BEEBUS_TOPIC_NONE) {
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_NO_TOPIC);
        return BEEBUS_TOP_ERR_NO_TOPIC;
    }

    // Read out of the record before anything calls application code. A handler
    // is free to bind or remove a topic, which reallocates the local vector and
    // leaves a reference into it dangling.
    const uint8_t boundType = map.localAt(index).last.type;
    const uint8_t boundFlags = map.localAt(index).flags;
    const bool queued = (map.localAt(index).localFlags & BEEBUS_TOPIC_L_QUEUED) != 0;
    const bool isString = bb_topicTypeIsString(boundType);

    if ((boundFlags & BEEBUS_TOPIC_F_WRITABLE) == 0) {
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_READ_ONLY);
        return BEEBUS_TOP_ERR_READ_ONLY;
    }

    // Checked against the binding rather than trusted: two nodes disagreeing
    // would otherwise reinterpret four bytes of float as an integer.
    if (pdu.value.type != boundType) {
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_TYPE);
        return BEEBUS_TOP_ERR_TYPE;
    }

    // A served topic may decide at once: its range, then its validate.
    BeeBusScalar value = pdu.value;
    uint8_t decided = 0xFF;   // Not decided here
    if (const BeeBusServed* terms = _protocol.served(pdu.topicID)) {
        const double min = terms->terms.min;
        const double max = terms->terms.max;
        const BeeBusValidateHandler validate = terms->terms.validate;   // A copy: see above
        if (!isString && min < max) {
            const double at = bb_scalarToDouble(value);
            if (at < min || at > max) decided = BEEBUS_OUTCOME_REFUSED;
        }
        if (decided == 0xFF && validate) {
            decided = validate(srcNodeID, pdu.topicID, value);
            if (decided != BEEBUS_OUTCOME_ACCEPTED && decided != BEEBUS_OUTCOME_CHANGED) {
                decided = BEEBUS_OUTCOME_REFUSED;
            }
            // A changed value has to be one the topic can hold.
            if (decided == BEEBUS_OUTCOME_CHANGED && value.type != boundType) {
                decided = BEEBUS_OUTCOME_REFUSED;
            }
        }
    }

    if (decided == BEEBUS_OUTCOME_REFUSED) {
        if (wantResult) {
            _protocol.sendResult(srcNodeID, pdu.topicID, pdu.requestID, BEEBUS_OUTCOME_REFUSED);
            return BEEBUS_TOP_OK;
        }
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_REFUSED);
        return BEEBUS_TOP_ERR_REFUSED;
    }

    // The validate is application code too: a topic it bound or removed moves
    // the records of the sorted local map, so find this one again.
    index = map.localIndex(pdu.topicID);
    if (index == BEEBUS_TOPIC_NONE || map.localAt(index).last.type != boundType) {
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_NO_TOPIC);
        return BEEBUS_TOP_ERR_NO_TOPIC;
    }

    // Undecided, it is decided once the program has run: held for a read-back.
    // So is a queued write, decided or not, which reaches the variable only
    // when the program takes it; it is read back against what the writer
    // asked for, so a value the validate changed answers CHANGED. No room to
    // hold it, and nothing is applied: the writer hears BUSY and asks again.
    const bool immediate = decided != 0xFF && !queued;
    if (wantResult && !immediate) {
        BeeBusScalar written = pdu.value;
        if (isString) {
            written.bits = bb_resultTextDigest(pdu.text.data(), pdu.text.size());
            written.high = 0;
        }
        if (!_protocol.holdResult(srcNodeID, pdu.topicID, pdu.requestID, written, !queued)) {
            _protocol.sendResult(srcNodeID, pdu.topicID, pdu.requestID, BEEBUS_OUTCOME_BUSY);
            return BEEBUS_TOP_OK;
        }
    }

    // A queued topic is not written here: the handlers below hand the value to
    // whoever takes it (the program, one write a scan), so no write is lost
    // between two scans and the program decides when it lands.
    if (!queued) {
        if (isString) {
            map.setLocalString(index, pdu.text.data(), (uint16_t)pdu.text.size());
        } else {
            bb_scalarStore(value, map.localAt(index).var);
        }
    }

    // Counted so a caller that polls can tell a peer's write from its own.
    BeeBusLocalTopic& written = map.localAt(index);
    written.writeCount = (uint16_t)(written.writeCount + 1);

    // Decided at once, answered at once.
    if (wantResult && immediate) {
        _protocol.sendResult(srcNodeID, pdu.topicID, pdu.requestID, decided);
    }

    // The record is finished with, so it is safe to call out.
    _protocol.raiseTopicWritten(srcNodeID, pdu.topicID, value);
    if (isString) {
        _protocol.raiseTopicStringWritten(srcNodeID, pdu.topicID, pdu.text.data(),
                                          (uint16_t)pdu.text.size());
    }
    return BEEBUS_TOP_OK;
}

// --- A peer asking for one of our topics ---
uint8_t BeeBusTopicHandler::handleRead(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    const uint16_t index = map.localIndex(pdu.topicID);
    if (index == BEEBUS_TOPIC_NONE) {
        refuse(srcNodeID, pdu.topicID, BEEBUS_TOP_ERR_NO_TOPIC);
        return BEEBUS_TOP_ERR_NO_TOPIC;
    }

    const BeeBusScalar value = bb_scalarLoad(map.localAt(index).last.type,
                                             map.localAt(index).var);
    _protocol.sendTopicValue(srcNodeID, index, BEEBUS_TOP_READ_REPLY, value, bb_millis());
    return BEEBUS_TOP_OK;
}

// --- Subscriptions to our topics ---
uint8_t BeeBusTopicHandler::handleSubscribe(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    // A new session is a new subscriber: what the old one had is dropped
    // before this subscription is recorded.
    _protocol.notePeerSession(srcNodeID, pdu.session);

    uint8_t status = BEEBUS_TOP_OK;
    const bool reliable = (pdu.flags & BEEBUS_SUB_Q_RELIABLE) != 0;

    const uint16_t index = map.localIndex(pdu.topicID);
    if (index == BEEBUS_TOPIC_NONE) {
        status = BEEBUS_TOP_ERR_NO_TOPIC;
    } else {
        const uint8_t contractId =
            map.internContract(pdu.deadband, bb_decodeInterval(pdu.interval));
        if (contractId == BEEBUS_SUB_NONE ||
            !map.subscribeSlot(index, srcNodeID, contractId, reliable)) {
            status = BEEBUS_TOP_ERR_NO_ROOM;
        }
    }

    // Refused, or a buffered topic: no value with the answer. A buffered
    // topic's values are its ring's, flushed in order, and its first one
    // comes out of the ring like the rest.
    BeeBusBytes reply;
    if (status != BEEBUS_TOP_OK || map.localAt(index).bufferDepth > 0) {
        const uint8_t type = status == BEEBUS_TOP_OK ? map.localAt(index).last.type : 0;
        if (BeeBusTopicFrame::buildSuback(reply, pdu.topicID, status, _protocol.session(),
                                          type)) {
            _protocol.queueTopicReply(srcNodeID, BEEBUS_TOP_SUBACK, pdu.topicID, reply);
        }
        return status;
    }

    // The answer carries the current value, so the first value cannot be lost
    // on its own: a lost SUBACK is asked for again by the subscriber's retry.
    // A RELIABLE one is numbered on the stream as it leaves; a LIVE one takes
    // the next number of the topic's own count.
    const uint16_t slot = map.slotFor(index, srcNodeID, false);
    const BeeBusLocalTopic& topic = map.localAt(index);
    const bool isString = bb_topicTypeIsString(topic.last.type);
    BeeBusScalar value;
    const char* text = nullptr;
    size_t length = 0;
    if (isString) {
        value.type = BEEBUS_T_STRING;
        text = map.localString(index);
        if (!text) text = "";
        const uint16_t cap = topic.textCap;
        length = strnlen(text, cap < BEEBUS_TOPIC_STRING_MAX ? (size_t)cap
                                                              : (size_t)BEEBUS_TOPIC_STRING_MAX);
    } else {
        value = bb_scalarLoad(topic.last.type, topic.var);
    }
    const uint8_t seq = reliable ? 0 : (uint8_t)(map.slotAt(slot).seq + 1);

    if (!BeeBusTopicFrame::buildSubackValue(reply, pdu.topicID, _protocol.session(), seq,
                                            value, text, (uint8_t)length,
                                            reliable ? BEEBUS_T_RELIABLE : 0) ||
        !_protocol.queueTopicReply(srcNodeID, BEEBUS_TOP_SUBACK, pdu.topicID, reply)) {
        // No room for the answer: the subscription stands, the scan sends the
        // value, and the subscriber asks again for its answer.
        return status;
    }

    // What went with the answer, the slot has had.
    BeeBusSubSlot& stream = map.slotAt(slot);
    if (!reliable) stream.seq = seq;
    stream.lastSent = value;
    stream.lastSentAt = bb_millis();
    stream.flags = (uint8_t)(stream.flags | BEEBUS_SUB_F_SENT);
    if (isString) map.noteStringSent(pdu.topicID, slot, text);
    map.localAt(index).lastPubAt = stream.lastSentAt;
    return status;
}

uint8_t BeeBusTopicHandler::handleSuback(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    // A new session is a restarted owner, which drops what was held of the old
    // one before this answer is taken.
    _protocol.notePeerSession(srcNodeID, pdu.session);

    // The far end's answer, unless this end cannot take what it agreed to.
    uint8_t reason = pdu.status;
    BeeBusResult result = bb_topicReasonToResult(reason);
    bool apply = false;

    const uint16_t index = map.remoteIndex(srcNodeID, pdu.topicID);
    if (index != BEEBUS_TOPIC_NONE && pdu.status == BEEBUS_TOP_OK) {
        if (pdu.value.type != map.remoteAt(index).type) {
            // Subscribed at the far end, but nothing here can store what
            // arrives, so the caller is told now rather than on the first value.
            reason = BEEBUS_TOP_ERR_TYPE;
            result = bb_topicReasonToResult(reason);
        } else {
            map.remoteAt(index).flags =
                (uint8_t)(map.remoteAt(index).flags | BEEBUS_REMOTE_F_SUBSCRIBED);
            apply = pdu.hasValue;
        }
    }

    // The first value: a RELIABLE one counted on the stream, a LIVE one on the
    // topic's own count, so an answer older than a value already here is not
    // applied over it. Never counted as a gap: it is not a publish.
    if (apply && (!pdu.reliable || _protocol.acceptStream(srcNodeID, pdu.seq))) {
        _protocol.applyTopicValue(srcNodeID, index, pdu, pdu.reliable);
    }

    _protocol.getOperations().completeTopicOp(srcNodeID, pdu.topicID, BEEBUS_TOP_SUBSCRIBE,
                                              result);
    if (result != BeeBusResult::Immediate) {
        _protocol.reportError(bb_topicReasonToError(reason), bb_topicReasonName(reason));
    }
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleUnsubscribe(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BeeBusTopicMap& map = _protocol.getTopicMap();

    const uint16_t index = map.localIndex(pdu.topicID);
    if (index != BEEBUS_TOPIC_NONE) {
        map.unsubscribeSlot(index, srcNodeID);
    }
    return BEEBUS_TOP_OK;
}

// --- Commissioning ---
uint8_t BeeBusTopicHandler::handleListRequest(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
#if BEEBUS_TOPIC_NAMES
    const bool withNames = pdu.wantNames;
#else
    const bool withNames = false;
#endif

    BeeBusBytes reply;
    uint16_t firstID = 0;
    if (!_protocol.buildTopicListPage(pdu.cursor, pdu.maxEntries, withNames,
                                      reply, firstID)) {
        return BEEBUS_TOP_ERR_MALFORMED;
    }

    // Keyed on the first topic of the page, so two pages to one node are two
    // queued replies rather than one overwriting the other.
    _protocol.queueTopicReply(srcNodeID, BEEBUS_TOP_LIST_REPLY, firstID, reply);
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleListReply(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    for (size_t i = 0; i < pdu.entries.size(); i++) {
#if BEEBUS_TOPIC_NAMES
        if (!_protocol.getTopicMap().noteDiscovered(srcNodeID, pdu.entries[i])) {
            _protocol.reportError(BeeBusError::BufferOverflow,
                                  "Out of memory for a discovered topic");
        }
#endif
        _protocol.raiseTopicList(srcNodeID, pdu.entries[i]);
    }

    // Pull the next page, so one fetch call walks the whole list.
    if (pdu.more) {
        BeeBusBytes request;
        if (BeeBusTopicFrame::buildListRequest(request, pdu.nextCursor,
                                               BEEBUS_TOPIC_LIST_MAX_ENTRIES,
                                               pdu.hasNames)) {
            _protocol.queueTopicOp(srcNodeID, BEEBUS_TOP_LIST_REQUEST, pdu.nextCursor,
                                   request, BeeBusDelivery::Unconfirmed, false, BeeBusCompletion());
        }
        return BEEBUS_TOP_OK;
    }

    // The walk is over. Any outstanding request to this node matches: a reply
    // does not say which cursor it answered, and one fetch is in flight.
    _protocol.getOperations().completeTopicOp(srcNodeID, -1, BEEBUS_TOP_LIST_REQUEST,
                                              BeeBusResult::Immediate);
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleRegister(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    if (pdu.entries.empty()) return BEEBUS_TOP_ERR_MALFORMED;

#if BEEBUS_TOPIC_NAMES
    if (!_protocol.getTopicMap().noteDiscovered(srcNodeID, pdu.entries[0])) {
        _protocol.reportError(BeeBusError::BufferOverflow, "Out of memory for a discovered topic");
    }
#endif
    _protocol.raiseTopicList(srcNodeID, pdu.entries[0]);
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleError(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    BEEBUS_DEBUG_IO("TOPIC: Node:%d refused topic %u: %s",
        srcNodeID, (unsigned)pdu.topicID, bb_topicReasonName(pdu.status));

    // Whatever asked for that topic is told why.
    _protocol.getOperations().completeTopicOp(srcNodeID, pdu.topicID, -1,
                                              bb_topicReasonToResult(pdu.status));
    _protocol.reportError(bb_topicReasonToError(pdu.status), bb_topicReasonName(pdu.status));
    return BEEBUS_TOP_OK;
}

// --- The reliable layer ---
uint8_t BeeBusTopicHandler::handleStreamAck(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    _protocol.handleStreamAck(srcNodeID, pdu);
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleResult(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    _protocol.handleResult(srcNodeID, pdu);
    return BEEBUS_TOP_OK;
}

uint8_t BeeBusTopicHandler::handleSession(const BeeBusTopicPdu& pdu, uint8_t srcNodeID) {
    _protocol.notePeerSession(srcNodeID, pdu.session);
    return BEEBUS_TOP_OK;
}
