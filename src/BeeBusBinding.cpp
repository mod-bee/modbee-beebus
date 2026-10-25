#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// Layer 3: serve / bind, built only from the layer 2 primitives. A served
// topic is a writable topic whose writes are decided by its owner and answered
// with a RESULT; a bind is a RELIABLE subscription to it plus writes that ask
// for that result. No timers or retries here beyond the protocol's own: the
// stream repairs values, the confirmed write repairs requests.

namespace {

uint32_t bindKey(uint8_t ownerID, uint16_t topicID) {
    return ((uint32_t)ownerID << 16) | topicID;
}

// The caller's variable as a value: its text's digest for a string bind, so a
// change of text is noticed without holding a second copy to compare against.
BeeBusScalar loadBound(const BeeBusBinding& bound) {
    if (bb_topicTypeIsString(bound.type)) {
        BeeBusScalar value;
        value.type = BEEBUS_T_STRING;
        const char* text = (const char*)bound.var;
        value.bits = bb_resultTextDigest(text, strnlen(text, bound.textCap));
        return value;
    }
    return bb_scalarLoad(bound.type, bound.var);
}

}  // namespace

// --- Serve ---
bool BeeBusProtocol::serve(uint16_t topicID, uint8_t type, void* variable,
                           const BeeBusServe& terms, const char* name, uint16_t textCap) {
    if (served(topicID)) return false;
    if (_served.full() && !_served.grow(_served.capacity() + 8)) return false;
    if (!_topicMap.addLocal(topicID, type, variable, terms.writable, name, textCap)) return false;

    size_t at = 0;
    while (at < _served.size() && _served[at].topicID < topicID) at++;
    BeeBusServed record;
    record.terms = terms;
    record.topicID = topicID;
    if (!_served.insertAt(at, record)) {
        _topicMap.removeLocal(topicID);
        return false;
    }
    return true;
}

bool BeeBusProtocol::unserve(uint16_t topicID) {
    for (size_t i = 0; i < _served.size(); i++) {
        if (_served[i].topicID != topicID) continue;
        _served.eraseAt(i);
        _topicMap.removeLocal(topicID);
        return true;
    }
    return false;
}

// --- Bind: the table ---
size_t BeeBusProtocol::bindingLowerBound(uint32_t key) const {
    size_t lo = 0;
    size_t hi = _bindings.size();
    while (lo < hi) {
        const size_t mid = (lo + hi) / 2;
        if (_bindings[mid].key < key) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

BeeBusBinding* BeeBusProtocol::binding(uint8_t ownerID, uint16_t topicID) {
    const uint32_t key = bindKey(ownerID, topicID);
    const size_t at = bindingLowerBound(key);
    return at < _bindings.size() && _bindings[at].key == key ? &_bindings[at] : nullptr;
}

const BeeBusBinding* BeeBusProtocol::binding(uint8_t ownerID, uint16_t topicID) const {
    const uint32_t key = bindKey(ownerID, topicID);
    const size_t at = bindingLowerBound(key);
    return at < _bindings.size() && _bindings[at].key == key ? &_bindings[at] : nullptr;
}

bool BeeBusProtocol::bind(uint8_t ownerID, uint16_t topicID, uint8_t type, void* variable,
                          const BeeBusBindTerms& terms, const char* name, uint16_t textCap) {
    if (ownerID == 0 || ownerID == _nodeID || ownerID == BEEBUS_NODE_BROADCAST || !variable) {
        return false;
    }
    if (binding(ownerID, topicID)) return false;
    // A topic the application registered itself is its own, not the bind's.
    if (_topicMap.remoteIndex(ownerID, topicID) != BEEBUS_TOPIC_NONE) return false;

    const bool isString = bb_topicTypeIsString(type);
    if (isString && textCap < 2) return false;
    if (_bindings.full() && !_bindings.grow(_bindings.capacity() + 8)) return false;

    BeeBusBinding bound;
    bound.key = bindKey(ownerID, topicID);
    bound.var = variable;
    bound.type = (uint8_t)(type & BEEBUS_T_MASK);
    bound.textCap = isString ? textCap : 0;
    bound.shadow = new (std::nothrow) uint8_t[isString ? textCap : 8]();
    if (isString) bound.syncedText = new (std::nothrow) char[textCap]();
    if (!bound.shadow || (isString && !bound.syncedText) ||
        !_topicMap.addRemote(ownerID, topicID, type, bound.shadow, name, textCap)) {
        delete[] bound.shadow;
        delete[] bound.syncedText;
        return false;
    }
    bound.synced = loadBound(bound);
    bound.state = isMember(ownerID) ? BeeBusBindState::Syncing : BeeBusBindState::Offline;
    bound.flags = BEEBUS_BIND_F_EVENT;
    _bindings.insertAt(bindingLowerBound(bound.key), bound);

    // The subscription, on the terms asked: RELIABLE carries every change.
    const bool reliable = terms.qos == BeeBusQoS::Reliable;
    const uint16_t index = _topicMap.remoteIndex(ownerID, topicID);
    const uint8_t contractId = _topicMap.internContract(reliable ? 0.0f : terms.deadband, 0);
    if (contractId != BEEBUS_SUB_NONE) {
        BeeBusRemoteTopic& remote = _topicMap.remoteAt(index);
        remote.contractId = contractId;
        remote.flags = (uint8_t)((remote.flags & ~BEEBUS_REMOTE_F_SUBSCRIBED) |
                                 (reliable ? BEEBUS_REMOTE_F_RELIABLE : 0));
        issueSubscribe(ownerID, topicID);
    }
    return true;
}

bool BeeBusProtocol::unbind(uint8_t ownerID, uint16_t topicID) {
    const uint32_t key = bindKey(ownerID, topicID);
    const size_t at = bindingLowerBound(key);
    if (at >= _bindings.size() || _bindings[at].key != key) return false;

    dropDeferredSubscribe(ownerID, topicID);
    if (isMember(ownerID)) {
        BeeBusBytes pdu;
        if (BeeBusTopicFrame::buildUnsubscribe(pdu, topicID)) {
            queueTopicOp(ownerID, BEEBUS_TOP_UNSUBSCRIBE, topicID, pdu,
                         BeeBusDelivery::Unconfirmed, false,
                         bb_completion(&BeeBusProtocol::silentCompletion, this));
        }
    }
    _topicMap.removeRemote(ownerID, topicID);
    _topicMap.releaseUnusedContracts();

    delete[] _bindings[at].shadow;
    delete[] _bindings[at].syncedText;
    _bindings.eraseAt(at);
    return true;
}

BeeBusBindState BeeBusProtocol::bindState(uint8_t ownerID, uint16_t topicID) const {
    const BeeBusBinding* bound = binding(ownerID, topicID);
    return bound ? bound->state : BeeBusBindState::Offline;
}

bool BeeBusProtocol::bindResult(uint8_t ownerID, uint16_t topicID, uint8_t& outcome,
                                BeeBusScalar& kept) const {
    const BeeBusBinding* bound = binding(ownerID, topicID);
    if (!bound) return false;
    outcome = bound->outcome;
    kept = bound->kept;
    return true;
}

// --- Bind: state ---
void BeeBusProtocol::setBindState(BeeBusBinding& bound, BeeBusBindState state) {
    if (bound.state == state) return;
    bound.state = state;
    // Raised from serviceBindings(), outside every walk of the table: the
    // handler is application code and may bind or unbind.
    bound.flags = (uint8_t)(bound.flags | BEEBUS_BIND_F_EVENT);
}

void BeeBusProtocol::copyShadow(BeeBusBinding& bound) {
    if (bb_topicTypeIsString(bound.type)) {
        char* text = (char*)bound.var;
        const char* held = (const char*)bound.shadow;
        const size_t n = strnlen(held, (size_t)bound.textCap - 1);
        memcpy(text, held, n);
        text[n] = '\0';
        memcpy(bound.syncedText, held, n);
        bound.syncedText[n] = '\0';
    } else {
        bb_scalarStore(bb_scalarLoad(bound.type, bound.shadow), bound.var);
    }
    bound.synced = loadBound(bound);
}

void BeeBusProtocol::bindValueArrived(uint8_t ownerID, uint16_t topicID) {
    BeeBusBinding* bound = binding(ownerID, topicID);
    if (!bound) return;
    bound->flags = (uint8_t)(bound->flags | BEEBUS_BIND_F_VALUE);

    // A change of the caller's is with the owner: its value waits for the
    // decision, so the variable does not flick back meanwhile.
    if (bound->state == BeeBusBindState::Pending) return;
    copyShadow(*bound);
    setBindState(*bound, BeeBusBindState::InSync);
}

// --- Bind: changes ---
void BeeBusProtocol::sendBindChange(BeeBusBinding& bound) {
    const uint8_t ownerID = (uint8_t)(bound.key >> 16);
    const uint16_t topicID = (uint16_t)(bound.key & 0xFFFF);
    const uint8_t flags = BEEBUS_WRITE_F_REQUEST | BEEBUS_WRITE_F_RESULT;

    BeeBusBytes pdu;
    bool built;
    if (bb_topicTypeIsString(bound.type)) {
        const char* text = (const char*)bound.var;
        size_t length = strnlen(text, bound.textCap);
        if (length > BEEBUS_TOPIC_STRING_MAX) length = BEEBUS_TOPIC_STRING_MAX;
        built = BeeBusTopicFrame::buildWriteString(pdu, topicID, text, (uint8_t)length, flags,
                                                   0, _session);
    } else {
        built = BeeBusTopicFrame::buildWrite(pdu, topicID, bound.requested, flags, 0, _session);
    }
    if (!built) {
        bound.flags = (uint8_t)(bound.flags & ~BEEBUS_BIND_F_RESEND);
        copyShadow(bound);
        setBindState(bound, BeeBusBindState::Failed);
        return;
    }

    // The request ID is given as the write goes into a frame, so a change
    // replacing one still queued takes its place under one ID.
    const BeeBusResult result =
        queueTopicOp(ownerID, BEEBUS_TOP_WRITE, topicID, pdu, BeeBusDelivery::Confirmed, false,
                     bb_completion(&BeeBusProtocol::bindWriteCompleted, this));
    if (result == BeeBusResult::QueueFull) return;   // Still due: next loop()

    BeeBusBinding* again = binding(ownerID, topicID);
    if (!again) return;
    again->flags = (uint8_t)(again->flags & ~(BEEBUS_BIND_F_RESEND | BEEBUS_BIND_F_ACKED));
    again->requestID = 0;
    again->reasks = 0;
    if (!bb_accepted(result)) {
        copyShadow(*again);
        setBindState(*again, BeeBusBindState::Failed);
    }
}

void BeeBusProtocol::bindWriteCompleted(void* ctx, uint8_t nodeID, uint16_t topicID,
                                        uint8_t opcode, BeeBusResult result) {
    (void)opcode;
    BeeBusProtocol& self = *static_cast<BeeBusProtocol*>(ctx);
    BeeBusBinding* bound = self.binding(nodeID, topicID);
    if (!bound || bound->state != BeeBusBindState::Pending) return;

    switch (result) {
        case BeeBusResult::Immediate:
            // The owner has it; its decision follows.
            bound->flags = (uint8_t)(bound->flags | BEEBUS_BIND_F_ACKED);
            bound->ackedAt = bb_millis();
            return;
        case BeeBusResult::Superseded:
            return;   // A later change took its place
        case BeeBusResult::Refused:
        case BeeBusResult::NotMapped:
        case BeeBusResult::TypeMismatch:
            // Refused outright (read only, no such topic, the wrong type): the
            // owner keeps what it has.
            bound->outcome = result == BeeBusResult::Refused ? BEEBUS_OUTCOME_REFUSED
                           : result == BeeBusResult::NotMapped ? BEEBUS_OUTCOME_NO_TOPIC
                                                               : BEEBUS_OUTCOME_TYPE;
            self.copyShadow(*bound);
            self.setBindState(*bound, BeeBusBindState::Refused);
            return;
        default:
            bound->outcome = BEEBUS_OUTCOME_BUSY;
            self.copyShadow(*bound);
            self.setBindState(*bound, BeeBusBindState::Failed);
            return;
    }
}

void BeeBusProtocol::bindChanged(uint8_t ownerID, uint16_t topicID) {
    BeeBusBinding* bound = binding(ownerID, topicID);
    if (!bound || (bound->flags & BEEBUS_BIND_F_VALUE) == 0) return;
    if (bound->state == BeeBusBindState::Offline || bound->state == BeeBusBindState::Syncing) return;

    const BeeBusScalar now = loadBound(*bound);
    const bool pending = bound->state == BeeBusBindState::Pending;
    if (pending ? bb_scalarEqual(now, bound->requested) : bb_scalarEqual(now, bound->synced)) {
        return;
    }
    bound->requested = bb_topicTypeIsString(bound->type) ? now : bb_scalarLoad(bound->type, bound->var);
    bound->flags = (uint8_t)(bound->flags | BEEBUS_BIND_F_RESEND);
    setBindState(*bound, BeeBusBindState::Pending);
    sendBindChange(*bound);
}

void BeeBusProtocol::serviceBindings() {
    if (_bindings.empty()) return;

    for (size_t i = 0; i < _bindings.size(); i++) {
        BeeBusBinding& bound = _bindings[i];
        const uint8_t ownerID = (uint8_t)(bound.key >> 16);

        // Gone: the variable takes the owner's last value, and nothing is
        // compared until a new one arrives.
        if (!isMember(ownerID)) {
            if (bound.state != BeeBusBindState::Offline) {
                if (bound.state == BeeBusBindState::Pending && (bound.flags & BEEBUS_BIND_F_VALUE)) {
                    copyShadow(bound);
                }
                bound.flags = (uint8_t)(bound.flags & BEEBUS_BIND_F_EVENT);
                setBindState(bound, BeeBusBindState::Offline);
            }
            continue;
        }
        if (bound.state == BeeBusBindState::Offline) {
            setBindState(bound, BeeBusBindState::Syncing);
        }

        if (bound.flags & BEEBUS_BIND_F_RESEND) {
            sendBindChange(bound);
            continue;
        }
        bindChanged(ownerID, (uint16_t)(bound.key & 0xFFFF));
    }

    // The state changes, each raised once, outside the walks above.
    if (!_bindStateHandler) {
        for (size_t i = 0; i < _bindings.size(); i++) {
            _bindings[i].flags = (uint8_t)(_bindings[i].flags & ~BEEBUS_BIND_F_EVENT);
        }
        return;
    }
    for (size_t i = 0; i < _bindings.size(); i++) {
        if ((_bindings[i].flags & BEEBUS_BIND_F_EVENT) == 0) continue;
        _bindings[i].flags = (uint8_t)(_bindings[i].flags & ~BEEBUS_BIND_F_EVENT);
        const uint32_t key = _bindings[i].key;
        const BeeBusBindState state = _bindings[i].state;
        _bindStateHandler((uint8_t)(key >> 16), (uint16_t)(key & 0xFFFF), state);
    }
}
