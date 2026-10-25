#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// --- Constructor and destructor ---
BeeBusOperations::BeeBusOperations() {
    // The queues take their storage in begin(), when the protocol starts.
}

bool BeeBusOperations::begin(size_t maxOps, size_t maxResponses) {
    bool ok = _pendingOps.reserve(maxOps);
    ok = _pendingResponses.reserve(maxResponses) && ok;
    ok = _finished.reserve(maxOps) && ok;
    ok = _retiring.reserve(maxOps) && ok;
    // Outcomes owed while a handler runs, and those of operations served in
    // place: each is at most one per operation the queues can hold, twice over
    // for the chains a handler can start.
    ok = _deferred.reserve(2 * maxOps) && ok;
    ok = _localCompletions.reserve(2 * maxOps) && ok;
    ok = frameReady.reserve(maxOps) && ok;
    ok = frameUnbuildable.reserve(maxOps) && ok;
    ok = frameSent.reserve(maxOps) && ok;
    ok = frameResponses.reserve(maxResponses) && ok;
    return ok;
}

BeeBusOperations::Owed BeeBusOperations::owed(const BeeBusCompletion& handler,
                                              const BeeBusPendingModbusOp& op,
                                              BeeBusResult result) {
    Owed entry;
    entry.handler = handler;
    entry.nodeID = op.destNodeID;
    entry.address = op.req.startAddr;
    entry.function = op.req.function;
    entry.result = result;
    return entry;
}

BeeBusOperations::~BeeBusOperations() {
    // Destructor - clear all containers
    clearPendingOperations();
    clearPendingResponses();
}

// --- Operation management ---
BeeBusResult BeeBusOperations::addPendingOperation(const BeeBusPendingModbusOp& op, BeeBusProtocol& protocol) {
    // Check if we already have too many pending operations
    // A payload that did not fit was cut short, and must not go out as if whole.
    if (op.req.data.overflow()) {
        return BeeBusResult::InvalidArgument;
    }
    if (_pendingOps.full()) {
        protocol.reportError(BeeBusError::BufferOverflow, "Too many pending operations");
        return BeeBusResult::QueueFull;
    }

    for (auto& existingOp : _pendingOps) {
        // A different PDU family is a different operation whatever the numbers
        // happen to be: 0x06 is a Modbus single-register write and a topic
        // UNSUBSCRIBE. First line, so the register path below is provably
        // untouched by the topic one.
        if (existingOp.pduType != op.pduType) {
            continue;
        }

        if (existingOp.destNodeID != op.destNodeID ||
            existingOp.req.function != op.req.function ||
            existingOp.req.startAddr != op.req.startAddr ||
            existingOp.req.quantity != op.req.quantity) {
            continue;
        }

        // A write that matches one still waiting to go out supersedes it. The
        // application is saying what the register should be, not queuing a
        // history of what it should have been, and a control loop updating a
        // setpoint faster than the token rotates does this constantly. Rejecting
        // it would leave the register holding a value the application has
        // already moved on from.
        //
        // A pending write has not been transmitted: writes are retired the
        // moment they are sent, so the payload can safely be replaced in place.
        // A history is not a current value. A store-and-forward flush and a
        // publish each carry a sequence number of their own, and collapsing two
        // of them would leave a number on the wire nothing was ever sent under.
        if (existingOp.noSupersede || op.noSupersede) {
            continue;
        }

        // A topic write that has carried its request ID onto the wire may have
        // been applied, whatever its acknowledgement says. While it waits to be
        // retried it counts as on the wire: the new write queues behind it, as
        // behind one awaiting acknowledgement below, or a command could be
        // lost and a value applied out of order.
        if (bb_topicRequestNumbered(existingOp)) {
            break;
        }

        if (!bb_opExpectsResponse(op)) {
            // A write awaiting acknowledgement is already on the wire. Rewriting
            // its payload now would leave the acknowledgement, when it arrives,
            // confirming a value that was never the one transmitted. Let the new
            // write queue separately behind it instead.
            if (existingOp.awaitingResponse) {
                break;
            }

            if (existingOp.req.data == op.req.data && existingOp.wantsAck == op.wantsAck) {
                return BeeBusResult::Duplicate;   // Same value, nothing to do
            }

            // The queued value never reaches the wire, so anything waiting on it
            // is told rather than left outstanding for ever. Taken by value
            // first: the handler is application code and may queue an operation
            // of its own, which moves the queue's items and leaves every
            // reference into it, existingOp included, on a different operation.
            const Owed superseded = owed(existingOp.onComplete, existingOp,
                                         BeeBusResult::Superseded);

            // Every field the new operation carries, not just its payload. The
            // delivery mode and completion handler belong to the caller that
            // asked for them, so a confirmed write replacing an unconfirmed one
            // must not inherit the old one's terms.
            existingOp.req.data = op.req.data;
            existingOp.timestamp = op.timestamp;
            existingOp.retryCount = 0;
            existingOp.wantsAck = op.wantsAck;
            existingOp.onComplete = op.onComplete;
            existingOp.opId = nextOpId();

            BEEBUS_DEBUG_OPERATIONS("SUPERSEDED: Node:%d FC:%02X Addr:%d carries a new value",
                op.destNodeID, op.req.function, op.req.startAddr);

            // The record is fully written, so nothing here reads it again and
            // the handler is free to queue.
            dispatch(superseded);
            return BeeBusResult::Queued;
        }

        // The same read twice is genuinely redundant.
        return BeeBusResult::Duplicate;
    }

    // Stamp the operation with an ID as it enters the queue. Everything that
    // later has to find this exact operation again matches on it.
    BeeBusPendingModbusOp queued = op;
    queued.opId = nextOpId();
    dropFinished(op.destNodeID, op.req.startAddr);
    _pendingOps.push_back(queued);

    BEEBUS_DEBUG_OPERATIONS("ADDED: Op %d/%d - Node:%d FC:%02X Addr:%d Qty:%d",
        _pendingOps.size(), _pendingOps.capacity(), op.destNodeID, op.req.function, op.req.startAddr, op.req.quantity);
    return BeeBusResult::Queued;
}

bool BeeBusOperations::addPendingResponse(const BeeBusPendingResponse& response) {
    // Check if we already have too many pending responses
    if (_pendingResponses.full() || response.response.data.overflow()) {
        BEEBUS_DEBUG_OPERATIONS("REJECTED: Response queue full (%d/%d) - FC:%02X",
            _pendingResponses.size(), _pendingResponses.capacity(), response.response.function);
        return false;
    }

    _pendingResponses.push_back(response);

    BEEBUS_DEBUG_OPERATIONS("RESPONSE ADDED: %d/%d - FC:%02X Addr:%d TO Node:%d FROM Node:%d",
        _pendingResponses.size(), _pendingResponses.capacity(),
        response.response.function, response.response.startAddr,
        response.destNodeID, response.sourceNodeID);
    return true;
}

// --- Completions for operations served in place ---
void BeeBusOperations::reportLocal(const BeeBusCompletion& handler, uint8_t nodeID,
                                   uint16_t address, uint8_t function, BeeBusResult result) {
    if (!handler) {
        return;
    }
    Owed entry;
    entry.handler = handler;
    entry.nodeID = nodeID;
    entry.address = address;
    entry.function = function;
    entry.result = result;
    // Full only if the application served more operations in place between two
    // loops than the queues hold, twice over; that one is reported now.
    if (!_localCompletions.push_back(entry)) {
        dispatch(entry);
    }
}

void BeeBusOperations::dispatchLocalCompletions() {
    // Only those owed when this drain began: a handler that reads or writes a
    // local address owes a completion of its own, and that one belongs to the
    // next loop rather than to this drain, which would otherwise not terminate.
    size_t count = _localCompletions.size();
    while (count-- > 0 && !_localCompletions.empty()) {
        const Owed entry = _localCompletions.front();
        _localCompletions.eraseAt(0);
        dispatch(entry);
    }
}

// --- Re-entrancy ---
void BeeBusOperations::dispatch(const Owed& entry) {
    if (!entry.handler) {
        return;
    }

    // Already inside one. Keep the outcome and let the outermost call run it,
    // so the reporting stays flat however the application chains its work. The
    // queue holds two per operation; a chain longer than that runs nested.
    if (_dispatchDepth != 0) {
        if (_deferred.push_back(entry)) {
            return;
        }
    }

    const uint8_t depth = _dispatchDepth;
    _dispatchDepth = 1;
    entry.handler(entry.nodeID, entry.address, entry.function, entry.result);
    if (depth == 0) {
        while (!_deferred.empty()) {
            const Owed next = _deferred.front();
            _deferred.eraseAt(0);
            next.handler(next.nodeID, next.address, next.function, next.result);
        }
    }
    _dispatchDepth = depth;
}

void BeeBusOperations::dispatchRetiring() {
    // Outside any handler: each runs in turn, with whatever it starts drained
    // before the next, as dispatch() does for a single one.
    while (!_retiring.empty()) {
        const Owed entry = _retiring.front();
        _retiring.eraseAt(0);
        dispatch(entry);
    }
}

void BeeBusOperations::retireAt(size_t index, BeeBusResult result) {
    if (index >= _pendingOps.size()) {
        return;
    }

    // Taken off the record before the erase, and the erase done before the
    // call: by the time application code runs, this operation is no longer in
    // the queue and the handler may queue whatever it likes.
    const BeeBusPendingModbusOp& op = _pendingOps[index];
    const Owed entry = owed(op.onComplete, op, result);
    _pendingOps.eraseAt(index);

    recordFinished(entry.nodeID, entry.address, result);
    dispatch(entry);
}

void BeeBusOperations::noteCompleted(uint8_t nodeID, uint16_t address, BeeBusResult result) {
    recordFinished(nodeID, address, result);
}

void BeeBusOperations::recordFinished(uint8_t nodeID, uint16_t address, BeeBusResult result) {
    for (size_t i = 0; i < _finished.size(); i++) {
        if (_finished[i].nodeID == nodeID && _finished[i].address == address) {
            _finished[i].result = result;
            return;
        }
    }

    // Bounded by the queue it mirrors: one entry per address with an operation
    // that ran, dropped as soon as the caller issues the next one.
    if (_finished.full()) {
        _finished.eraseAt(0);
    }
    FinishedOp entry;
    entry.nodeID = nodeID;
    entry.address = address;
    entry.result = result;
    _finished.push_back(entry);
}

void BeeBusOperations::dropFinished(uint8_t nodeID, uint16_t address) {
    for (size_t i = 0; i < _finished.size(); i++) {
        if (_finished[i].nodeID == nodeID && _finished[i].address == address) {
            _finished.eraseAt(i);
            return;
        }
    }
}

BeeBusRequestState BeeBusOperations::requestState(uint8_t nodeID, uint16_t address,
                                                  BeeBusResult& result) const {
    for (size_t i = 0; i < _pendingOps.size(); i++) {
        if (_pendingOps[i].destNodeID == nodeID && _pendingOps[i].req.startAddr == address) {
            result = BeeBusResult::Queued;
            return BeeBusRequestState::Pending;
        }
    }
    for (size_t i = 0; i < _finished.size(); i++) {
        if (_finished[i].nodeID == nodeID && _finished[i].address == address) {
            result = _finished[i].result;
            return _finished[i].result == BeeBusResult::Immediate
                       ? BeeBusRequestState::Done
                       : BeeBusRequestState::Failed;
        }
    }
    result = BeeBusResult::Immediate;
    return BeeBusRequestState::Idle;
}

// --- Cleanup and maintenance ---
void BeeBusOperations::cleanupTimedOutOperations(BeeBusProtocol& protocol) {
    bb_time_t now = bb_millis();

    // An operation gets a few rotations to complete: its request has to reach
    // the destination on one token and the answer come back on another, so the
    // allowance has to be a multiple of what a rotation costs here rather than
    // a fixed number of milliseconds.
    const bb_time_t operationLimit =
        protocol.ringTimeout(bb_setting<uint8_t>(protocol.config().rotationsOperation, BEEBUS_ROTATIONS_OPERATION), protocol.config().operationTimeout,
                             BEEBUS_REFERENCE_OPERATION);
    const bb_time_t responseLimit =
        protocol.ringTimeout(bb_setting<uint8_t>(protocol.config().rotationsOperation, BEEBUS_ROTATIONS_OPERATION), protocol.config().responseTimeout,
                             BEEBUS_REFERENCE_OPERATION);
    int removedOps = 0;
    int retriedOps = 0;
    int removedResponses = 0;

    // Clean up timed out operations. Walked by index rather than by iterator:
    // retireAt() calls application code, which may queue an operation and
    // reallocate the vector out from under an iterator.
    const uint8_t retryLimit = bb_retryLimit(protocol.config().maxRetries);

    for (size_t i = 0; i < _pendingOps.size();) {
        if (bb_elapsed(_pendingOps[i].timestamp, now) <= operationLimit) {
            i++;
            continue;
        }

        // Check if we should retry
        if (_pendingOps[i].retryCount < retryLimit) {
            // Retry the operation
            _pendingOps[i].timestamp = now;
            _pendingOps[i].retryCount++;
            _pendingOps[i].awaitingResponse = false;   // Allow it to be sent again
            retriedOps++;
            BEEBUS_DEBUG_OPERATIONS("RETRY: Node:%d FC:%02X Addr:%d (attempt %d/%d)",
                _pendingOps[i].destNodeID, _pendingOps[i].req.function,
                _pendingOps[i].req.startAddr, _pendingOps[i].retryCount, retryLimit);
            i++;
            continue;
        }

        // Out of retries. A confirmed write asked to be told what happened to
        // it, and "nothing" is an answer the application needs: dropped
        // silently, a lost setpoint looks exactly like a delivered one.
        BEEBUS_DEBUG_OPERATIONS("TIMEOUT: Removing Node:%d FC:%02X Addr:%d after %d retries",
            _pendingOps[i].destNodeID, _pendingOps[i].req.function,
            _pendingOps[i].req.startAddr, _pendingOps[i].retryCount);

        const bool wantedAck = _pendingOps[i].wantsAck;
        retireAt(i, BeeBusResult::Timeout);
        removedOps++;

        if (wantedAck) {
            protocol.reportError(BeeBusError::OperationTimeout,
                                 "Confirmed write was never acknowledged");
        }
        // The entry at i is now whatever followed it, so i does not advance.
    }

    // Clean up timed out responses
    for (auto it = _pendingResponses.begin(); it != _pendingResponses.end();) {
        if (bb_elapsed(it->timestamp, now) > responseLimit) {
            BEEBUS_DEBUG_OPERATIONS("RESPONSE TIMEOUT: Removing FC:%02X Addr:%d",
                it->response.function, it->response.startAddr);
            it = _pendingResponses.erase(it);
            removedResponses++;
        } else {
            ++it;
        }
    }

    if (removedOps > 0 || retriedOps > 0 || removedResponses > 0) {
        BEEBUS_DEBUG_OPERATIONS("CLEANUP: Removed:%d ops, %d responses; Retried:%d",
            removedOps, removedResponses, retriedOps);
    }
}

void BeeBusOperations::clearPendingOperations() {
    // Every operation is off the queue before any handler hears it was
    // cancelled, so a handler that queues again finds an empty queue.
    const int count = (int)_pendingOps.size();
    if (_dispatchDepth != 0) {
        // Inside a handler: dispatch() keeps each outcome for the outermost
        // call, after this one returns.
        for (size_t i = 0; i < _pendingOps.size(); i++) {
            dispatch(owed(_pendingOps[i].onComplete, _pendingOps[i], BeeBusResult::Cancelled));
        }
        _pendingOps.clear();
    } else {
        for (size_t i = 0; i < _pendingOps.size(); i++) {
            if (_pendingOps[i].onComplete) {
                _retiring.push_back(owed(_pendingOps[i].onComplete, _pendingOps[i],
                                         BeeBusResult::Cancelled));
            }
        }
        _pendingOps.clear();
        dispatchRetiring();
    }

    if (count > 0) {
        BEEBUS_DEBUG_OPERATIONS("CLEARED: %d pending operations", count);
    }
}

void BeeBusOperations::clearPendingResponses() {
    int count = _pendingResponses.size();
    _pendingResponses.clear();
    if (count > 0) {
        BEEBUS_DEBUG_OPERATIONS("CLEARED: %d pending responses", count);
    }
}

// --- Accessor methods ---
BeeBusList<BeeBusPendingModbusOp>& BeeBusOperations::getPendingOps() {
    return _pendingOps;
}

const BeeBusList<BeeBusPendingModbusOp>& BeeBusOperations::getPendingOps() const {
    return _pendingOps;
}

BeeBusList<BeeBusPendingResponse>& BeeBusOperations::getPendingResponses() {
    return _pendingResponses;
}

const BeeBusList<BeeBusPendingResponse>& BeeBusOperations::getPendingResponses() const {
    return _pendingResponses;
}

uint16_t BeeBusOperations::getPendingOpCount() const {
    return _pendingOps.size();
}

uint16_t BeeBusOperations::getPendingResponseCount() const {
    return _pendingResponses.size();
}

// --- Node-specific operations ---

void BeeBusOperations::clearNodeOperations(uint8_t nodeID) {
    // The node is gone, so are its operations. Every one comes off the queue
    // before any handler hears about it, as for clearPendingOperations().
    const bool nested = _dispatchDepth != 0;
    for (size_t i = 0; i < _pendingOps.size();) {
        if (_pendingOps[i].destNodeID != nodeID) {
            i++;
            continue;
        }
        if (_pendingOps[i].onComplete) {
            const Owed entry = owed(_pendingOps[i].onComplete, _pendingOps[i],
                                    BeeBusResult::NodeLost);
            if (nested) {
                _pendingOps.eraseAt(i);
                dispatch(entry);
                continue;
            }
            _retiring.push_back(entry);
        }
        _pendingOps.eraseAt(i);
    }
    BEEBUS_DEBUG_OPERATIONS("CLEARED: All operations for Node %d", nodeID);
    if (!nested) {
        dispatchRetiring();
    }
}

void BeeBusOperations::applyFailsafeForNode(uint8_t nodeID) {
    BEEBUS_DEBUG_OPERATIONS("APPLYING FAILSAFE: Resetting variables for lost Node %d", nodeID);
    int cleared_vars = 0;

    // Walked by index rather than by iterator: retireAt() calls application
    // code, which may queue an operation and reallocate the vector.
    for (size_t i = 0; i < _pendingOps.size();) {
        // Check if the operation is for the lost node and has a result pointer.
        // A topic read leaves the result in its bound variable rather than in a
        // buffer of the caller's, so there is nothing here to clear.
        const BeeBusPendingModbusOp& op = _pendingOps[i];
        if (op.pduType != BEEBUS_PDU_MODBUS || op.destNodeID != nodeID ||
            op.resultPtr == nullptr) {
            // Not for the target node or no result pointer, just move on
            i++;
            continue;
        }

        // Determine the data type from the function code and reset the variable(s)
        switch (op.req.function) {
            case MB_FC_READ_COILS:
            case MB_FC_READ_DISCRETE_INPUTS: {
                for (uint16_t v = 0; v < op.req.quantity; ++v) {
                    *bb_elemAt<bool>(op.resultPtr, v, op.resultStride) = false;
                }
                break;
            }
            case MB_FC_READ_HOLDING_REGISTERS:
            case MB_FC_READ_INPUT_REGISTERS: {
                if (op.resultKind == BeeBusResultKind::Text) {
                    static_cast<char*>(op.resultPtr)[0] = '\0';
                    break;
                }
                for (uint16_t v = 0; v < op.req.quantity; ++v) {
                    *bb_elemAt<int16_t>(op.resultPtr, v, op.resultStride) = 0;
                }
                break;
            }
            default:
                // This operation is not a read operation with a result pointer,
                // so we don't need to do anything for failsafe.
                break;
        }
        cleared_vars++;

        // The read cannot now be answered, and its caller asked to be told.
        retireAt(i, BeeBusResult::NodeLost);
    }

    if (cleared_vars > 0) {
        BEEBUS_DEBUG_OPERATIONS("FAILSAFE APPLIED: Cleared %d variables and removed operations for Node %d", cleared_vars, nodeID);
    }
}

// --- Operation processing ---
void BeeBusOperations::processPendingOperations(BeeBusProtocol& protocol) {
    // This method is called to check if any operations need processing
    // The actual sending is done when the protocol has the token

    // Check for operations that need retry
    int readyOps = 0;

    for (auto& op : _pendingOps) {
        if (isOperationReady(op, protocol)) {
            readyOps++;
        }
    }

    if (readyOps > 0) {
        BEEBUS_DEBUG_OPERATIONS("READY: %d operation(s) waiting for the token", readyOps);
    }
}

void BeeBusOperations::clearPendingOps() {
    clearPendingOperations();
}

void BeeBusOperations::markAwaitingResponse(const BeeBusPendingModbusOp& op) {
    markAwaitingResponse(op.destNodeID, op.opId);
}

void BeeBusOperations::markAwaitingResponse(uint8_t destNodeID, uint16_t opId) {
    for (auto& pending : _pendingOps) {
        if (pending.opId == opId && pending.destNodeID == destNodeID) {
            pending.awaitingResponse = true;
            pending.timestamp = bb_millis();   // Time out from transmission
            return;
        }
    }
}

void BeeBusOperations::removePendingOperation(const BeeBusPendingModbusOp& op,
                                             BeeBusResult result) {
    removePendingOperation(op.destNodeID, op.opId, result);
}

void BeeBusOperations::removePendingOperation(uint8_t destNodeID, uint16_t opId,
                                             BeeBusResult result) {
    for (size_t i = 0; i < _pendingOps.size(); i++) {
        if (_pendingOps[i].opId == opId && _pendingOps[i].destNodeID == destNodeID) {
            BEEBUS_DEBUG_OPERATIONS("REMOVED: Op Node:%d FC:%02X Addr:%d",
                destNodeID, _pendingOps[i].req.function, _pendingOps[i].req.startAddr);
            retireAt(i, result);
            break;
        }
    }
}

void BeeBusOperations::removePendingResponse(const BeeBusPendingResponse& sent) {
    removePendingResponse(sent.pduType, sent.destNodeID, sent.response.function,
                          sent.response.startAddr);
}

void BeeBusOperations::removePendingResponse(uint8_t pduType, uint8_t destNodeID,
                                             uint8_t function, uint16_t startAddr) {
    for (size_t i = 0; i < _pendingResponses.size(); i++) {
        const BeeBusPendingResponse& it = _pendingResponses[i];
        if (it.pduType == pduType &&
            it.destNodeID == destNodeID &&
            it.response.function == function &&
            it.response.startAddr == startAddr) {
            _pendingResponses.eraseAt(i);
            BEEBUS_DEBUG_OPERATIONS("REMOVED: Response FC:%02X Addr:%d to Node:%d",
                function, startAddr, destNodeID);
            break;
        }
    }
}

bool BeeBusOperations::isOperationReady(const BeeBusPendingModbusOp& op,
                                       const BeeBusProtocol& protocol) const {
    // A first attempt goes on the next token that has room for it.
    if (op.retryCount == 0) {
        return true;
    }

    // A retry waits a rotation, because a rotation is how long it takes for
    // anything on this network to have had a chance to happen. Expressing the
    // backoff in milliseconds would be either pointless on a slow ring or
    // needlessly slow on a fast one.
    const bb_time_t backoff =
        protocol.ringTimeout(bb_setting<uint8_t>(protocol.config().rotationsRetry, BEEBUS_ROTATIONS_RETRY), protocol.config().retryDelay,
                             BEEBUS_REFERENCE_OPERATION);
    return bb_elapsed(op.timestamp, bb_millis()) >= backoff;
}

bool BeeBusOperations::hasOutstandingRead(uint8_t nodeID, uint8_t functionCode,
                                          uint16_t address) const {
    for (size_t i = 0; i < _pendingOps.size(); i++) {
        const BeeBusPendingModbusOp& op = _pendingOps[i];
        if (op.pduType != BEEBUS_PDU_MODBUS || op.req.isResponse) continue;
        if (op.destNodeID != nodeID) continue;
        if (op.req.function != functionCode || op.req.startAddr != address) continue;
        return true;
    }
    return false;
}

// --- Response matching ---
size_t BeeBusOperations::indexByOpId(uint8_t srcNodeID, uint16_t opId) const {
    for (size_t i = 0; i < _pendingOps.size(); i++) {
        if (_pendingOps[i].destNodeID == srcNodeID && _pendingOps[i].opId == opId) {
            return i;
        }
    }
    return NO_OP;
}

BeeBusPendingModbusOp* BeeBusOperations::findByOpId(uint8_t srcNodeID, uint16_t opId) {
    const size_t index = indexByOpId(srcNodeID, opId);
    return index == NO_OP ? nullptr : &_pendingOps[index];
}

bool BeeBusOperations::completeAcknowledgedWrite(uint8_t srcNodeID, uint16_t opId,
                                                 uint8_t status, BeeBusProtocol& protocol) {
    const size_t index = indexByOpId(srcNodeID, opId);
    if (index == NO_OP) {
        // Nothing outstanding under that ID. Normal when an acknowledgement is
        // duplicated by the medium, or arrives after the operation timed out.
        return false;
    }

    const bool accepted = (status == MB_ACK_OK);

    if (accepted) {
        BEEBUS_DEBUG_OPERATIONS("ACK: Node:%d op:%u confirmed", srcNodeID, (unsigned)opId);
    } else {
        BEEBUS_DEBUG_OPERATIONS("ACK: Node:%d op:%u refused, code:%02X",
            srcNodeID, (unsigned)opId, status);
        protocol.reportError(bb_exceptionToError(status), bb_exceptionName(status));
    }

    // Removed before the handler runs, so a handler that queues the next write
    // does not reallocate the queue under a pointer this still held.
    retireAt(index, accepted ? BeeBusResult::Immediate
                             : bb_exceptionToResult(status));
    return true;
}

size_t BeeBusOperations::indexOfTopicOp(uint8_t srcNodeID, int topicID, int opcode) const {
    for (size_t i = 0; i < _pendingOps.size(); i++) {
        const BeeBusPendingModbusOp& op = _pendingOps[i];
        if (op.pduType != BEEBUS_PDU_TOPIC || op.destNodeID != srcNodeID) continue;
        if (topicID >= 0 && op.req.startAddr != (uint16_t)topicID) continue;
        if (opcode >= 0 && op.req.function != (uint8_t)opcode) continue;
        return i;
    }
    return NO_OP;
}

BeeBusPendingModbusOp* BeeBusOperations::findTopicOp(uint8_t srcNodeID, int topicID, int opcode) {
    const size_t index = indexOfTopicOp(srcNodeID, topicID, opcode);
    return index == NO_OP ? nullptr : &_pendingOps[index];
}

bool BeeBusOperations::completeTopicOp(uint8_t srcNodeID, int topicID, int opcode,
                                       BeeBusResult result) {
    const size_t index = indexOfTopicOp(srcNodeID, topicID, opcode);
    if (index == NO_OP) {
        return false;
    }

    retireAt(index, result);
    return true;
}

bool BeeBusOperations::failPendingRequest(const BeeBusModbusRequest& response, uint8_t srcNodeID) {
    const size_t index = indexOfMatchingRequest(srcNodeID, response);
    if (index == NO_OP) {
        return false;
    }

    // The caller's variable is left alone: an exception says the value was
    // never read, and overwriting it would present a fabricated reading as
    // real. The operation is dropped rather than retried, because the faults
    // it reports are decisions the responder has already made and will make
    // again. The application learns of it through the error handler.
    const uint8_t code = response.data.empty() ? 0 : response.data[0];
    BEEBUS_DEBUG_OPERATIONS("EXCEPTION: Node:%d FC:%02X Addr:%d code:%02X",
        srcNodeID, response.function, response.startAddr, code);

    // Anything waiting on the read is told why, rather than being left to infer
    // a refusal from a variable that never changes. Retired first, so the
    // handler may queue a retry to another node.
    retireAt(index, bb_exceptionToResult(code));
    return true;
}

bool BeeBusOperations::matchAndFulfillResponse(const BeeBusModbusRequest& response, uint8_t srcNodeID) {
    // Find matching pending request
    const size_t index = indexOfMatchingRequest(srcNodeID, response);
    if (index == NO_OP) {
        return false;
    }

    // Write response data directly to user's variable. Done while the record is
    // still in the queue, because nothing here calls application code yet.
    writeResponseToVariable(_pendingOps[index], response);

    BEEBUS_DEBUG_OPERATIONS("FULFILLED: Direct response for Node:%d FC:%02X Addr:%d",
        srcNodeID, response.function, response.startAddr);

    // Retired before the completion handler runs, so a handler that issues the
    // next read in a chain cannot reallocate the queue under this call.
    retireAt(index, BeeBusResult::Immediate);
    return true;
}

size_t BeeBusOperations::indexOfMatchingRequest(uint8_t srcNodeID,
                                                const BeeBusModbusRequest& response) const {
    uint8_t baseFunction = response.function & 0x7F; // Remove error bit

    for (size_t i = 0; i < _pendingOps.size(); i++) {
        const BeeBusPendingModbusOp& op = _pendingOps[i];
        if (op.pduType != BEEBUS_PDU_MODBUS ||
            op.destNodeID != srcNodeID ||
            op.req.function != baseFunction ||
            op.req.startAddr != response.startAddr) {
            continue;
        }

        // An exception says only which function failed and where. There is no
        // quantity to compare, so the function and address identify it.
        if (BeeBusModbusFrame::isErrorResponse(response.function)) {
            return i;
        }

        // A bit response carries a byte count, not a bit count, so the exact
        // quantity cannot be recovered from it: a 1 coil read and an 8 coil
        // read both return one byte. Match on the byte count the request would
        // have produced instead of demanding an exact quantity match.
        if (bb_isBitReadFunction(baseFunction)) {
            const uint16_t expectedBytes = (uint16_t)((op.req.quantity + 7) / 8);
            if (!response.data.empty() && response.data[0] == expectedBytes) {
                return i;
            }
            continue;
        }

        if (op.req.quantity == response.quantity) {
            return i;
        }
    }

    return NO_OP;
}

BeeBusPendingModbusOp* BeeBusOperations::findMatchingRequest(uint8_t srcNodeID,
                                                       const BeeBusModbusRequest& response) {
    const size_t index = indexOfMatchingRequest(srcNodeID, response);
    return index == NO_OP ? nullptr : &_pendingOps[index];
}

void BeeBusOperations::writeResponseToVariable(const BeeBusPendingModbusOp& op, const BeeBusModbusRequest& response) {
    if (!op.resultPtr) {
        return; // No variable to write to
    }

    // A string arrives as ordinary registers, so the function code cannot say
    // that the buffer behind resultPtr is characters. The operation says it.
    if (op.resultKind == BeeBusResultKind::Text) {
        extractStringData(response, static_cast<char*>(op.resultPtr), op.resultCapacity);
        return;
    }

    uint8_t baseFunction = response.function & 0x7F;

    switch (baseFunction) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS: {
            if (op.isArray) {
                extractCoilData(response, op.resultPtr, op.arraySize, op.resultStride);
            } else if (response.data.size() >= 2) {
                uint8_t byteData = response.data[1]; // Skip byte count
                *static_cast<bool*>(op.resultPtr) = (byteData & 0x01) != 0;
            }
            break;
        }

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            if (op.isArray) {
                extractRegisterData(response, op.resultPtr, op.arraySize, op.resultStride);
            } else if (response.data.size() >= 3) {
                *static_cast<int16_t*>(op.resultPtr) = (int16_t)bb_unpackBE16(&response.data[1]);
            }
            break;
        }
    }
}

// --- Data extraction utilities ---
bool BeeBusOperations::extractCoilData(const BeeBusModbusRequest& response, void* values,
                                       uint16_t maxValues, uint16_t stride) {
    if (response.data.size() < 1 || !values) {
        return false;
    }

    uint8_t byteCount = response.data[0];
    if (response.data.size() < (size_t)(1 + byteCount)) {
        return false;
    }

    uint16_t bitIndex = 0;
    for (uint8_t byteIdx = 0; byteIdx < byteCount && bitIndex < maxValues; byteIdx++) {
        uint8_t byteData = response.data[1 + byteIdx];

        for (uint8_t bitPos = 0; bitPos < 8 && bitIndex < maxValues; bitPos++) {
            *bb_elemAt<bool>(values, bitIndex, stride) = (byteData & (1 << bitPos)) != 0;
            bitIndex++;
        }
    }

    return true;
}

bool BeeBusOperations::extractStringData(const BeeBusModbusRequest& response, char* text,
                                         uint16_t maxBytes) {
    if (response.data.empty() || !text || maxBytes == 0) {
        return false;
    }

    // Against the byte count, not against a register count derived from it. An
    // odd byte count of 2 * registers + 1 divides down to exactly the register
    // bound and then copies one byte past the end of the caller's buffer. No
    // conforming node sends an odd count on a register read; a foreign
    // implementation, a gateway or a corrupt frame that still passes CRC can.
    const uint8_t byteCount = response.data[0];
    if (response.data.size() < (size_t)(1 + byteCount) || byteCount > maxBytes) {
        return false;
    }

    if (byteCount == 0) {
        text[0] = '\0';
        return true;
    }

    memcpy(text, &response.data[1], byteCount);

    // The wire carries a fixed field rather than a terminated string, so a
    // sender that filled the field completely leaves nothing to stop on. The
    // last character is given up rather than handing back a buffer that runs on.
    if (memchr(text, '\0', byteCount) == nullptr) {
        text[byteCount - 1] = '\0';
    }
    return true;
}

bool BeeBusOperations::extractRegisterData(const BeeBusModbusRequest& response, void* values,
                                           uint16_t maxValues, uint16_t stride) {
    if (response.data.size() < 1 || !values) {
        return false;
    }

    uint8_t byteCount = response.data[0];
    uint16_t regCount = byteCount / 2;

    if (response.data.size() < (size_t)(1 + byteCount) || regCount > maxValues) {
        return false;
    }

    for (uint16_t regIdx = 0; regIdx < regCount; regIdx++) {
        *bb_elemAt<int16_t>(values, regIdx, stride) =
            (int16_t)bb_unpackBE16(&response.data[1 + (regIdx * 2)]);
    }

    return true;
}
