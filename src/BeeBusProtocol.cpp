#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// --- Lifecycle ---
BeeBusProtocol::BeeBusProtocol()
    : _nodeID(0),
      _state(BeeBusState::Offline),
      _ringMemberCount(0),
      _errorHandler(nullptr),
      _io(nullptr),
      _rotationsSinceJoin(0),
      _lastTokenSeen(0),
      _lastServiceAt(0),
      _lastServiceUs(0),
      _serviceUs(0),
      _hopStart(0),
      _hopSuccessor(0),
      _hopWindowMs(0),
      // Both start valid: noteTokenAcquired already skips the first
      // acquisition, so starting false would discard the second one too.
      _rotationSampleValid(true),
      _hopSampleValid(false),
      _observedSampleValid(true),
      _lastHeldToken(0),
      _rotationsSinceSuccessorVisit(0),
      _tokenReceivedForUs(false),
      _tokenConfirmed(false),
      _confirmedBy(0),
      _pendingSuccessor(0),
      _tokenRetryCount(0),
      _passesWithoutTurn(0),
      _turnParity(false),
      _turnHistory{},
      _turnHistoryAt{},
      _turnHistoryNext(0),
      _passFloorNode(0),
      _passFloor(0),
      _skippedSuccessor(0),
      _skippedSuccessorCount(0),
      _retryingPass(false),
      _freshTurn(true),
      _joinWaitStart(0),
      _solicitedUs(false),
      _joinAnswerAt(0),
      _answeredRound(0),
      _initialListenTime(0),
      _initialListenSet(false),
      _solicitInterval(1),
      _rotationsSinceSolicit(0),
      _solicitDecision(0),
      _solicitRound(0),
      _solicitSpread(1),
      _announceHeard(false),
      _solicitPending(false),
      _corruptionSinceSolicit(false),
      _announceAttempts(0),
      _announceSkip(0),
      _announceIdleFrames(0),
      _solicitWindowEnds(0),
      _solicitWindowOpen(false),
      _ownWindowEnds(0),
      _windowNoiseBytes(0),
      _windowLineFault(false),
      _statedWindowMs(0),
      _passCarriedSolicit(false),
      _loneToken(false),
      _lastFrameReceived(0),
      _lastRingActivity(0),
      _hasReceivedFrame(false),
      _lastMemberTimeoutCheck(0),
      _stateEntryTime(0),
      _duplicateNodeID(false),
      _duplicateSeenAt(0),
      _connectRequested(false)
{
    for (size_t i = 0; i < sizeof(_ringMembers); i++) {
        _ringMembers[i] = 0;
    }
    for (size_t i = 0; i < 256; i++) {
        _lastHeardFrom[i] = 0;
    }
}

BeeBusProtocol::~BeeBusProtocol() {
    delete _io;
    _io = nullptr;
    for (size_t i = 0; i < _bindings.size(); i++) {
        delete[] _bindings[i].shadow;
        delete[] _bindings[i].syncedText;
    }
}

void BeeBusProtocol::reportError(BeeBusError error, const char* msg) {
    if (_errorHandler) {
        _errorHandler(error, msg);
    }
}

namespace {
// How deep the queues are for `config`: its own figures, or the platform's.
void bb_queueSizes(const BeeBusConfig& config, uint8_t& ops, uint8_t& responses) {
#if defined(BEEBUS_PLATFORM_ARDUINO)
    const uint8_t defaultQueue = 16;
#else
    const uint8_t defaultQueue = BEEBUS_MAX_PENDING_OPS;
#endif
    ops = config.maxPendingOps ? config.maxPendingOps : defaultQueue;
    responses = config.maxPendingResponses ? config.maxPendingResponses : defaultQueue;
}
}  // namespace

bool BeeBusProtocol::prepare(const BeeBusConfig& config) {
    if (!_io) _io = new (std::nothrow) BeeBusIO(*this);
    if (!_io) return false;
    uint8_t ops = 0, responses = 0;
    bb_queueSizes(config, ops, responses);
    return _operations.begin(ops, responses);
}

bool BeeBusProtocol::begin(uint8_t nodeID, BeeBusTransport* transport, const BeeBusConfig& config) {
    _nodeID = nodeID;
    _config = config;

    // A fresh claim to an address, so whatever was heard under the last one
    // says nothing about this one.
    _duplicateNodeID = false;
    _duplicateSeenAt = 0;

    // A node is always a member of its own ring.
    _ringMembers[0] = _nodeID;
    _ringMemberCount = 1;

    const bb_time_t now = bb_millis();
    noteHeardFrom(_nodeID);
    _lastTokenSeen = now;
    _lastHeldToken = now;

    if (!_io) {
        _io = new (std::nothrow) BeeBusIO(*this);
    }
    if (!_io) {
        reportError(BeeBusError::Unknown, "Out of memory for the frame I/O");
        return false;
    }

    if (!_io->begin(transport)) {
        reportError(BeeBusError::Unknown, "Failed to initialize IO");
        return false;
    }

    _operations.clearPendingOperations();
    _operations.clearPendingResponses();

    // The queues' storage, taken once, now that the configuration is known
    // (already, when prepare() ran with the same configuration). A queue with
    // no storage would refuse every operation for ever, so it is a failure to
    // start - said now, never discovered as QueueFull later.
    uint8_t ops = 0, responses = 0;
    bb_queueSizes(_config, ops, responses);
    if (!_operations.begin(ops, responses)) {
        reportError(BeeBusError::Unknown, "Out of memory for the operation queues");
        return false;
    }

    // This instance's session: what tells peers it is not the one before it,
    // however quickly it came back.
    _session = _config.sessionId;
    while (_session == 0) _session = (uint16_t)(bb_entropy() >> 8);
    _sessionTurns = BEEBUS_SESSION_TURNS;

    // Taken once, so a write held for a read-back never allocates; the peer
    // table grows as peers appear.
    if (!_results.reserve(BEEBUS_MAX_PENDING_RESULTS) || !_peers.reserve(4) ||
        !_resultRecords.reserve(BEEBUS_MAX_RESULT_RECORDS) ||
        !_sentResults.reserve(BEEBUS_MAX_SENT_RESULTS)) {
        reportError(BeeBusError::Unknown, "Out of memory for the reliable topic tables");
        return false;
    }

    reportError(BeeBusError::StateChange, "Protocol started");
    return true;
}

void BeeBusProtocol::adoptApplication(BeeBusProtocol& from) {
    // Moved, not copied: the old protocol is deleted straight afterwards, and
    // a copy of every map would cost flash for each tree type as well as time.
    _config = from._config;
    _dataMap = std::move(from._dataMap);
    _topicMap = std::move(from._topicMap);
    _deferredSubs = std::move(from._deferredSubs);
    _served = std::move(from._served);
    _bindings = std::move(from._bindings);
    _topicResultHandler = std::move(from._topicResultHandler);
    _topicDecidedHandler = std::move(from._topicDecidedHandler);
    _bindStateHandler = std::move(from._bindStateHandler);

    // A bind starts again on the new ring: the owner's value is asked for
    // again, and a change that was with the old ring is not with this one.
    for (size_t i = 0; i < _bindings.size(); i++) {
        _bindings[i].flags = 0;
        _bindings[i].state = BeeBusBindState::Offline;
    }

    _errorHandler = from._errorHandler;
    _memberJoinedHandler = std::move(from._memberJoinedHandler);
    _memberLeftHandler = std::move(from._memberLeftHandler);
    _topicChangedHandler = std::move(from._topicChangedHandler);
    _topicWrittenHandler = std::move(from._topicWrittenHandler);
    _topicStringChangedHandler = std::move(from._topicStringChangedHandler);
    _topicStringWrittenHandler = std::move(from._topicStringWrittenHandler);
    _topicGapHandler = std::move(from._topicGapHandler);
    _topicCompleteHandler = std::move(from._topicCompleteHandler);
    _topicListHandler = std::move(from._topicListHandler);

    // The old ring's knowledge, both ways: our subscriptions there are not
    // standing on this ring yet, and its subscribers to our topics are gone.
    // The standing intents (each remote topic's terms) stay, so
    // resubscribeTo() asks again as each peer joins this ring.
    for (unsigned node = 1; node <= 255; node++) _topicMap.removeNode((uint8_t)node);
    for (size_t i = 0; i < _topicMap.remoteCount(); i++) {
        const BeeBusRemoteTopic& topic = _topicMap.remoteAt(i);
        if (topic.contractId == BEEBUS_SUB_NONE) continue;
        const uint8_t node = (uint8_t)(topic.key >> 16);
        const uint16_t id = (uint16_t)(topic.key & 0xFFFF);
        deferSubscribe(node, id);
    }
}

// --- Main loop ---
void BeeBusProtocol::loop() {
    if (!_io) {
        return;
    }

    // The debug instance is process-wide, so say whose output this is.
    BEEBUS_DEBUG_NODE(_nodeID);

    if (_duplicateNodeID) {
        // Stood down. Keep draining the receive path so the transport does not
        // back up, but take no part and claim no address. Draining is also what
        // keeps hearing the duplicate, so the address is known to be in use for
        // as long as it is.
        _io->processIncoming();

        // The mistake is usually fixed by taking the other device off the
        // network, and nothing else would then bring this node back.
        if (_connectRequested &&
            bb_elapsed(_duplicateSeenAt, bb_millis()) > duplicateHoldoff()) {
            resumeAfterDuplicate();
        }
        return;
    }

    bb_time_t now = bb_millis();

    // How long this node went between looks at the bus. A timeout shorter than
    // that interval cannot be enforced however fast the network is, so this is
    // what floors every derived timeout.
    if (_lastServiceAt != 0) {
        _serviceTiming.addSample(bb_elapsed(_lastServiceAt, now));
    }
    _lastServiceAt = now;

    // And in microseconds, for the interframe gap: a floor rounded to the
    // nearest millisecond would be the whole of it on a fast line. Smoothed
    // rather than shortest, since the floor needs the interval this node can be
    // relied on to manage.
    const bb_time_t nowUs = bb_micros();
    if (_lastServiceUs != 0) {
        const bb_time_t delta = bb_elapsed(_lastServiceUs, nowUs);
        if (delta < 1000000u) {   // A stall is not a service interval
            _serviceUs = _serviceUs == 0 ? delta : (_serviceUs * 3 + delta) / 4;
        }
    }
    _lastServiceUs = nowUs;

    // Counted before anything is received, so a write applied in this loop()
    // is read back in a later one, after the program has run.
    _loopCount++;

    // Incoming data first: everything below decides on what it stamped.
    _io->processIncoming();

    // The gap a frame has just left behind is the one moment a node nobody has
    // invited can speak without talking over anybody. Taken here rather than
    // inside the receive path, and before the clock is read again below,
    // because saying so means transmitting.
    if (_io->takeFrameCompleted()) {
        announceIfDue();
    }

    // What was owed for an address on this node, served in place since the last
    // loop. Before the queued work, so a handler that queues something is
    // serviced on this pass rather than waiting for the next.
    _operations.dispatchLocalCompletions();

    _operations.processPendingOperations(*this);
    _operations.cleanupTimedOutOperations(*this);

    // Whatever a subscriber is owed, from a bounded slice of the topic map.
    serviceTopics();

    // Writes waiting to be read back, then the binds: a change of a bound
    // variable goes out, the owner's value comes in.
    serviceResults();
    serviceBindings();

    // Everything from here on is measured against stamps the receive path and
    // the handlers above have just taken, so the clock is read again. Against
    // the reading on the way in, a stamp taken since lies in the future, and
    // the unsigned subtraction wraps its age to nearly fifty days: every
    // timeout at once, including the quiet that licenses regenerating a token.
    now = bb_millis();

    // Membership and isolation are only meaningful once this node is in a ring.
    if (_state == BeeBusState::Member || _state == BeeBusState::HoldingToken ||
        _state == BeeBusState::PassingToken) {
        if (bb_elapsed(_lastMemberTimeoutCheck, now) >= getMembershipCheckInterval()) {
            checkMemberTimeouts();
            _lastMemberTimeoutCheck = now;
        }

        // The token has gone round without arriving here. A member is handed it
        // once per rotation, so this is proof the ring is running without us.
        //
        // Checked in every ring state, not just Member: a node evicted while
        // stalling on a pass stays in PassingToken, retrying to a successor
        // that will never confirm, and cannot be solicited back in because a
        // node that believes it is a member does not answer solicitations.
        if (_ringMemberCount > 1 &&
            _passesWithoutTurn >
                (uint16_t)bb_setting<uint8_t>(_config.passesWithoutTurn, BEEBUS_PASSES_WITHOUT_TURN) * _ringMemberCount) {
            BEEBUS_DEBUG_PROTOCOL("TOKEN: %u passes heard and none for us, rejoining",
                (unsigned)_passesWithoutTurn);
            _passesWithoutTurn = 0;
            dropAllPeers();
            resetJoiningState();
            transitionToState(BeeBusState::Joining);
            return;
        }

        // A node that can still transmit but no longer receives must stop
        // acting as a ring member, or peers keep handing it a token it can
        // never accept. Withdrawing and rejoining is the clean recovery.
        if (_hasReceivedFrame &&
            bb_elapsed(_lastFrameReceived, now) > getIsolationTimeout()) {
            handleBusIsolation();
            return;
        }
    }

    switch (_state) {
        case BeeBusState::Listening: {
            if (_io->_rxAvailable) {
                BEEBUS_DEBUG_PROTOCOL("LISTENING: traffic heard, a ring already exists");
                resetJoiningState();
                transitionToState(BeeBusState::Joining);
                break;
            }

            if (bb_elapsed(_stateEntryTime, now) >= getInitialListenTime()) {
                // Silence. A ring of one is its own successor, so its gap is
                // the whole address space and its first token pass is also the
                // solicitation that finds everybody else.
                BEEBUS_DEBUG_PROTOCOL("LISTENING: silence, Node %d starts a ring of its own",
                    _nodeID);
                resetSolicitPacing();
                transitionToState(BeeBusState::HoldingToken);
            }
            break;
        }

        case BeeBusState::Joining: {
            // Being handed the token is the proof that a join request landed.
            // Until then this node is still a candidate, free to answer the
            // next solicitation if its request was lost or collided.
            if (_tokenReceivedForUs) {
                BEEBUS_DEBUG_PROTOCOL("JOINING: handed the token, we are in");
                resetJoiningState();
                transitionToState(BeeBusState::Member);
                break;
            }

            if (_solicitedUs) {
                transitionToState(BeeBusState::Requesting);
                break;
            }

            if (hasJoinWaitTimedOut()) {
                BEEBUS_DEBUG_PROTOCOL("JOINING: nothing solicited us, listening again");
                transitionToState(BeeBusState::Listening);
            }
            break;
        }

        case BeeBusState::Requesting: {
            // Answer in our own slot of the contention window. Answering early
            // is what causes the collision the slotting exists to resolve.
            if (bb_isFuture(now, _joinAnswerAt)) {
                break;
            }

            if (_io->sendJoinRequestFrame(_nodeID)) {
                // Sent, not necessarily heard: another node in the same slot
                // destroys both requests and neither sender can tell. Go back
                // to waiting so the next solicitation is answered.
                BEEBUS_DEBUG_PROTOCOL("REQUESTING: join request sent");
                resetJoiningState();
                transitionToState(BeeBusState::Joining);
            }
            break;
        }

        case BeeBusState::Leaving: {
            if (_tokenReceivedForUs) {
                if (_io->sendDepartureFrame(_nodeID)) {
                    BEEBUS_DEBUG_PROTOCOL("LEAVING: departure announced");
                    transitionToState(BeeBusState::Offline);
                    break;
                }
            }

            // The departure waits for a turn that only the rest of the ring can
            // give. One that has not come in as long as a member waits before
            // concluding it has been dropped is not coming: the ring has gone,
            // or has already let this node go, and either way there is nobody
            // left to tell.
            if (bb_elapsed(_stateEntryTime, now) > getRejoinTimeout()) {
                BEEBUS_DEBUG_PROTOCOL("LEAVING: no turn came, going offline unannounced");
                transitionToState(BeeBusState::Offline);
            }
            break;
        }

        case BeeBusState::Member: {
            if (_loneToken) {
                // We handed the token to ourselves, so nothing will hand it
                // back. Wait out the solicitation window and take it up again,
                // paced by the join cycle while still alone so an empty bus is
                // not asked continuously.
                if (_solicitWindowOpen) {
                    if (bb_isFuture(now, _solicitWindowEnds)) break;
                    _solicitWindowOpen = false;
                }

                // Alone, so nothing paces this but itself. The window just held
                // is how long an answer takes on this medium.
                bb_time_t pace = 0;
                if (_ringMemberCount <= 1) {
                    pace = _statedWindowMs != 0 ? _statedWindowMs
                                                : bb_setting<bb_time_t>(_config.joinCycleInterval, BEEBUS_JOIN_CYCLE_MS);
                }
                if (bb_elapsed(_stateEntryTime, now) < pace) {
                    break;
                }

                _loneToken = false;
                transitionToState(BeeBusState::HoldingToken);
                break;
            }

            if (_tokenReceivedForUs) {
                // A solicitation rode in on the token we were handed. Whoever
                // is in that gap has to be given its slot before we transmit,
                // or we talk over the answer we asked for.
                if (_solicitWindowOpen) {
                    if (bb_isFuture(now, _solicitWindowEnds)) break;
                    _solicitWindowOpen = false;
                }

                transitionToState(BeeBusState::HoldingToken);
                break;
            }

            // Token reclaim. Only the lowest-numbered member regenerates a lost
            // token, so two members never do it at once. Every other member
            // falls back on rejoining (below) if no turn comes at all, which
            // also covers the lowest member having gone.
            const bb_time_t reclaimDue = getReclaimTimeout();

            if (bb_elapsed(_stateEntryTime, now) > reclaimDue) {
                // A token is lost when our ring goes quiet with it, not merely
                // when this node has waited a while for a turn. A member
                // retrying a pass transmits the whole time it does so, and
                // regenerating underneath it puts two tokens on the bus.
                //
                // Only our ring's token traffic counts. A node that
                // has dropped out runs a ring of one, handing itself the token
                // for as long as it likes: counted as traffic, it would keep
                // the member whose job it is from ever reclaiming, and the ring
                // would stand with no token until the rejoin timeout. A foreign
                // ring's traffic is settled by the merge rule once ours runs.
                const bool busQuiet =
                    _lastRingActivity == 0 ||
                    bb_elapsed(_lastRingActivity, now) > getReclaimTimeout();

                if (isLowestMemberID() && busQuiet) {
                    discardTimingSample();
                    transitionToState(BeeBusState::HoldingToken);
                    break;
                }

                // Not our turn to reclaim. A token that is merely late is
                // normal while the ring heals, so only give up after several
                // rotations with none at all; a bus we cannot hear is handled
                // by isolation detection.
                if (bb_elapsed(_lastHeldToken, now) > getRejoinTimeout()) {
                    BEEBUS_DEBUG_PROTOCOL("TOKEN: no turn for %lu ms, rejoining",
                        (unsigned long)bb_elapsed(_lastHeldToken, now));
                    // Give the old membership up with it, or this node goes on
                    // naming a peer as its successor and the ring reads that
                    // as a chain edge and takes it back in unjoined.
                    dropAllPeers();
                    resetJoiningState();
                    transitionToState(BeeBusState::Joining);
                } else {
                    _stateEntryTime = now;   // Keep waiting for our turn
                }
            }
            break;
        }

        case BeeBusState::HoldingToken: {
            // Before the work is counted, so anything released from a
            // store-and-forward ring rides this turn's frame rather than
            // waiting a whole rotation for the next.
            drainTopicBuffers();

            const bool haveWork = _operations.getPendingResponseCount() > 0 ||
                                  _operations.getPendingOpCount() > 0;

            // The token goes to the next member by ID, except that a successor
            // whose link costs many times the rest of the ring is given it less
            // often, so one radio hop does not pace a whole wired segment.
            uint8_t nextNodeID = successor();
            const uint8_t visitInterval = successorVisitInterval();
            if (visitInterval > 1) {
                if (_rotationsSinceSuccessorVisit + 1 < visitInterval) {
                    const uint8_t skipTo = memberAfter(nextNodeID);
                    if (skipTo != _nodeID && skipTo != nextNodeID) {
                        _rotationsSinceSuccessorVisit++;
                        nextNodeID = skipTo;

                        // A lap that left a member out is not a rotation, and
                        // folding it in is self-reinforcing: the estimate
                        // shrinks, the slow hop looks like a larger share, and
                        // the node is skipped harder still.
                        discardTimingSample();
                    } else {
                        _rotationsSinceSuccessorVisit = 0;
                    }
                } else {
                    _rotationsSinceSuccessorVisit = 0;
                }
            } else {
                _rotationsSinceSuccessorVisit = 0;
            }

            // A fresh turn moves the turn bit on; a retransmission carries the
            // same one. Committed after the send, not before: entering this
            // state and leaving without transmitting has not used a turn up,
            // and moving the bit anyway would have the successor read the next
            // pass as a retransmission.
            const bool freshTurn = _freshTurn;
            const bool turnParity = freshTurn ? !_turnParity : _turnParity;

            BeeBusHeader header;
            header.srcNodeID = _nodeID;
            header.successorID = nextNodeID;
            if (turnParity) header.flags |= BEEBUS_HDR_TURN;
            if (!freshTurn) header.flags |= BEEBUS_HDR_REPEAT;

            // An adjacent successor leaves no address to hold a node, so there
            // is nobody to invite and no window worth waiting out. Only a fresh
            // turn solicits: a retry is the same turn again, and letting it
            // open another response window is self-sustaining.
            // Once per turn, not once per attempt. A node polls loop() far
            // faster than the interframe gap, so it reaches here many times
            // over before the bus is free, and asking the pacing each time
            // would spend a rotation's worth of budget on every poll.
            if (_solicitDecision == 0) {
                _solicitDecision = (freshTurn && !bb_gapIsEmpty(_nodeID, nextNodeID) &&
                                    shouldSolicitThisRotation())
                                       ? 2 : 1;
            }
            const bool soliciting = _solicitDecision == 2;

            // Worked out once and stated on the wire, so the gap answers
            // against the same window this node is about to hold.
            const bb_time_t window = soliciting ? solicitWindowMs() : 0;
            if (soliciting) {
                header.solicit(_solicitRound, bb_encodeWindow(window));
            }

            // A ring of one has nobody to hand the token to, and on a slow link
            // a frame to itself is in the air long enough to talk over the
            // answer its solicitation asked for. So it speaks only when it has
            // something to say.
            if (nextNodeID == _nodeID && !soliciting && !haveWork) {
                _solicitDecision = 0;
                _loneToken = true;
                transitionToState(BeeBusState::Member);
                break;
            }

            const bool tokenSent = haveWork ? _io->sendDataFrame(header)
                                            : _io->sendTokenFrame(header);
            if (!tokenSent) {
                break;
            }

            // On the wire, so the next turn decides afresh.
            _solicitDecision = 0;

            _turnParity = turnParity;
            _freshTurn = false;

            // Our own frames never come back to us, so the hop we have just
            // opened has to be stamped here rather than in observeFrame.
            noteHopStart(nextNodeID, soliciting ? window : 0);

            if (soliciting) {
                // The stated window and not the computed one: what the gap was
                // told is what it will answer against, and the encoding rounds.
                _statedWindowMs = bb_decodeWindow(bb_encodeWindow(window));
                _solicitWindowEnds = bb_millis() + _statedWindowMs;
                _solicitWindowOpen = true;

                // Our own round, which is the only one corruption says
                // anything about this node's gap in. Stamped before
                // noteSolicitSent(), which clears the evidence for it.
                _ownWindowEnds = _solicitWindowEnds;
                noteSolicitSent();
            }
            _passCarriedSolicit = soliciting;

            _pendingSuccessor = nextNodeID;

            // Karn's rule: only a pass that went out first time can be timed,
            // since there is no telling which copy an answer belongs to.
            _hopSampleValid = (_tokenRetryCount == 0);

            if (nextNodeID == _nodeID) {
                // A ring of one hands the token to itself, so nothing will ever
                // confirm the pass.
                _loneToken = true;
                transitionToState(BeeBusState::Member);
            } else {
                transitionToState(BeeBusState::PassingToken);
            }
            break;
        }

        case BeeBusState::PassingToken: {
            if (_ringMemberCount <= 1) {
                // Everyone else has gone. Keep the token and keep soliciting:
                // a ring of one is a working state, not a failure.
                transitionToState(BeeBusState::HoldingToken);
                break;
            }

            if (_tokenConfirmed) {
                // The dwell here is how long the successor took to pick the
                // token up: airtime out, its turnaround, airtime back. That is
                // the quantity a per-peer timeout is built from.
                //
                // A pass carrying a solicitation spent most of its dwell in the
                // response window, which is no property of the link. Subtract
                // it rather than discarding the sample, or the one node that
                // owns a gap worth soliciting is never timed.
                bb_time_t dwell = bb_elapsed(_stateEntryTime, now);
                if (_passCarriedSolicit) {
                    const bb_time_t window = _statedWindowMs;
                    dwell = dwell > window ? dwell - window : 0;
                }
                // Only when the successor itself is what we heard. A frame from
                // elsewhere in the ring proves the token is moving, which is
                // enough to stop waiting, but timing this link by it measures
                // whatever that other node was doing.
                const bool measured = _hopSampleValid &&
                                      _confirmedBy == _pendingSuccessor;
                if (measured) {
                    _hopTiming.addSample(_pendingSuccessor, dwell);
                }

                // The pass landed, so whatever made us go round this peer is
                // over.
                if (_skippedSuccessor == _pendingSuccessor) {
                    _skippedSuccessor = 0;
                    _skippedSuccessorCount = 0;
                }

                // A measurement retires the backoff outright (RFC 6298): it
                // exists to find an unknown link and has nothing left to say
                // once that link is timed. Without one it only decays, since it
                // is what let this pass through and the optimistic guess would
                // time the next one out again.
                if (measured && _passFloorNode == _pendingSuccessor) {
                    _passFloor = 0;
                    _passFloorNode = 0;
                } else {
                    _passFloor -= _passFloor / 4;
                }
                _passCarriedSolicit = false;
                _tokenConfirmed = false;
                _confirmedBy = 0;
                transitionToState(BeeBusState::Member);
                break;
            }

            // A pass carrying a solicitation has to allow for the successor
            // waiting out the response window before it answers.
            bb_time_t allowance = getTokenPassTimeout(_pendingSuccessor);
            if (_passCarriedSolicit) {
                allowance += _statedWindowMs;
            }

            // Jitter the retry. Two nodes backing off on identical timers
            // collide again on every attempt, each destroying the other's frame
            // and neither hearing the answer that would end it.
            allowance += bb_spread((uint32_t)_nodeID * 977u + _tokenRetryCount +
                                       (uint32_t)_passFloor,
                                   allowance * bb_setting<uint8_t>(_config.retrySpreadPercent, BEEBUS_RETRY_SPREAD_PERCENT) / 100);

            if (bb_elapsed(_stateEntryTime, now) <= allowance) {
                break;
            }

            // Back the timer off for this peer. A link whose real round trip is
            // a second is never found by waiting a tenth of one repeatedly:
            // each attempt has to allow more than the last until one lands.
            if (_passFloorNode != _pendingSuccessor) {
                _passFloorNode = _pendingSuccessor;
                _passFloor = allowance;
            }
            // Doubling, but never past the longest this node may wait for a
            // pass. The floor decays only a quarter at a time, so a runaway one
            // would outlive the conditions that caused it by hundreds of
            // rotations.
            const bb_time_t floorCeiling = passAllowanceCeiling();
            _passFloor = _passFloor < floorCeiling / 2
                             ? (bb_time_t)(_passFloor * 2)
                             : floorCeiling;

            _passCarriedSolicit = false;
            _observedSampleValid = false;
            _tokenRetryCount++;

            // Whatever happens from here, this rotation is a recovery and not
            // a measurement.
            discardTimingSample();

            if (_tokenRetryCount < bb_retryLimit(_config.maxRetries)) {
                BEEBUS_DEBUG_PROTOCOL("TOKEN: retry to Node %d (attempt %d)",
                    _pendingSuccessor, _tokenRetryCount);
                _retryingPass = true;
                transitionToState(BeeBusState::HoldingToken);
                break;
            }

            // A peer we can still hear is not gone, and neither is one that has
            // never been timed: its allowance was a guess, and on a link where
            // one hop costs most of a second it expires while the answer is
            // still in the air. Keep retransmitting and let silence, not
            // retries, decide membership.
            if (isMemberResponsive(_pendingSuccessor) ||
                !hasHopSample(_pendingSuccessor)) {
                _tokenRetryCount = 0;
                BEEBUS_DEBUG_PROTOCOL("TOKEN: Node %d missed the token but is still heard from",
                    _pendingSuccessor);
                _retryingPass = true;
                transitionToState(BeeBusState::HoldingToken);
                break;
            }

            // Silent and out of retries. One rotation of failed passes is thin
            // evidence on a link that loses frames, and removing a member costs
            // the ring an announcement, a solicitation cycle and a join. So the
            // first time the ring goes round the node instead: it stays a
            // member and is asked again next rotation. Only a peer that fails
            // twice running is removed.
            {
                const uint8_t skipTo = memberAfter(_pendingSuccessor);
                const bool firstFailure = _skippedSuccessor != _pendingSuccessor;

                if (firstFailure && skipTo != _nodeID && skipTo != _pendingSuccessor) {
                    _skippedSuccessor = _pendingSuccessor;
                    _skippedSuccessorCount = 1;

                    BeeBusHeader header;
                    header.srcNodeID = _nodeID;
                    header.successorID = skipTo;
                    if (_turnParity) header.flags |= BEEBUS_HDR_TURN;
                    header.flags |= BEEBUS_HDR_REPEAT;

                    if (_io->sendTokenFrame(header)) {
                        BEEBUS_DEBUG_PROTOCOL("TOKEN: Node %d silent, going round it to Node %d",
                            _pendingSuccessor, skipTo);

                        // A lap that went round a member is a recovery, not a
                        // rotation.
                        discardTimingSample();
                        _pendingSuccessor = skipTo;
                        _tokenRetryCount = 0;
                        _stateEntryTime = bb_millis();
                        noteHopStart(skipTo);
                    }
                    break;
                }
            }

            // Silent as well as unresponsive, twice running, so it really has
            // gone.
            {
                const uint8_t lost = _pendingSuccessor;
                BEEBUS_DEBUG_PROTOCOL("TOKEN: Node %d not responding, removing it", lost);
                handleMemberLeft(lost, _nodeID);

                // The pass sequence is over however it ends, so the retry count
                // goes with it: left standing it suppresses the next hop
                // measurement.
                _tokenRetryCount = 0;
                _skippedSuccessor = 0;
                _skippedSuccessorCount = 0;

                const uint8_t replacement = successor();
                if (replacement == _nodeID) {
                    transitionToState(BeeBusState::HoldingToken);
                    break;
                }

                BeeBusHeader header;
                header.srcNodeID = _nodeID;
                header.successorID = replacement;
                if (_turnParity) header.flags |= BEEBUS_HDR_TURN;
                header.left(lost);

                if (_io->sendTokenFrame(header)) {
                    BEEBUS_DEBUG_PROTOCOL("TOKEN: removed Node %d, passed to Node %d",
                        lost, replacement);
                    _pendingSuccessor = replacement;
                    _tokenRetryCount = 0;
                    _stateEntryTime = bb_millis();
                }
                // A failed send stays in PassingToken and is retried next loop.
            }
            break;
        }

        case BeeBusState::Offline:
            break;
    }
}

// --- State machine ---
void BeeBusProtocol::transitionToState(BeeBusState newState) {
    if (_state == newState) {
        return;
    }

    BEEBUS_DEBUG_PROTOCOL("STATE: %s -> %s", bb_stateName(_state), bb_stateName(newState));

    _state = newState;
    _stateEntryTime = bb_millis();

    switch (newState) {
        case BeeBusState::Listening:
            // Not in any ring, so last ring's membership goes: otherwise this
            // node names one of those peers as its successor and the real ring
            // reads that as a chain edge and takes it back in unjoined.
            dropAllPeers();
            _initialListenSet = false;
            _initialListenTime = 0;
            _hasReceivedFrame = false;   // Re-arm isolation detection

            // No hop of ours is outstanding once we stop taking part.
            _hopSuccessor = 0;
            break;

        case BeeBusState::Joining:
            _solicitedUs = false;
            _announceAttempts = 0;
            _announceIdleFrames = 0;

            // Backed off before the first announcement, not only between
            // retries. Nodes powered up together reach this state on the same
            // frame, and announcing on the one after it puts every one of them
            // in the same gap, where they destroy each other and none is
            // heard. Drawn from the address, so they pick different gaps
            // before colliding rather than after.
            _announceSkip = (uint8_t)bb_spread((uint32_t)_nodeID * 977u,
                                               bb_setting<uint8_t>(_config.announceSpreadFrames, BEEBUS_ANNOUNCE_SPREAD_FRAMES));
            break;

        case BeeBusState::Member:
            // In the ring, so the round it answered to get here is spent.
            _answeredRound = 0;
            break;

        case BeeBusState::HoldingToken:
            _lastHeldToken = _stateEntryTime;
            _loneToken = false;

            // Taking the token consumes it, by whichever route we arrived. A
            // flag that never falls never shows the false-to-true edge that
            // times a rotation.
            _tokenReceivedForUs = false;

            // Retransmitting a pass is the same turn again, not a new one.
            // The turn bit says so on the wire, so a successor can tell a
            // retransmitted token from the next one.
            _freshTurn = !_retryingPass;
            _retryingPass = false;
            break;

        default:
            break;
    }
}

// --- Callbacks ---
void BeeBusProtocol::onError(BeeBusErrorHandler handler) {
    _errorHandler = handler;
}

// --- Connection management ---
void BeeBusProtocol::nodeConnect() {
    _connectRequested = true;

    // An explicit connect() claims the address again. Whether anybody else
    // still holds it is then settled by the bus rather than by what was heard
    // before the application intervened.
    _duplicateNodeID = false;
    _duplicateSeenAt = 0;

    // Leaving as well as Offline: a departure takes a frame and a turn to
    // complete, and an application that asks to connect again inside that
    // window means it. Refusing would leave the node stuck announcing a
    // departure it has already changed its mind about.
    if (_state == BeeBusState::Offline || _state == BeeBusState::Leaving) {
        transitionToState(BeeBusState::Listening);
    }
}

void BeeBusProtocol::nodeDisconnect() {
    _connectRequested = false;

    if (_state == BeeBusState::Offline || _state == BeeBusState::Leaving) {
        return;
    }

    // A departure is announced on the token, and only a ring with somebody
    // else in it hands the token back. A node still looking for a ring, or
    // running one of its own, has nobody to tell and nothing that would ever
    // end Leaving, so it goes straight offline.
    if (isRingParticipant() && _ringMemberCount > 1) {
        transitionToState(BeeBusState::Leaving);
    } else {
        transitionToState(BeeBusState::Offline);
    }
    _operations.clearPendingOps();
}

void BeeBusProtocol::resumeAfterDuplicate() {
    BEEBUS_DEBUG_PROTOCOL("DUPLICATE: node ID %d is clear again", _nodeID);

    _duplicateNodeID = false;
    _duplicateSeenAt = 0;
    resetJoiningState();
    resetSolicitPacing();

    reportError(BeeBusError::StateChange, "Node ID is clear again, rejoining");
    transitionToState(BeeBusState::Listening);
}

bool BeeBusProtocol::isConnected() const {
    // Participation first, then membership: the member list fills while this
    // node is still a candidate, so a count above one is not yet a place in
    // the rotation. Testing the count alone would report a connected node all
    // through Joining and Requesting, while every operation is refused with
    // UnknownNode for exactly that window -- one answer for the application
    // and another for the operation, the same split isAlive() carries this
    // guard to avoid.
    //
    // Membership then adds liveness and company: checkMemberTimeouts() drops
    // a silent peer and handleBusIsolation() clears the list when nothing is
    // being received, so a count above one means someone else is really there.
    if (!isRingParticipant()) {
        return false;
    }
    return _ringMemberCount > 1;
}
