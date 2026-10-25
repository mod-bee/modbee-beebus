#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// Gap solicitation and joining.
//
// Every member owns the address space between itself and its successor, which
// is exactly the range its own token pass asserts is empty. BEEBUS_HDR_SOLICIT
// on that pass invites the whole of it at once, and the members' gaps tile the
// address space, so one rotation covers every address in parallel.
//
// A node alone on a silent bus is a ring of one whose successor is itself, so
// its gap is everything: the bootstrap needs no special state and no role.

// Doubles a backoff without passing `cap`. In eight bits a plain doubling of
// 128 or more wraps to nothing, which would leave the gap never asked again.
static inline uint8_t bb_doubleUpTo(uint8_t value, uint8_t cap) {
    const unsigned doubled = (unsigned)value * 2u;
    return (uint8_t)(doubled < cap ? doubled : cap);
}

// --- Timing ---
bb_time_t BeeBusProtocol::controlExchangeMs() const {
    // Three frame times: the invitation, the spread an answer may be delayed
    // by, and the answer itself.
    const bb_time_t byteUs = lineByteTimeUs();
    if (byteUs == 0) return 0;
    return (byteUs * BEEBUS_CONTROL_FRAME_BYTES * 3 + 999) / 1000;
}

bb_time_t BeeBusProtocol::solicitWindowBudget(bool sharedAcrossRing) const {
    // A response window is bus silence the ring imposes on itself, and nothing
    // credits that silence back to the peers being timed out during it. So the
    // window has to stay inside the shortest patience any member has with a
    // peer that has gone quiet, or a ring that invites a gap evicts everybody
    // in it for doing as it asked.
    //
    // isMemberResponsive() is the shorter of the two tests and therefore the
    // binding one. Half of it, so a window and the frame that follows it still
    // leave room for the test to pass.
    if (_ringMemberCount <= 1) {
        return 0;   // Nothing to evict, so nothing to protect
    }

    bb_time_t budget = livenessTimeout(bb_setting<uint8_t>(_config.rotationsStillThere, BEEBUS_ROTATIONS_STILL_THERE),
                                       _config.nodeTimeout, BEEBUS_REFERENCE_NODE_TIMEOUT) / 2;

    // Every member owns a gap and may invite it on the same rotation, so a
    // rotation holds one window per member. What this node states has to fit
    // alongside the rest of them; what it merely honours does not, since a
    // stated window already carries that division at its sender.
    if (sharedAcrossRing) {
        budget /= _ringMemberCount;
    }

    return budget;
}

bb_time_t BeeBusProtocol::solicitWindowBaseMs() const {
    // What one invitation and its answer cost on this link, before any
    // allowance for several nodes contending. A hop is one frame out and one
    // back, the same shape as an invitation and its answer. The timeout rather
    // than the mean, so it carries the link's spread; warmed rather than
    // sampled, so one fast hop cannot shorten it.
    bb_time_t window = _observedHop.isReady()
                           ? _observedHop.timeout(bb_setting<bb_time_t>(_config.timingFloorMinMs, BEEBUS_TIMING_FLOOR_MIN_MS))
                           : bb_setting<bb_time_t>(_config.joinResponseTimeout, BEEBUS_JOIN_RESPONSE_MS);

    // Never shorter than the line takes to carry the exchange. At 1200 bps one
    // control frame outlasts the configured value.
    const bb_time_t airMs = controlExchangeMs();
    if (airMs > window) window = airMs;

    return window;
}

bb_time_t BeeBusProtocol::solicitWindowMs() const {
    // Stated on the wire, so only the solicitor derives it.
    const bb_time_t base = solicitWindowBaseMs();

    // Widened by however much answering here has been seen to need. The node
    // that has timed no hop is the case this exists for: the first on a segment
    // invites the whole address space with nothing measured, and on a
    // store-and-forward radio the byte time it can see belongs to the module's
    // UART rather than to the air, so the window it would otherwise state is an
    // order of magnitude short of one exchange.
    bb_time_t window = base * _solicitSpread;

    // Widened as far as the evidence asks, then held to what the ring can
    // stand. Left at sixteen, a segment that keeps producing corruption states
    // a window longer than the membership tests running underneath it, and the
    // ring evicts the gap it just invited.
    //
    // The bound applies to the widening alone and never cuts into the base: one
    // exchange is what the link costs, and a window below it invites nobody. So
    // a ring too fast to afford any spread states its base and stops there,
    // rather than an unanswerably short window.
    const bb_time_t budget = solicitWindowBudget(true);
    if (budget != 0 && window > budget) {
        window = budget > base ? budget : base;
    }

    if (window > BEEBUS_TIMING_CEILING_MS) window = BEEBUS_TIMING_CEILING_MS;
    return window;
}

bb_time_t BeeBusProtocol::getJoinWaitTimeout() const {
    // Silence waited out before listening again from scratch. Any frame
    // restarts it, so it bounds silence and not joining.
    bb_time_t interval = bb_setting<bb_time_t>(_config.joinCycleInterval, BEEBUS_JOIN_CYCLE_MS) + bb_setting<bb_time_t>(_config.joinResponseTimeout, BEEBUS_JOIN_RESPONSE_MS);
    if (_observedHop.hasSample()) {
        const bb_time_t observed = _observedHop.timeout(timingFloor());
        if (observed > interval) interval = observed;
    }

    return interval * bb_setting<uint8_t>(_config.solicitBackupRotations, BEEBUS_SOLICIT_BACKUP_ROTATIONS) * BEEBUS_JOINING_ROTATION_MARGIN;
}

bb_time_t BeeBusProtocol::getInitialListenTime() {
    if (_initialListenSet) {
        return _initialListenTime;
    }

    // Stagger the moment a node decides the bus is silent, so nodes powered up
    // together do not all start rings at once. A hash of the ID spreads every
    // address across the whole window.
    uint32_t hash = (uint32_t)_nodeID * 2654435761u;   // Knuth multiplicative
    hash ^= hash >> 13;
    hash *= 2246822519u;
    hash ^= hash >> 16;

    const bb_time_t spread = _config.initialListenPeriod > 0
                                 ? (bb_time_t)(hash % _config.initialListenPeriod)
                                 : 0;

    _initialListenTime = _config.initialListenPeriod + spread;
    _initialListenSet = true;
    return _initialListenTime;
}

bool BeeBusProtocol::hasJoinWaitTimedOut() const {
    return bb_elapsed(_joinWaitStart, bb_millis()) >= getJoinWaitTimeout();
}

// --- Pacing ---
bool BeeBusProtocol::shouldSolicitThisRotation() {
    // The invitation rides in a header the token carries anyway; what it costs
    // is the successor's wait for the window. Backs off while nothing changes.
    if (_rotationsSinceSolicit < _solicitInterval) {
        _rotationsSinceSolicit++;
        return false;
    }

    // Counted from the decision, not the send: one that could not reach the bus
    // has still used up its rotation.
    _rotationsSinceSolicit = 0;
    return true;
}

void BeeBusProtocol::noteSolicitSent() {
    // Reached only once the frame is on the bus. A round consumed by an
    // invitation never sent would have the gap sit out the one it can hear.
    if (_solicitPending) {
        if (_corruptionSinceSolicit) {
            // Something is waiting, so backing off is wrong. And hold the bus
            // longer when asking: a collision says the window was too narrow
            // for the number waiting, or for what this medium costs to answer
            // across.
            _solicitInterval = 1;
            _solicitSpread = bb_doubleUpTo(_solicitSpread,
                bb_setting<uint8_t>(_config.solicitSpreadMax, BEEBUS_SOLICIT_SPREAD_MAX));
        } else if (_announceHeard &&
                   _solicitInterval < bb_setting<uint8_t>(_config.solicitBackupRotations, BEEBUS_SOLICIT_BACKUP_ROTATIONS)) {
            // Nobody there, and a node that wanted in could have said so. Ask
            // less often, down to the backup rate.
            //
            // Evidence sets this, not the medium, which is what makes one rule
            // fit both. Where an announcement lands, a gap is empty by the time
            // the sweep reaches it and the ring keeps its rotation for its own
            // work. Where one cannot land, a store-and-forward radio being the
            // case in point, the sweep is the only thing that finds a waiting
            // node, so it stays at every rotation.
            _solicitInterval = bb_doubleUpTo(_solicitInterval,
                bb_setting<uint8_t>(_config.solicitBackupRotations, BEEBUS_SOLICIT_BACKUP_ROTATIONS));
        }

        // A round nothing collided in did not need the room it took.
        if (!_corruptionSinceSolicit && _solicitSpread > 1) {
            _solicitSpread = (uint8_t)(_solicitSpread / 2);
        }
    }

    // Zero is reserved for a node that has answered nothing yet.
    _solicitRound++;
    if (_solicitRound == 0) _solicitRound = 1;

    _solicitPending = true;
    _corruptionSinceSolicit = false;

    // A fresh round is fresh evidence. The line may have been repaired since
    // the last one, so the fault latch does not outlive the round that set it.
    _windowNoiseBytes = 0;
    _windowLineFault = false;
}

void BeeBusProtocol::resetSolicitPacing() {
    // Membership changed, so whatever contention there was is one node smaller.
    // The rate is left alone where a joiner can announce itself: somebody else
    // arriving says nothing about this node's own gap, and winding every member
    // back to inviting on every rotation whenever anyone joins costs a large
    // ring its rotation for as long as nodes keep arriving.
    //
    // Where nothing can announce itself, a membership change is the only sign a
    // node gets that others may be waiting, so it goes back to asking.
    if (!_announceHeard) {
        _solicitInterval = 1;
    }
    _rotationsSinceSolicit = 0;
    _solicitPending = false;
    _corruptionSinceSolicit = false;
    _windowNoiseBytes = 0;
    _windowLineFault = false;
    _ownWindowEnds = 0;
}

void BeeBusProtocol::resetJoiningState() {
    // A node starting its search for a ring has not completed a rotation in it.
    _rotationsSinceJoin = 0;

    // Whatever was learnt about the last segment does not carry to the next.
    _announceHeard = false;
    _joinWaitStart = bb_millis();
    _solicitedUs = false;
    _joinAnswerAt = 0;
    _answeredRound = 0;
}

void BeeBusProtocol::announceIfDue() {
    // Only while waiting to join, and only for a bounded number of attempts.
    if (_state != BeeBusState::Joining || !_io) {
        return;
    }
    if (_announceAttempts >= bb_setting<uint8_t>(_config.announceAttempts, BEEBUS_ANNOUNCE_ATTEMPTS)) {
        // Still here, and nothing has invited anybody since the last burst.
        // Earned by the ring's own traffic, so a node arriving at a ring that
        // has backed off keeps saying so instead of falling silent after its
        // first three frames.
        if (_announceIdleFrames < bb_setting<uint8_t>(_config.announceRearmFrames, BEEBUS_ANNOUNCE_REARM_FRAMES)) {
            _announceIdleFrames++;
            return;
        }
        _announceIdleFrames = 0;
        _announceAttempts = 0;
    }

    // Skipping whole frames separates two nodes woken by the same one. A gap is
    // too short to hold a backoff and too short to be scheduled within.
    if (_announceSkip != 0) {
        _announceSkip--;
        return;
    }

    if (!_io->announcePresence()) {
        return;
    }

    _announceAttempts++;
    _announceSkip = (uint8_t)bb_spread(
        (uint32_t)_nodeID * 977u + _announceAttempts, bb_setting<uint8_t>(_config.announceSpreadFrames, BEEBUS_ANNOUNCE_SPREAD_FRAMES));

    BEEBUS_DEBUG_PROTOCOL("JOINING: announced, attempt %d", _announceAttempts);
}

void BeeBusProtocol::handleJoinAnnounce() {
    // Only a member has a gap to invite. A joiner hearing its own byte echoed
    // back, or another joiner's, has nothing to do with it.
    if (_state != BeeBusState::Member) {
        return;
    }

    BEEBUS_DEBUG_PROTOCOL("JOIN: somebody is waiting");

    // Proof that a joiner here can be heard, which is what lets the sweep slow
    // down at all.
    _announceHeard = true;

    // Nothing is admitted, so it does not matter that only part of the ring
    // heard it. Set to fire on this node's next turn rather than reset to
    // zero, which would spend a turn counting first.
    _solicitInterval = 1;
    _rotationsSinceSolicit = _solicitInterval;
    _corruptionSinceSolicit = false;
}

void BeeBusProtocol::noteCorruptFrame(BeeBusCorruption kind, uint32_t noiseBytes) {
    // A corrupt frame inside our own window is two nodes answering at once.
    // Strictly inside it: a radio corrupts frames of its own accord, and
    // counting those would pin the search at its most eager setting.
    //
    // Our own window, not whichever member's invitation last went past. The
    // round number is this node's, the gap is this node's, and corruption in
    // somebody else's gap is evidence about theirs.
    if (!_solicitPending || !bb_isFuture(bb_millis(), _ownWindowEnds)) {
        return;
    }

    if (kind == BeeBusCorruption::Noise) {
        // A collision is bounded: two nodes answer, destroy each other and
        // stop. An undriven pair is not, and it is the more likely reading
        // once the run has outlasted anything two answers could have been.
        // Latched for the round rather than tested per run, so a fault that
        // arrives in pieces is not read as a fresh collision each time.
        _windowNoiseBytes += noiseBytes;
        if (_windowNoiseBytes > BEEBUS_CONTENTION_NOISE_MAX_BYTES) {
            if (!_windowLineFault) {
                BEEBUS_DEBUG_PROTOCOL("SOLICIT: %lu noise bytes in round %d, reading the line as faulty rather than contended",
                    (unsigned long)_windowNoiseBytes, _solicitRound);
            }
            _windowLineFault = true;
        }
    }

    // A round the line spent floating says nothing about who is waiting in the
    // gap, so it neither widens the window nor holds the rate at its fastest.
    if (_windowLineFault) {
        _corruptionSinceSolicit = false;
        return;
    }

    BEEBUS_DEBUG_PROTOCOL("SOLICIT: contention seen in round %d", _solicitRound);
    _corruptionSinceSolicit = true;
}

// --- Event handlers ---
void BeeBusProtocol::handleSolicitation(const BeeBusHeader& header, bool fromOurRing) {
    // The window the solicitor is holding, not one derived here: two sides
    // deriving it separately do not reach the same number.
    bb_time_t window = bb_decodeWindow(header.solicitWindow);

    // Held to what this node can stay quiet for without timing the ring out
    // around it. A peer stating its window correctly is already inside this,
    // so the agreement the design rests on is untouched; what it bounds is a
    // peer whose own estimate has run away, which one byte is enough to do —
    // the encoding reaches nearly four seconds.
    bb_time_t honourable = solicitWindowBudget(false);

    // Never less than this node would ask for itself. A peer on a slower link
    // legitimately states more than we would, and refusing to wait it out
    // would talk over the answer it asked for.
    const bb_time_t ourBase = solicitWindowBaseMs();
    if (honourable != 0 && honourable < ourBase) honourable = ourBase;

    if (honourable != 0 && window > honourable) {
        BEEBUS_DEBUG_PROTOCOL("SOLICIT: node %d asks for %ld ms, holding %ld",
            header.srcNodeID, (long)window, (long)honourable);
        window = honourable;
    }

    if (_state != BeeBusState::Joining && _state != BeeBusState::Listening) {
        // Members stay off the bus until the window closes, the successor
        // included. Only for our own ring: another ring must not silence ours.
        if (fromOurRing) {
            _solicitWindowEnds = bb_millis() + window;
            _solicitWindowOpen = true;
        }
        return;
    }

    // The ring is inviting, so there is nothing to tell it. Only a ring that
    // has stopped asking is worth announcing to, and this is what says so.
    _announceIdleFrames = 0;

    if (!bb_inGap(_nodeID, header.srcNodeID, header.successorID)) {
        return;   // Somebody else's gap
    }

    // One answer per round: a retransmitted invitation is the same invitation.
    if (header.solicitRound != 0 && header.solicitRound == _answeredRound) {
        return;
    }

    // Both ends of the pass are members of the ring being joined.
    admitMember(header.srcNodeID);
    admitMember(header.successorID);

    _answeredRound = header.solicitRound;
    _solicitedUs = true;

    // Drawn from the address and the round together, so two nodes that collide
    // once do not collide again.
    uint32_t hash = (uint32_t)_nodeID * 2654435761u + header.solicitRound;
    hash ^= hash >> 13;
    hash *= 2246822519u;
    hash ^= hash >> 16;
    const uint16_t ownByteUs = byteTimeUs();
    const bb_time_t answerMs = ownByteUs == 0
                                   ? 0
                                   : ((bb_time_t)ownByteUs * BEEBUS_CONTROL_FRAME_BYTES
                                      + 999) / 1000;
    // Leave room for the answer at the end of the window. With no timed line
    // there is no width to subtract, so keep back a fraction instead.
    const bb_time_t spread = (answerMs != 0 && window > answerMs)
                                 ? window - answerMs
                                 : window / 3 + 1;
    _joinAnswerAt = bb_millis() + bb_spread(hash, spread);

    BEEBUS_DEBUG_PROTOCOL("SOLICIT: gap %d..%d round %d, window %ld ms",
        header.srcNodeID, header.successorID, header.solicitRound, (long)window);

    transitionToState(BeeBusState::Requesting);
}

void BeeBusProtocol::handleJoinRequest(uint8_t joiningNodeID, uint8_t fromNodeID) {
    BEEBUS_DEBUG_PROTOCOL("JOIN: Node %d asks to join", joiningNodeID);

    // Somebody answered, so this round of contention is over for everyone,
    // whether or not they were the one soliciting. The window is held to keep
    // the bus clear for an answer, so an answer is the end of it: sitting out
    // the rest costs every member the remainder for nothing.
    // Whether it beat the window we stated, read before that window is closed
    // below. An answer to our own invitation arriving after it says the window
    // was too short for this medium, which is the one unambiguous measure of
    // that: a collision inside the window cannot show it, because a window too
    // short to hold the answers is too short to hold their collision either.
    // Against our own window, which is the one we stated and the one the gap
    // was answering. _solicitWindowEnds may belong to another member by now.
    const bool late = _solicitPending && _ownWindowEnds != 0 &&
                      !bb_isFuture(bb_millis(), _ownWindowEnds);

    _solicitWindowOpen = false;

    // Our own gap held somebody, so it may hold more: ask on every rotation
    // until one finds nobody. Whether a round found anyone is this node's own
    // evidence about its own gap, so somebody else's arrival does not spend it.
    if (bb_inGap(joiningNodeID, _nodeID, successor())) {
        _solicitPending = false;
        _solicitInterval = 1;

        if (late) {
            _solicitSpread = bb_doubleUpTo(_solicitSpread,
                bb_setting<uint8_t>(_config.solicitSpreadMax, BEEBUS_SOLICIT_SPREAD_MAX));
        }
    }

    // A node asking to join has started from nothing, so whatever turn bit we
    // last saw from it says nothing about the one it is about to send.
    forgetTurn(joiningNodeID);

    handleMemberJoined(joiningNodeID, fromNodeID);
}
