#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// Token passing, membership and measured timing.
//
// The ring is derived, not agreed: every node keeps a list of the members it
// has heard taking part and hands the token to the next ID above its own. Two
// nodes with the same membership compute the same ring.

// --- Membership ---
bool BeeBusProtocol::isMember(uint8_t nodeID) const {
    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        if (_ringMembers[i] == nodeID) {
            return true;
        }
    }
    return false;
}

void BeeBusProtocol::observeFrame(const BeeBusHeader& header) {
    const uint8_t src = header.srcNodeID;
    const uint8_t next = header.successorID;

    if (src == 0 || src == _nodeID) {
        return;
    }

    // Any frame that passed the CRC proves our receive path still works,
    // whoever sent it and whatever it says.
    const bb_time_t now = bb_millis();

    // A hop closes when the node the token was handed to speaks. Nothing else
    // is a hop: a retransmission comes from the node that already had it, a
    // reclaim comes after a stall, and a join request carries no token.
    if (_hopSuccessor != 0 && _hopSuccessor == src) {
        bb_time_t hop = bb_elapsed(_hopStart, now);
        hop = hop > _hopWindowMs ? hop - _hopWindowMs : 0;

        // The chain lining up does not prove the interval holds only one hop:
        // the pass may have been retransmitted, with the copies lost on the way
        // here. Bounded by the derived timeout rather than the mean, so a medium
        // that genuinely slows down carries the bound up with it.
        const bool plausible = !_observedHop.isReady() ||
                               hop <= _observedHop.timeout(timingFloor()) * 4;
        if (plausible && hop <= BEEBUS_TIMING_CEILING_MS) {
            _observedHop.addSample(hop);

            // And against the peer it belongs to. Every frame reaches every
            // node, so a hop between two other members is as measurable here as
            // one of our own, and timing all of them is what gives each node
            // the same picture of the ring rather than of its own corner.
            _hopTiming.addSample(src, hop);
        }
    }
    _hopSuccessor = 0;

    if (next != 0 && next != src) {
        noteTurnSeen(src, header.has(BEEBUS_HDR_TURN));
    }

    // And this frame opens the next hop, if it hands the token on. A frame
    // addressed to nobody ends the chain rather than continuing it.
    if (next != 0 && next != src) {
        noteHopStart(next, header.has(BEEBUS_HDR_SOLICIT)
                           ? bb_decodeWindow(header.solicitWindow) : 0);
    }

    _lastFrameReceived = now;
    _hasReceivedFrame = true;

    // And it proves there is a ring here to be let into, so a node waiting to
    // be solicited keeps waiting. Only silence sends it back to listening.
    if (_state == BeeBusState::Joining || _state == BeeBusState::Requesting) {
        _joinWaitStart = _lastFrameReceived;
    }

    // Before learning anything from it, work out whether it belongs to a ring
    // this node should be giving way to.
    considerMerge(header);

    // Somebody else is passing the token while we hold one, so there are two
    // tokens on this bus. Ours is the one to drop: the other is demonstrably
    // circulating, and a node that keeps a duplicate token talks over the ring
    // on every rotation. IEEE 802.4 resolves it the same way.
    // Only for a turn this node has not already seen that peer take. The
    // medium can hand over a copy of a pass made before this node took the
    // token, and a copy repeats the turn bit of the frame it copied where a
    // node taking the token afresh flips it. Standing down for a copy leaves
    // the ring with no token at all until somebody reclaims one, and the churn
    // while that happens evicts members and discards their pending work.
    const bool freshPass = next != 0 && next != src &&
                           !isKnownTurn(src, header.has(BEEBUS_HDR_TURN));

    if (_state == BeeBusState::HoldingToken && next != 0 && next != _nodeID &&
        freshPass && isMember(src)) {
        BEEBUS_DEBUG_PROTOCOL("TOKEN: Node %d is passing one too, dropping ours", src);
        reportError(BeeBusError::DuplicateToken, "Duplicate token detected, standing off");
        _tokenReceivedForUs = false;
        _loneToken = false;
        discardTimingSample();
        transitionToState(BeeBusState::Member);
    }

    // A node handing the token to itself is running a ring of one, which is
    // what a failed receiver looks like. It is not taking part in our ring, so
    // it neither enters our membership nor stays alive in it.
    if (next != 0 && next == src) {
        return;
    }

    // A member handing the token on names its successor, and that is what puts
    // the successor in the ring. Membership grows forward along the chain from
    // a node already known, which is enough to learn all of it: the chain is a
    // cycle, so following it forward from any one member reaches every other.
    if (next != 0 && isMember(src)) {
        _lastRingActivity = now;
        admitMember(src);
        admitMember(next);
        return;
    }

    // The other way round is the sender's own claim to be holding the token,
    // which nothing corroborates: taken at face value, anything transmitting on
    // the segment writes itself into the ring with one frame. Accepted only
    // from a node this ring has had before, which is a member coming back.
    if (next != 0) {
        if (isMember(next) && _lastHeardFrom[src] != 0) {
            admitMember(src);
            admitMember(next);
        }
        return;
    }

    // A frame carrying no token: a join request, a departure, a solicitation
    // from a ring we are not in. It keeps a member alive but admits nobody.
    if (isMember(src)) {
        noteHeardFrom(src);
    }
}

bb_time_t BeeBusProtocol::duplicateHoldoff() const {
    return livenessTimeout(bb_setting<uint8_t>(_config.rotationsMembership, BEEBUS_ROTATIONS_MEMBERSHIP),
                           _config.nodeTimeout, BEEBUS_REFERENCE_NODE_TIMEOUT);
}

void BeeBusProtocol::handleDuplicateNodeID(uint8_t nodeID) {
    (void)nodeID;

    // Every frame the duplicate sends arrives here, so the report is paced by
    // the same window that lifts the stand-down: once when the address is
    // first heard elsewhere, and again only after it has been quiet since.
    const bb_time_t now = bb_millis();
    const bool firstSighting =
        _duplicateSeenAt == 0 || bb_elapsed(_duplicateSeenAt, now) > duplicateHoldoff();
    _duplicateSeenAt = now;

    if (firstSighting) {
        BEEBUS_DEBUG_PROTOCOL("DUPLICATE: another node is using ID %d", _nodeID);
        reportError(BeeBusError::DuplicateNodeID,
                    "Another node is using this node ID");
    }

    // A node already carrying the ring's traffic keeps it; standing both down
    // would take out the working half of a commissioning mistake. If both are
    // running rings, the merge rule settles that first.
    if (_ringMemberCount > 1 &&
        (_state == BeeBusState::Member || _state == BeeBusState::HoldingToken ||
         _state == BeeBusState::PassingToken)) {
        return;
    }

    _duplicateNodeID = true;
    _operations.clearPendingOperations();
    _operations.clearPendingResponses();
    transitionToState(BeeBusState::Offline);
}

void BeeBusProtocol::considerMerge(const BeeBusHeader& header) {
    const uint8_t src = header.srcNodeID;
    const uint8_t next = header.successorID;

    if (next == 0) {
        return;   // Carries no token, so it says nothing about a ring
    }
    if (_state != BeeBusState::Member && _state != BeeBusState::HoldingToken &&
        _state != BeeBusState::PassingToken) {
        return;   // Not in a ring of our own to give up
    }

    // Knowing either end means this is plausibly our own ring, seen from a node
    // whose membership differs from ours for a moment. Standing down on that
    // would turn every lost frame on a busy link into a rejoin.
    if (isMember(src) || isMember(next)) {
        return;
    }

    // Two rings really are running on one bus. The one containing the lowest
    // node ID absorbs the other: both sides can evaluate that from what they
    // can already see, they always agree, and exactly one of them stands down.
    uint8_t theirs = src < next ? src : next;
    uint8_t ours = _nodeID;
    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        if (_ringMembers[i] != 0 && _ringMembers[i] < ours) ours = _ringMembers[i];
    }

    if (theirs >= ours) {
        return;   // Ours is the senior ring; the other side will stand down
    }

    standDown("Merging into a ring that holds a lower node ID");
}

void BeeBusProtocol::dropAllPeers() {
    for (;;) {
        uint8_t peer = 0;
        for (uint8_t i = 0; i < _ringMemberCount; i++) {
            if (_ringMembers[i] != _nodeID) {
                peer = _ringMembers[i];
                break;
            }
        }
        if (peer == 0) break;
        handleMemberLeft(peer, _nodeID);
    }

    // Everything this node held of every peer is gone, and each of them may
    // still hold this node's subscriptions and streams. A new session says
    // so: on rejoining, each peer that sees it drops what it held of the old
    // one and asks again, as it would for a restart. What is known of the
    // peers' own sessions stays true.
    const uint16_t previous = _session;
    do {
        _session = (uint16_t)(bb_entropy() >> 8);
    } while (_session == 0 || _session == previous);
    _sessionTurns = BEEBUS_SESSION_TURNS;
    for (size_t i = 0; i < _peers.size(); i++) {
        const BeeBusPeer known = _peers[i];
        _peers[i] = BeeBusPeer();
        _peers[i].node = known.node;
        _peers[i].session = known.session;
    }
    // Every writer numbers its requests to the new session afresh, so the
    // decisions held or remembered for the old numbering would answer the new
    // requests that happen to carry the same IDs.
    _results.clear();
    _sentResults.clear();

    // And this node's own requests belong to the session they were made in:
    // asked again under the new one, an owner would take them for new ones.
    // What became of them is not known.
    for (size_t i = 0; i < _resultRecords.size(); i++) {
        BeeBusResultRecord& record = _resultRecords[i];
        if ((record.flags & BEEBUS_RESULT_F_WAITING) == 0) continue;
        record.outcome = BEEBUS_OUTCOME_NONE;
        record.count = (uint16_t)(record.count + 1);
        record.flags = 0;
    }
    for (size_t i = 0; i < _bindings.size(); i++) {
        _bindings[i].requestID = 0;
        _bindings[i].flags = (uint8_t)(_bindings[i].flags & ~BEEBUS_BIND_F_ACKED);
    }
}

void BeeBusProtocol::standDown(const char* why) {
    BEEBUS_DEBUG_PROTOCOL("STAND DOWN: %s", why);
    reportError(BeeBusError::StateChange, why);

    dropAllPeers();

    _tokenReceivedForUs = false;
    _tokenConfirmed = false;
    _loneToken = false;
    discardTimingSample();
    resetSolicitPacing();
    resetJoiningState();
    transitionToState(BeeBusState::Joining);
}

void BeeBusProtocol::admitMember(uint8_t nodeID) {
    // Only an addressable node can take a place in the ring. 0 means "none"
    // and 255 is the broadcast destination, and a header naming either as a
    // successor or an arrival describes no node that could be handed a token.
    if (nodeID < BEEBUS_NODE_ID_MIN || nodeID > BEEBUS_NODE_ID_MAX || nodeID == _nodeID) {
        return;
    }

    const bb_time_t now = bb_millis();
    noteHeardFrom(nodeID);
    _lastFrameReceived = now;   // Proof the receive path still works
    _hasReceivedFrame = true;

    if (isMember(nodeID)) {
        return;
    }

    if (_ringMemberCount >= BEEBUS_MAX_KNOWN_NODES) {
        return;
    }

    _ringMembers[_ringMemberCount++] = nodeID;
    _passesWithoutTurn = 0;

    // A peer that has come back has lost whatever subscriber list it had, so
    // anything this node was subscribed to has to be asked for again.
    resubscribeTo(nodeID);

    // Where one node has arrived another may be, so solicit eagerly again.
    resetSolicitPacing();

    // The rotation in progress spans both ring sizes and describes neither.
    discardTimingSample();

    BEEBUS_DEBUG_PROTOCOL("MEMBER: Node %d joined (%d members)", nodeID, _ringMemberCount);
    reportError(BeeBusError::MemberJoined, "Peer entered the ring");

    if (_memberJoinedHandler) {
        _memberJoinedHandler(nodeID);
    }
}

void BeeBusProtocol::handleMemberJoined(uint8_t nodeID, uint8_t fromNodeID) {
    BEEBUS_DEBUG_PROTOCOL("MEMBER: Node %d announced by Node %d", nodeID, fromNodeID);
    (void)fromNodeID;   // Named for the log line, which compiles out
    admitMember(nodeID);
}

void BeeBusProtocol::handleMemberLeft(uint8_t nodeID, uint8_t fromNodeID) {
    if (nodeID == _nodeID) {
        return;   // Nothing may remove us from our own ring
    }

    // The announcer is alive, but announcing something is not taking part in
    // the ring: a node we have evicted must not readmit itself by telling us
    // about a third party.
    if (isMember(fromNodeID)) {
        noteHeardFrom(fromNodeID);
    }

    int removeIndex = -1;
    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        if (_ringMembers[i] == nodeID) {
            removeIndex = (int)i;
            break;
        }
    }
    if (removeIndex < 0) {
        return;
    }

    for (uint8_t i = (uint8_t)removeIndex; i + 1 < _ringMemberCount; i++) {
        _ringMembers[i] = _ringMembers[i + 1];
    }
    _ringMemberCount--;
    _passesWithoutTurn = 0;

    // Alone now: the rotation measured describes a ring that is gone, and on
    // a link that degraded it has grown to many times what a ring of one
    // takes. Every reclaim and rejoin allowance is drawn from it, so a node
    // left on its own would wait that long to reclaim its own token while
    // the others, equally alone, did the same. A ring of one starts
    // its measurement afresh, as it did when it first started alone.
    if (_ringMemberCount == 1) {
        _rotationTiming.reset();
        _observedRotation.reset();
    }

    // Drive the outputs this node commanded to their stated safe values. An
    // address with no safe state stated holds what it has.
    if (_config.enableFailSafe) {
        _dataMap.applyFailsafeForNode(nodeID);
        _operations.applyFailsafeForNode(nodeID);
    }

    _operations.clearNodeOperations(nodeID);

    // Its subscriptions to our topics go with it, and so does anything it told
    // us about its own. The bindings this application made stay: a peer coming
    // back should find its topics still bound, with their quality saying why
    // the values cannot be acted on meanwhile.
    _topicMap.removeNode(nodeID);

    // The gap it leaves behind is one this node now owns.
    resetSolicitPacing();

    // The rotation in progress is discarded for the same reason as above. The
    // departed node's hop history goes with it: the same ID coming back may be
    // a different device, or the same device on a different link.
    discardTimingSample();
    _hopTiming.forget(nodeID);
    forgetTurn(nodeID);

    BEEBUS_DEBUG_PROTOCOL("MEMBER: Node %d left (%d remaining)", nodeID, _ringMemberCount);
    reportError(BeeBusError::MemberLeft, "Peer left the ring");

    if (_memberLeftHandler) {
        _memberLeftHandler(nodeID);
    }
}

// --- Successor ---
uint8_t BeeBusProtocol::successor() const {
    if (_ringMemberCount <= 1) {
        return _nodeID;   // A ring of one: we hand the token to ourselves
    }

    // The smallest member ID above ours, wrapping at the top. Found by a single
    // pass, so no sort and no scratch array: the membership list is unordered
    // and a fixed-size copy of it would be 254 bytes of stack per call.
    uint8_t next = 0;
    uint8_t lowest = 0;

    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        const uint8_t id = _ringMembers[i];
        if (id == 0) continue;

        if (lowest == 0 || id < lowest) {
            lowest = id;
        }
        if (id > _nodeID && (next == 0 || id < next)) {
            next = id;
        }
    }

    if (next != 0) return next;      // Somebody above us
    if (lowest != 0) return lowest;  // Wrap to the bottom, possibly ourselves
    return _nodeID;
}

uint8_t BeeBusProtocol::memberAfter(uint8_t nodeID) const {
    uint8_t next = 0;
    uint8_t lowest = 0;

    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        const uint8_t id = _ringMembers[i];
        if (id == 0) continue;

        if (lowest == 0 || id < lowest) lowest = id;
        if (id > nodeID && (next == 0 || id < next)) next = id;
    }

    if (next != 0) return next;
    if (lowest != 0) return lowest;
    return nodeID;
}

uint8_t BeeBusProtocol::successorVisitInterval() const {
    // Nothing to skip to, or nothing measured to judge by.
    if (_ringMemberCount <= 2 || !_rotationTiming.isReady()) {
        return 1;
    }

    const bb_time_t hop = getHopTime(successor());
    const bb_time_t rotation = _rotationTiming.smoothed();
    if (hop == 0 || rotation <= hop) {
        return 1;
    }

    // One hop against the average of the rest. A uniform ring gives a ratio of
    // one and skips nobody. Safe only because a lap that skipped somebody is
    // not folded into the rotation estimate, which would otherwise make the
    // skipped hop look like an ever larger share and starve the node out.
    const bb_time_t others = (rotation - hop) / (bb_time_t)(_ringMemberCount - 1);
    if (others == 0) {
        return 1;
    }

    const bb_time_t ratio = hop / others;
    if (ratio < bb_setting<uint8_t>(_config.tokenShareMinRatio, BEEBUS_TOKEN_SHARE_MIN_RATIO)) return 1;
    return ratio > bb_setting<uint8_t>(_config.tokenShareMax, BEEBUS_TOKEN_SHARE_MAX) ? (uint8_t)bb_setting<uint8_t>(_config.tokenShareMax, BEEBUS_TOKEN_SHARE_MAX)
                                          : (uint8_t)ratio;
}

bool BeeBusProtocol::isLowestMemberID() const {
    return memberRank() == 0;
}

uint8_t BeeBusProtocol::memberRank() const {
    // How many members sort below this node. Zero for the lowest, and the same
    // answer on every node, since they are all sorting the same list.
    uint8_t rank = 0;
    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        if (_ringMembers[i] != 0 && _ringMembers[i] < _nodeID) {
            rank++;
        }
    }
    return rank;
}

// --- Token ---
void BeeBusProtocol::handleTokenReceived(uint8_t fromNodeID, bool addressedToUs) {
    (void)addressedToUs;

    // Any token frame heard while passing confirms the pass. Narrowing it to
    // the successor alone turns a single lost frame into a retry and a
    // duplicate token, since the next frame heard is then from further round.
    // Membership itself is learnt in observeFrame, from the chain.
    if (_state == BeeBusState::PassingToken) {
        _tokenConfirmed = true;
        _confirmedBy = fromNodeID;
        _tokenRetryCount = 0;
    }
}

bool BeeBusProtocol::isRepeatedTurn(const BeeBusHeader& header) {
    const uint16_t stamp = (uint16_t)(((uint16_t)header.srcNodeID << 1) |
                                      (header.has(BEEBUS_HDR_TURN) ? 1u : 0u));

    // A copy arrives within whatever the link delays and reorders by, which is
    // bounded by a hop. The genuine next turn of the same parity is two
    // rotations away, so no ring is small enough for the two to be confused.
    const bb_time_t now = bb_millis();
    bb_time_t recent = _observedHop.isReady()
                           ? _observedHop.timeout(timingFloor())
                           : bb_setting<bb_time_t>(_config.joinResponseTimeout, BEEBUS_JOIN_RESPONSE_MS);

    // Never as far back as the genuine next turn of the same parity, which is
    // two rotations away. A short ring on a fast link turns twice inside the
    // floor under every derived timeout, and a copy window that long would
    // read the ring's own traffic as copies of itself.
    // The genuine next turn of the same parity falls two rotations out, so the
    // memory has to expire before then and a hop is the margin: nothing can
    // arrive closer together than the ring hands the token on. Everything up to
    // that is room for a copy the link delayed.
    const bb_time_t rotation = getRotationTime();
    if (rotation != 0) {
        const bb_time_t span = rotation * 2;
        const bb_time_t margin = _observedHop.isReady() ? _observedHop.smoothed()
                                                        : rotation / 2;
        const bb_time_t bound = span > margin ? span - margin : rotation;
        if (recent > bound) recent = bound;
    }

    for (uint8_t i = 0; i < BEEBUS_TURN_HISTORY; i++) {
        if (_turnHistory[i] != stamp) continue;
        if (bb_elapsed(_turnHistoryAt[i], now) <= recent) return true;
        _turnHistory[i] = 0;
    }

    _turnHistory[_turnHistoryNext] = stamp;
    _turnHistoryAt[_turnHistoryNext] = now;
    _turnHistoryNext = (uint8_t)((_turnHistoryNext + 1) % BEEBUS_TURN_HISTORY);
    return false;
}

void BeeBusProtocol::noteTokenAcquired() {
    const bb_time_t now = bb_millis();

    // Rotations, not milliseconds: the store-and-forward settle window has to
    // mean the same thing on a wired segment and on a radio.
    if (_rotationsSinceJoin < 0xFF) _rotationsSinceJoin++;

    // Acknowledgements that stopped, and binds waiting on a decision, are
    // measured in rotations too.
    tickStreams();

    // This node's session, said to the whole ring on its first turns in it
    //: a peer that holds an earlier instance's state sees a session
    // it knows under another value, and drops that state. One broadcast
    // section a turn, three turns, rather than a confirmed operation per
    // peer: losing all three is as unlikely as losing three attempts of one,
    // and a ring joining at 9600 baud is not handed a burst of them.
    if (_sessionTurns > 0 && _ringMemberCount > 1) {
        BeeBusBytes pdu;
        if (BeeBusTopicFrame::buildSession(pdu, _session) &&
            queueTopicReply(BEEBUS_NODE_BROADCAST, BEEBUS_TOP_SESSION, 0, pdu)) {
            _sessionTurns--;
        }
    }

    // The first acquisition has nothing to measure against.
    if (_lastTokenSeen != 0) {
        // Zero is a real measurement: a short ring at 115200 baud comes round
        // inside one millisecond, which is all the clock resolves. The floor on
        // the derived timeout is what keeps a zero usable.
        const bb_time_t sample = bb_elapsed(_lastTokenSeen, now);

        // A rotation holds a hop per member, so one shorter than a single hop
        // is two nodes handing us the token at once rather than a rotation.
        if (_observedHop.hasSample() && sample < _observedHop.smoothed()) {
            _lastTokenSeen = now;
            _rotationSampleValid = true;
            _observedSampleValid = true;
            return;
        }

        // What the ring is actually managing, membership changes and reclaims
        // included, so a ring running slower widens its own liveness windows. A
        // rotation spent retrying a pass is excluded: that stall is already
        // bounded by the per-peer token timeout, and counting it here too is a
        // feedback loop. Beyond that, a rotation many times longer than the
        // ring has shown it can achieve was a recovery, and is clamped.
        if (_observedSampleValid) {
            bb_time_t observed = sample;
            if (_rotationTiming.isReady()) {
                const bb_time_t bound =
                    _rotationTiming.smoothed() * BEEBUS_OBSERVED_ROTATION_BOUND;
                if (observed > bound) observed = bound;
            }
            _observedRotation.addSample(observed);
        }

        // Only an uninterrupted one counts towards how fast it can go.
        if (_rotationSampleValid) {
            _rotationTiming.addSample(sample);
        }
    }

    _lastTokenSeen = now;
    _rotationSampleValid = true;
    _observedSampleValid = true;
}

// --- Measured timing ---
namespace {

// Rescales a pinned configuration value from the rotation count it describes to
// the one the caller wants, never down to zero.
inline bb_time_t bb_scaleRotations(bb_time_t value, uint8_t rotations, uint8_t reference) {
    if (reference == 0) return value;
    const bb_time_t scaled = (bb_time_t)((uint64_t)value * rotations / reference);
    return scaled > 0 ? scaled : 1;
}

} // namespace

bb_time_t BeeBusProtocol::presumedRotation() const {
    // Before any rotation has been measured, what a solicitation and its answer
    // cost, once per member, so it grows with the ring.
    const bb_time_t members = _ringMemberCount > 2 ? (bb_time_t)_ringMemberCount : 2;
    bb_time_t rotation = (bb_setting<bb_time_t>(_config.joinCycleInterval, BEEBUS_JOIN_CYCLE_MS) + bb_setting<bb_time_t>(_config.joinResponseTimeout, BEEBUS_JOIN_RESPONSE_MS)) * members;

    // On a long-range radio those constants are an order of magnitude short, so
    // the observed hop takes over where it is larger. Hops are measurable from
    // the second frame a node hears, long before it holds a token. The mean and
    // not the timeout: compounding four times the spread across every member
    // gives a rotation nobody could ever be late for.
    if (_observedHop.hasSample()) {
        const bb_time_t observed = _observedHop.smoothed() * members;
        if (observed > rotation) rotation = observed;
    }

    return rotation;
}

bb_time_t BeeBusProtocol::hopDerivedRotation() const {
    // A rotation is the hops in it, one per member, each taken from the peer it
    // belongs to. Added up rather than averaged: a ring holding one slow radio
    // link and a dozen fast wired ones is timed as what it is, where a mean
    // times it as thirteen links of some speed none of them runs at.
    //
    // Hops are chain-verified and have their response window taken off, so a
    // stall cannot get into one. That is what puts this outside the liveness
    // feedback loop and makes it the only estimate able to bound the others.
    // The timeout and not the mean, so every hop carries its own spread.
    if (!_observedHop.hasSample()) {
        return 0;
    }

    const bb_time_t members = _ringMemberCount > 2 ? (bb_time_t)_ringMemberCount : 2;

    bb_time_t total = 0;
    bb_time_t timed = 0;
    for (uint8_t i = 0; i < _ringMemberCount; i++) {
        const BeeBusEstimator* hop = _hopTiming.find(_ringMembers[i]);
        if (hop == nullptr) continue;
        total += hop->timeout(timingFloor());
        timed++;
    }

    // Nothing timed yet, so the pooled estimate is all there is.
    if (timed == 0) {
        return _observedHop.timeout(timingFloor()) * members;
    }

    // A member not yet timed stands in at the average of those that are. This
    // node never times itself, so there is always at least one, and reporting
    // only the hops on hand would make the ring look shorter than it is and
    // shorten every timeout drawn from it.
    if (timed < members) {
        total += (total / timed) * (members - timed);
    }
    return total;
}

bb_time_t BeeBusProtocol::ringTimeout(uint8_t rotations, bb_time_t configured,
                                      uint8_t reference) const {
    if (configured != BEEBUS_AUTO) {
        return bb_scaleRotations(configured, rotations, reference);
    }

    // The longer of the two, never the shorter. An allowance built on the clean
    // estimate alone expires while a ring is closing over a lost node,
    // discarding operations that were about to succeed.
    const bb_time_t clean = _rotationTiming.isReady() ? _rotationTiming.timeout(timingFloor()) : 0;
    const bb_time_t observed = _observedRotation.hasSample() ? _observedRotation.timeout(timingFloor()) : 0;

    bb_time_t rotation = clean > observed ? clean : observed;
    if (rotation == 0) rotation = presumedRotation();

    return (bb_time_t)rotations * rotation;
}

bb_time_t BeeBusProtocol::livenessTimeout(uint8_t rotations, bb_time_t configured,
                                          uint8_t reference) const {
    if (configured != BEEBUS_AUTO) {
        return bb_scaleRotations(configured, rotations, reference);
    }

    // One sample is enough here, unlike the clean-rotation family: declaring a
    // healthy node dead costs more than noticing a dead one late, and an early
    // estimate errs long.
    if (_observedRotation.hasSample()) {
        return (bb_time_t)rotations * _observedRotation.timeout(timingFloor());
    }

    // Nothing measured, so the presumed rotation stands in. No further margin
    // on top: it is already generous and the rotation count is a large
    // multiple, so the two compound. The margin belongs to the rotation
    // estimate, not to every timeout derived from it.
    return (bb_time_t)rotations * presumedRotation();
}

bb_time_t BeeBusProtocol::lineByteTimeUs() const {
    return _io ? _io->lineByteTimeUs() : 0;
}

uint16_t BeeBusProtocol::byteTimeUs() const {
    return _io ? _io->byteTimeUs() : 0;
}

bb_time_t BeeBusProtocol::interframeGapUs() const {
    // Derived in one place, by the layer that does the measuring.
    return _io ? _io->effectiveGapUs() : _config.interframeGapUs;
}

bb_time_t BeeBusProtocol::interframeGapMs() const {
    return (bb_time_t)((interframeGapUs() + 999) / 1000);
}

bb_time_t BeeBusProtocol::getRotationTime() const {
    return _rotationTiming.isReady() ? _rotationTiming.smoothed() : 0;
}

bb_time_t BeeBusProtocol::getRotationTimeout() const {
    return _rotationTiming.isReady() ? _rotationTiming.timeout(timingFloor()) : 0;
}

bb_time_t BeeBusProtocol::getHopTime(uint8_t nodeID) const {
    const BeeBusEstimator* hop = _hopTiming.find(nodeID);
    return (hop && hop->isReady()) ? hop->smoothed() : 0;
}

bb_time_t BeeBusProtocol::getHopTimeout(uint8_t nodeID) const {
    const BeeBusEstimator* hop = _hopTiming.find(nodeID);
    return (hop && hop->isReady()) ? hop->timeout(timingFloor()) : 0;
}

// --- Derived timeouts ---
bool BeeBusProtocol::isMemberAlive(uint8_t nodeID) const {
    // The single test for "is this node still there". Membership decisions all
    // route through it, so silence is the only thing that removes a node.
    return bb_elapsed(_lastHeardFrom[nodeID], bb_millis()) <=
           livenessTimeout(bb_setting<uint8_t>(_config.rotationsMembership, BEEBUS_ROTATIONS_MEMBERSHIP), _config.nodeTimeout,
                           BEEBUS_REFERENCE_NODE_TIMEOUT);
}

bool BeeBusProtocol::isMemberResponsive(uint8_t nodeID) const {
    // A fraction of the membership window, to avoid deadlock rather than out of
    // impatience: the ring is silent while a node retries a pass, so a retry
    // path that waited out the full window would have every node time every
    // other one out at the same instant.
    return bb_elapsed(_lastHeardFrom[nodeID], bb_millis()) <=
           livenessTimeout(bb_setting<uint8_t>(_config.rotationsStillThere, BEEBUS_ROTATIONS_STILL_THERE), _config.nodeTimeout,
                           BEEBUS_REFERENCE_NODE_TIMEOUT);
}

bb_time_t BeeBusProtocol::getMembershipCheckInterval() const {
    // Nothing can change between one rotation and the next, so checking more
    // often than that is wasted work on a slow ring and no faster on a fast one.
    return livenessTimeout(1, _config.nodeTimeout, BEEBUS_REFERENCE_NODE_TIMEOUT);
}

bb_time_t BeeBusProtocol::getIsolationTimeout() const {
    // Comfortably longer than the membership window, so ordinary node removal
    // happens first and only a genuine loss of the bus triggers withdrawal.
    return livenessTimeout(bb_setting<uint8_t>(_config.rotationsIsolation, BEEBUS_ROTATIONS_ISOLATION), _config.nodeTimeout,
                           BEEBUS_REFERENCE_NODE_TIMEOUT);
}

bb_time_t BeeBusProtocol::getRejoinTimeout() const {
    // Several token rotations, so a ring healing around a lost node has time to
    // settle before a healthy member concludes it has been dropped.
    return livenessTimeout(bb_setting<uint8_t>(_config.rotationsRejoin, BEEBUS_ROTATIONS_REJOIN), _config.tokenReclaimTimeout,
                           BEEBUS_REFERENCE_TOKEN_RECLAIM);
}

uint8_t BeeBusProtocol::reclaimRotations() const {
    // A node may hold the token for its first attempt and every retry after it,
    // each of them worth up to rotationsTokenGiveUp rotations. Only
    // once all of that has gone by without a frame is the token actually lost.
    const uint32_t budget = (uint32_t)bb_setting<uint8_t>(_config.rotationsTokenGiveUp, BEEBUS_ROTATIONS_TOKEN_GIVEUP) *
                            ((uint32_t)bb_retryLimit(_config.maxRetries) + 1u);
    const uint32_t total = budget + bb_setting<uint8_t>(_config.rotationsReclaim, BEEBUS_ROTATIONS_RECLAIM);
    return total > 255u ? 255u : (uint8_t)total;
}

bb_time_t BeeBusProtocol::getReclaimTimeout() const {
    // Reference and count are the same, so a configured tokenReclaimTimeout is
    // taken at face value rather than scaled.
    const uint8_t rotations = reclaimRotations();
    return ringTimeout(rotations, _config.tokenReclaimTimeout, rotations);
}

bb_time_t BeeBusProtocol::getTokenPassTimeout(uint8_t nodeID) const {
    // The one timeout that is per peer rather than ring-wide: a wired neighbour
    // answers in under a millisecond and one behind a radio takes a hundred.
    if (_config.tokenResponseTimeout != BEEBUS_AUTO) {
        return _config.tokenResponseTimeout + interframeGapMs();
    }

    // A completed pass to this peer beats every estimate below it.
    bb_time_t allowance = getHopTimeout(nodeID);

    if (allowance == 0) {
        // Never timed. Start from what the bus is observably doing.
        allowance = BEEBUS_BOOTSTRAP_TOKEN_RESPONSE_MS +
                    bb_setting<bb_time_t>(_config.joinCycleInterval, BEEBUS_JOIN_CYCLE_MS) + bb_setting<bb_time_t>(_config.joinResponseTimeout, BEEBUS_JOIN_RESPONSE_MS) +
                    interframeGapMs();

        // Those constants describe a bus nobody has timed, and at 1200 bps one
        // control frame alone outlasts them: a pass would be given up on before
        // the successor could physically have answered, so no pass ever
        // completes and nothing is ever measured to correct them.
        const bb_time_t airMs = controlExchangeMs();
        if (airMs > allowance) allowance = airMs;

        // A pass is one frame out and one back, which is one hop, and hops are
        // measurable from the bus before any peer has been timed. On a slow
        // radio the constants above are a fraction of what a hop costs.
        if (_observedHop.hasSample()) {
            const bb_time_t observed = _observedHop.timeout(timingFloor());
            if (observed > allowance) allowance = observed;
        }
    }

    // The backoff applies whether or not the peer has been timed: Karn's second
    // half is that a failing exchange raises the timer regardless of what the
    // estimate says. One short sample would otherwise pin a slow hop at a
    // fraction of its cost with no way back. The measurement takes over again
    // as the floor decays past it.
    if (_passFloorNode == nodeID && _passFloor > allowance) {
        allowance = _passFloor;
    }

    const bb_time_t ceiling = passAllowanceCeiling();
    return allowance > ceiling ? ceiling : allowance;
}

bb_time_t BeeBusProtocol::passAllowanceCeiling() const {
    // The ring is silent while this node waits, so waiting longer than the
    // window every other node judges membership by has them all conclude at the
    // same moment that everyone else has gone. The backoff exists to find a
    // slow link, not to outlast a dead one.
    const bb_time_t fromRing = livenessTimeout(bb_setting<uint8_t>(_config.rotationsTokenGiveUp, BEEBUS_ROTATIONS_TOKEN_GIVEUP),
                                               _config.nodeTimeout,
                                               BEEBUS_REFERENCE_NODE_TIMEOUT);

    // And no longer than the hops say a rotation can be. The liveness window
    // above follows the rotations the ring is managing, stalls included, and
    // those stalls are the ones this ceiling permits — so taking the ceiling
    // from that number alone closes a feedback loop. Hops are chain-verified
    // and cannot contain a stall, which breaks it. The smaller of the two
    // never the larger, so this only ever tightens the ring-wide rule.
    if (_config.nodeTimeout != BEEBUS_AUTO) {
        return fromRing;
    }
    const bb_time_t fromHops =
        (bb_time_t)bb_setting<uint8_t>(_config.rotationsTokenGiveUp, BEEBUS_ROTATIONS_TOKEN_GIVEUP) * hopDerivedRotation();
    if (fromHops != 0 && fromHops < fromRing) {
        return fromHops;
    }
    return fromRing;
}

// --- Failure handling ---
void BeeBusProtocol::checkMemberTimeouts() {
    if (_state != BeeBusState::Member && _state != BeeBusState::HoldingToken &&
        _state != BeeBusState::PassingToken) {
        return;
    }

    // Removal shifts the list down, so the index only advances when the entry
    // at it survived. Written this way rather than with a decrement after the
    // removal: that relied on a uint8_t underflowing to 255 and the loop's own
    // increment wrapping it back to 0, which is correct only by accident and
    // stops being correct the moment the index type changes.
    uint8_t i = 0;
    while (i < _ringMemberCount) {
        const uint8_t nodeID = _ringMembers[i];

        if (nodeID != _nodeID && !isMemberAlive(nodeID)) {
            BEEBUS_DEBUG_PROTOCOL("MEMBER: Node %d silent for too long, removing", nodeID);
            handleMemberLeft(nodeID, _nodeID);
            continue;   // Whatever shifted into this slot has not been looked at
        }
        i++;
    }
}

void BeeBusProtocol::handleBusIsolation() {
    BEEBUS_DEBUG_PROTOCOL("ISOLATED: no frame for %lu ms, leaving the ring to rejoin",
        (unsigned long)bb_elapsed(_lastFrameReceived, bb_millis()));
    reportError(BeeBusError::NetworkTimeout, "Bus isolation detected, rejoining");

    dropAllPeers();
    resetSolicitPacing();
    resetJoiningState();
    transitionToState(BeeBusState::Listening);
}
