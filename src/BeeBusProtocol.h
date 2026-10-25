#pragma once
#include "BeeBusGlobal.h"

class BeeBusIO;

// A BeeBus node: a logical token ring on a broadcast bus, every node a peer.
//
// Split across three translation units:
//   BeeBusProtocol.cpp   lifecycle, loop() and the state machine
//   BeeBusRing.cpp       token passing, successor, membership, measured timing
//   BeeBusDiscovery.cpp  solicitation, contention and joining
class BeeBusProtocol {
public:
    friend class BeeBusAPI;
    friend class BeeBusModbusHandler;
    friend class BeeBusIO;

    BeeBusProtocol();
    ~BeeBusProtocol();

    // --- Lifecycle ---

    // Takes what a running node needs - the frame I/O and the operation
    // queues, sized by `config` - BEFORE begin(), so a board without the
    // memory is told while nothing has changed yet. begin() with the same
    // config then allocates nothing.
    bool prepare(const BeeBusConfig& config);

    // False when prepare() would fail (the memory is not there) or the I/O
    // will not start; the node must then not be used.
    bool begin(uint8_t nodeID, BeeBusTransport* transport, const BeeBusConfig& config);

    // Takes the APPLICATION'S half of another protocol before begin(): what
    // this node serves (the data map, the topic map and their bindings), what
    // it subscribes to, the handlers and the configuration. What the other one
    // knew about its ring - members, timing, token, who subscribed to what -
    // is not taken: a node that changes its ID or its line joins a ring from
    // scratch, and its peers subscribe again when it arrives, as they do for
    // any node that comes back. For BeeBusAPI::rebind().
    void adoptApplication(BeeBusProtocol& from);
    void loop();

    void onError(BeeBusErrorHandler handler);

    // Raised when a peer enters or leaves this node's ring, after the
    // membership list has been updated.
    void onMemberJoined(BeeBusMemberHandler handler) { _memberJoinedHandler = handler; }
    void onMemberLeft(BeeBusMemberHandler handler) { _memberLeftHandler = handler; }

    void nodeConnect();
    void nodeDisconnect();
    bool isConnected() const;

    void reportError(BeeBusError error, const char* msg);
    uint8_t getNodeID() const { return _nodeID; }
    BeeBusState state() const { return _state; }

    // --- Ring membership ---
    bool isMember(uint8_t nodeID) const;
    uint8_t getMemberCount() const { return _ringMemberCount; }

    // A member by position, so the ring can be walked. 0 past the end. The
    // order is the order the token takes, so consecutive entries are the two
    // ends of one hop.
    uint8_t getMemberAt(uint8_t index) const {
        return index < _ringMemberCount ? _ringMembers[index] : 0;
    }

    // Whether this node is taking part in the ring rather than still joining
    // one. _ringMembers is populated while the node is still a candidate, since
    // a token is only accepted from a node already counted as a member, so
    // membership of the list is not membership of the ring.
    bool isRingParticipant() const {
        return _state == BeeBusState::Member || _state == BeeBusState::HoldingToken ||
               _state == BeeBusState::PassingToken;
    }

    // Admits a node to the ring and stamps it as heard from. Only for a node
    // that has demonstrably taken part: passed us the token, answered a
    // solicitation, or was announced by a member.
    void admitMember(uint8_t nodeID);

    // Applies everything learnt from a frame this node did not send: another
    // ring on the bus, who is taking part in ours, and when each was last heard.
    void observeFrame(const BeeBusHeader& header);

    // A token has just been handed to nodeID, by us or by anybody. Opens the hop
    // being timed; the successor's next frame closes it.
    void noteHopStart(uint8_t nodeID, bb_time_t windowMs = 0) {
        _hopStart = bb_millis();
        _hopSuccessor = nodeID;
        _hopWindowMs = windowMs;
    }

    // --- Liveness ---
    //
    // Stamps a node as heard from now. Never records 0, which marks a node never
    // heard from: a frame arriving in the millisecond the clock wraps would
    // otherwise read as never seen.
    void noteHeardFrom(uint8_t nodeID) {
        const bb_time_t now = bb_millis();
        _lastHeardFrom[nodeID] = now ? now : 1;
    }

    // Milliseconds since a node was last heard from, or BEEBUS_SILENCE_NEVER.
    // The number behind isAlive() and topic quality.
    bb_time_t silenceFor(uint8_t nodeID) const {
        if (nodeID == 0 || _lastHeardFrom[nodeID] == 0) return BEEBUS_SILENCE_NEVER;
        return bb_elapsed(_lastHeardFrom[nodeID], bb_millis());
    }

    // Whether a node is taking part in this ring now. The token proves liveness
    // once per rotation, so nothing polls for it.
    //
    // Taking part in a ring of our own is part of the question. A node that has
    // dropped out still holds its last membership list, and without this it
    // reports peers as alive that every read and write refuses as unknown —
    // one answer to "can I reach it" for the application and another for the
    // operation.
    bool isAlive(uint8_t nodeID) const {
        return isRingParticipant() && isMember(nodeID) && isMemberAlive(nodeID);
    }

    // Another node is transmitting under our node ID.
    void handleDuplicateNodeID(uint8_t nodeID);

    // Whether this node is currently stood down because its address is in use
    // elsewhere. Held while the duplicate keeps transmitting and cleared once
    // the address has been quiet long enough, so a host can poll it beside
    // isConnected() and tell a commissioning mistake from a slow start.
    bool hasDuplicateNodeID() const { return _duplicateNodeID; }

    // Silence from our own address before the stand-down is lifted. The same
    // window that declares any other peer gone, because it is the same
    // question asked about our own ID.
    bb_time_t duplicateHoldoff() const;

    void handleMemberJoined(uint8_t nodeID, uint8_t fromNodeID);
    void handleMemberLeft(uint8_t nodeID, uint8_t fromNodeID);

    // The node this one hands the token to: the smallest member ID above ours,
    // wrapping at the top.
    uint8_t successor() const;

    // The member after nodeID, for when the successor is skipped this rotation.
    uint8_t memberAfter(uint8_t nodeID) const;

    // How often the successor should be given the token, as one rotation in N.
    uint8_t successorVisitInterval() const;

    // --- Token ---
    void handleTokenReceived(uint8_t fromNodeID, bool addressedToUs = true);

    // Whether a token addressed to us is one already taken, by the turn bit.
    // Records the turn when it is a new one, so call exactly once per frame.
    bool isRepeatedTurn(const BeeBusHeader& header);

    // Whether a pass from this node carries a turn we have already seen it
    // take. A copy the medium made repeats the turn bit of the frame it
    // copied, where a node taking the token afresh flips it.
    bool isKnownTurn(uint8_t nodeID, bool parity) const {
        return _lastTurnSeen[nodeID] == (uint8_t)(parity ? 2 : 1);
    }

    void noteTurnSeen(uint8_t nodeID, bool parity) {
        _lastTurnSeen[nodeID] = (uint8_t)(parity ? 2 : 1);
    }

    // Our turn has been handed back, so the pass was never heard. Retransmits
    // it. Returns true when the token should be taken instead.
    bool noteRepeatedTurn(uint8_t fromNodeID) {
        // Both sides passing to each other means two tokens, one each. The
        // lower ID keeps its token; both sides decide this identically.
        if (_state == BeeBusState::PassingToken && _pendingSuccessor == fromNodeID &&
            _nodeID > fromNodeID) {
            _tokenConfirmed = false;
            _confirmedBy = 0;
            _tokenRetryCount = 0;
            discardTimingSample();
            transitionToState(BeeBusState::Member);
            return true;
        }

        // Only while a pass is outstanding. Answering from Member cascades: the
        // answer is itself a pass, read as a repeat by the next node round.
        if (_state != BeeBusState::PassingToken) {
            return false;
        }

        // _freshTurn false proves this turn has already been passed on, without
        // which a node still holding the token would hand out another.
        if (_freshTurn || _loneToken) return false;

        discardTimingSample();
        _retryingPass = true;
        transitionToState(BeeBusState::HoldingToken);
        return false;
    }

    // Forgets the last turn taken from a node, so one that restarts is not
    // mistaken for one retransmitting.
    void forgetTurn(uint8_t nodeID) {
        for (uint8_t i = 0; i < BEEBUS_TURN_HISTORY; i++) {
            if ((uint8_t)(_turnHistory[i] >> 1) == nodeID) _turnHistory[i] = 0;
        }
    }

    // Called at decode time for a token addressed to us. The false->true edge is
    // one completed rotation, so the ring is timed here: no loop() jitter, once
    // per acquisition, and never on the reclaim path.
    void setTokenReceivedForUs() {
        if (!_tokenReceivedForUs) noteTokenAcquired();
        _tokenReceivedForUs = true;
        _passesWithoutTurn = 0;
    }

    // A token pass heard that was not for us. A member is handed the token once
    // per rotation, so hearing it go round without arriving means this node is
    // no longer in the ring it thinks it is.
    void notePassNotForUs() {
        if (_passesWithoutTurn < 0xFFFFu) _passesWithoutTurn++;
    }

    // --- Discovery ---
    void handleSolicitation(const BeeBusHeader& header, bool fromOurRing);
    void handleJoinRequest(uint8_t joiningNodeID, uint8_t fromNodeID);

    // What the framer saw, since the two mean different things to the search.
    // A frame-shaped thing that failed its CRC, or a runt between two flags,
    // is two nodes answering at once. A run of bytes outside any frame may be
    // the same collision with no delimiter left in it, or it may be a pair
    // that is floating because the connector carrying its bias was pulled.
    enum class BeeBusCorruption : uint8_t {
        Frame,   // Delimited, and failed: a collision left something behind
        Noise,   // Undelimited: a collision left nothing, or the line is bad
    };

    // A corrupt frame during our own response window means two nodes answered.
    // noiseBytes is the length of the run for Noise, and 0 for Frame.
    void noteCorruptFrame(BeeBusCorruption kind = BeeBusCorruption::Frame,
                          uint32_t noiseBytes = 0);

    // A node saying it is waiting, without claiming to have joined. Changes no
    // membership: it only puts the search back to its most eager setting, so
    // the gap it sits in is invited on the next turn instead of whenever the
    // backoff came round to it.
    void handleJoinAnnounce();

    // Offered once for every frame that completes, from the gap that follows
    // it. Says this node is waiting, when it is.
    void announceIfDue();

    // What the line takes to carry a control frame out and an answer back,
    // from measured byte timing. Zero until the bus has been timed.
    bb_time_t controlExchangeMs() const;

    // What one invitation and its answer cost on this link, before any
    // allowance for contention. The floor under any window, stated or honoured.
    bb_time_t solicitWindowBaseMs() const;

    // The response window this node would state on an invitation of its own.
    bb_time_t solicitWindowMs() const;

    // The longest window this node can hold, or have held over it, without the
    // silence it asks for outlasting its own patience with a silent peer. Zero
    // where there is no ring to protect, which is the bootstrap: a node alone
    // has nobody to evict and needs its window generous.
    bb_time_t solicitWindowBudget(bool sharedAcrossRing) const;

    // --- Topic layer (BeeBusTopics.cpp) ---
    BeeBusTopicMap& getTopicMap() { return _topicMap; }
    const BeeBusTopicMap& getTopicMap() const { return _topicMap; }

    void onTopicChanged(BeeBusTopicChangedHandler handler) { _topicChangedHandler = handler; }
    void onTopicWritten(BeeBusTopicWrittenHandler handler) { _topicWrittenHandler = handler; }
    void onTopicGap(BeeBusTopicGapHandler handler) { _topicGapHandler = handler; }
    void onTopicComplete(BeeBusTopicCompleteHandler handler) {
        _topicCompleteHandler = handler;
    }
    void onTopicList(BeeBusTopicListHandler handler) { _topicListHandler = handler; }

    void raiseTopicChanged(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value) {
        if (_topicChangedHandler) _topicChangedHandler(nodeID, topicID, value);
    }
    void raiseTopicWritten(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value) {
        if (_topicWrittenHandler) _topicWrittenHandler(nodeID, topicID, value);
    }
    void onTopicStringChanged(BeeBusTopicStringChangedHandler handler) {
        _topicStringChangedHandler = handler;
    }
    void onTopicStringWritten(BeeBusTopicStringWrittenHandler handler) {
        _topicStringWrittenHandler = handler;
    }

    void raiseTopicStringChanged(uint8_t nodeID, uint16_t topicID, const char* text,
                                 uint16_t length) {
        if (_topicStringChangedHandler) {
            _topicStringChangedHandler(nodeID, topicID, text, length);
        }
    }
    void raiseTopicStringWritten(uint8_t nodeID, uint16_t topicID, const char* text,
                                 uint16_t length) {
        if (_topicStringWrittenHandler) {
            _topicStringWrittenHandler(nodeID, topicID, text, length);
        }
    }

    void raiseTopicComplete(uint8_t nodeID, uint16_t topicID, uint8_t opcode,
                            BeeBusResult result) {
        if (_topicCompleteHandler) _topicCompleteHandler(nodeID, topicID, opcode, result);
    }

    void raiseTopicGap(uint8_t nodeID, uint16_t topicID, uint8_t missed) {
        if (_topicGapHandler) _topicGapHandler(nodeID, topicID, missed);
    }
    void raiseTopicList(uint8_t nodeID, const BeeBusTopicEntry& entry) {
        if (_topicListHandler) _topicListHandler(nodeID, entry);
    }

    // Queues one topic PDU to a peer, already encoded.
    BeeBusResult queueTopicOp(uint8_t destNodeID, uint8_t opcode, uint16_t topicID,
                              const BeeBusBytes& pdu, BeeBusDelivery delivery,
                              bool noSupersede,
                              BeeBusCompletion onComplete);

    // The completion a topic operation gets when its caller gave none: the
    // protocol's topic-complete handler. `ctx` is the protocol.
    static void topicCompleted(void* ctx, uint8_t nodeID, uint16_t topicID, uint8_t opcode,
                               BeeBusResult result);

    // The same as a reply. Replies are never acknowledged and never retried;
    // the requester's own timeout is the backstop.
    // False when the response queue has no room.
    bool queueTopicReply(uint8_t destNodeID, uint8_t opcode, uint16_t topicID,
                         const BeeBusBytes& pdu);

    // Sends a value of a local topic to one node, advancing that destination's
    // stream. `sampleAt` is when the value was taken: now for a live publish,
    // earlier for one out of a ring.
    // `skipped` spends that many sequence numbers before this one, so a ring
    // can declare what it overwrote as a gap of that size.
    // `text` overrides the bound variable for a string topic, which is how a
    // buffered sample carries the value it was recorded with rather than
    // whatever the variable holds now.
    BeeBusResult sendTopicValue(uint8_t destNodeID, uint16_t topicIndex, uint8_t opcode,
                                const BeeBusScalar& value, bb_time_t sampleAt,
                                BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                                uint8_t skipped = 0, const char* text = nullptr);

    // Applies an inbound value to a registered remote topic: sequence, gap,
    // freshness and the bound variable.
    uint8_t applyTopicValue(uint8_t srcNodeID, uint16_t remoteIndex,
                            const BeeBusTopicPdu& pdu, bool broadcast = false);

    // Issues one subscription from the terms recorded against a remote topic.
    // Shared by subscribe(), the rejoin path and wildcard expansion.
    BeeBusResult issueSubscribe(uint8_t nodeID, uint16_t topicID);

    // Re-issues the subscriptions held for a peer that has come back: it has
    // lost its subscriber list.
    void resubscribeTo(uint8_t nodeID);

    // Forgets a subscription held for a peer that never arrived, so an
    // unsubscribe before the peer turns up is not undone when it does.
    void dropDeferredSubscribe(uint8_t nodeID, uint16_t topicID);

    // Holds a subscription that will not fit the queue yet, drained a few per
    // token.
    void deferSubscribe(uint8_t nodeID, uint16_t topicID);

    // Builds one page of this node's topic list, from the first topic whose ID
    // is at least `cursor`. `firstID` is the first entry included.
    bool buildTopicListPage(uint16_t cursor, uint8_t maxEntries, bool withNames,
                            BeeBusBytes& out, uint16_t& firstID);

    // --- Reliable topic layer (BeeBusStreams.cpp) ---
    //
    // This node's session, drawn at begin().
    uint16_t session() const { return _session; }

    // The record kept for a peer, made when `create` asks. Null when there is
    // none (or no memory for one). Insertions move records: use it at once.
    BeeBusPeer* peer(uint8_t nodeID, bool create);

    // A peer stated its session. A different one from before means the peer
    // restarted, whether or not the ring noticed it go.
    void notePeerSession(uint8_t nodeID, uint16_t session);

    // The peer is a new instance: what it held of ours (subscriptions,
    // streams, its request window) and what we held of its are forgotten, and
    // our subscriptions to it are made again.
    void peerRestarted(uint8_t nodeID);

    // Called by the frame builder as a topic section goes into a frame, with
    // its PDU in place: numbers a RELIABLE value on the stream to that peer,
    // and gives a write its request ID. `stored` is the operation's own copy
    // of the PDU, which keeps the request ID so a retry carries the same one;
    // null for a response.
    void stampTopicSection(uint8_t destNodeID, uint8_t* pdu, uint16_t length,
                           BeeBusModbusRequest* stored);

    // A RELIABLE value arrived from a peer under `seq`. False when it is a
    // copy or older than what is held: not applied, only acknowledged again.
    bool acceptStream(uint8_t srcNodeID, uint8_t seq);
    void handleStreamAck(uint8_t srcNodeID, const BeeBusTopicPdu& pdu);

    // Whether a write request from a peer is new, recording it if so.
    bool acceptRequest(uint8_t srcNodeID, uint16_t requestID);

    // --- Write results, on the owner ---
    //
    // The terms a served topic was bound with, or null.
    const BeeBusServed* served(uint16_t topicID) const;

    // Holds a write for a decision after the program has run. False when the
    // table is full (the writer is answered BUSY).
    bool holdResult(uint8_t writer, uint16_t topicID, uint16_t requestID,
                    const BeeBusScalar& written, bool applied);

    // Answers a writer.
    // `repeat`: answering a request asked again, from what was decided.
    void sendResult(uint8_t writer, uint16_t topicID, uint16_t requestID, uint8_t outcome,
                    bool repeat = false);
    // A repeated request: answered again if it was decided, otherwise its
    // decision is still to come.
    void answerRepeat(uint8_t writer, uint16_t topicID, uint16_t requestID);

    // The program has taken a queued write of this topic (one), which starts
    // its read-back, after `settleMs`; a command is answered once taken.
    void noteTopicApplied(uint16_t topicID, uint16_t settleMs = 0, bool command = false);
    void onTopicDecided(BeeBusTopicDecidedHandler handler) { _topicDecidedHandler = handler; }

    // --- Write results, on the writer ---
    void handleResult(uint8_t srcNodeID, const BeeBusTopicPdu& pdu);
    // A write asking for a result was queued, or has finished going: Immediate
    // when the owner has it, anything else when it never will.
    void noteResultWanted(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& asked);
    void noteResultWriteDone(uint8_t nodeID, uint16_t topicID, BeeBusResult result);
    // The last decision on a write of ours with a result. False when none.
    bool topicResult(uint8_t nodeID, uint16_t topicID, uint8_t& outcome, BeeBusScalar& kept,
                     uint16_t& count) const;
    void onTopicResult(BeeBusTopicResultHandler handler) { _topicResultHandler = handler; }

    // --- Layer 3 (BeeBusBinding.cpp) ---
    bool serve(uint16_t topicID, uint8_t type, void* variable, const BeeBusServe& terms,
               const char* name, uint16_t textCap);
    bool unserve(uint16_t topicID);
    bool bind(uint8_t ownerID, uint16_t topicID, uint8_t type, void* variable,
              const BeeBusBindTerms& terms, const char* name, uint16_t textCap);
    bool unbind(uint8_t ownerID, uint16_t topicID);
    BeeBusBindState bindState(uint8_t ownerID, uint16_t topicID) const;
    // The last decision on a change of this bind, and the value the owner kept.
    bool bindResult(uint8_t ownerID, uint16_t topicID, uint8_t& outcome,
                    BeeBusScalar& kept) const;
    // Says the caller's variable has changed, so the change goes with this
    // loop() rather than waiting to be noticed.
    void bindChanged(uint8_t ownerID, uint16_t topicID);
    void onBindState(BeeBusBindStateHandler handler) { _bindStateHandler = handler; }

    // A value of a bound topic was stored in its shadow.
    void bindValueArrived(uint8_t ownerID, uint16_t topicID);

    // Bytes held by the reliable layer's tables, for heapBytes().
    size_t streamHeapBytes() const;

    // --- Access for BeeBusAPI and BeeBusModbusHandler ---
    const BeeBusConfig& config() const { return _config; }

    BeeBusIO* getIO() { return _io; }
    BeeBusModbusDataMap& getDataMap() { return _dataMap; }
    BeeBusOperations& getOperations() { return _operations; }

    // --- Measured timing ---
    // What one rotation costs before any has been measured, from the join cycle.
    bb_time_t presumedRotation() const;

    // The longest a rotation can be given its hops. Zero until a hop is seen.
    bb_time_t hopDerivedRotation() const;

    // Derives a ring-wide timeout.
    //
    //   rotations    how many rotations to allow
    //   configured   the field the caller may have pinned to a number
    //   reference    how many rotations that pinned field describes, so a
    //                pinned value keeps its family's proportions
    bb_time_t ringTimeout(uint8_t rotations, bb_time_t configured, uint8_t reference) const;

    // The same for the liveness family, built on observed rather than clean
    // rotations so a link losing frames widens the window.
    bb_time_t livenessTimeout(uint8_t rotations, bb_time_t configured, uint8_t reference) const;

    bb_time_t getMembershipCheckInterval() const;
    bb_time_t getReclaimTimeout() const;

    // Rotations of dead air before the token counts as lost: the holder's whole
    // retry budget plus a margin.
    uint8_t reclaimRotations() const;
    bb_time_t getTokenPassTimeout(uint8_t nodeID) const;

    // What one byte costs on this bus in microseconds, 0 before any frame has
    // been timed, and how long a loop() iteration takes here.
    uint16_t byteTimeUs() const;
    bb_time_t lineByteTimeUs() const;
    bb_time_t getServiceInterval() const {
        return _serviceTiming.hasSample() ? _serviceTiming.smoothed() : 0;
    }

    // Shortest timeout this node can enforce, and so the floor under every one
    // it derives. Measured from how long it goes between looks at the bus.
    //
    // Raises the standing floor and never lowers it. A node is not the only
    // thing that has to schedule: its peer answers on a loop of its own, and a
    // link losing frames spends its margin on retries. What this node can
    // measure is its own half of that, which is grounds for demanding more
    // time and not for granting less.
    // The interval this node looks at the bus in, in microseconds. Zero until
    // two passes through loop() have been seen.
    bb_time_t serviceIntervalUs() const { return _serviceUs; }

    bb_time_t timingFloor() const {
        // A timeout shorter than the interval this node can check it in is not
        // enforceable, and that interval is the one quantity here measurable
        // without a network at all. The configured floor is what to assume
        // before it has been measured, not a bound on the measurement: a loop
        // running every millisecond held to fifty of them puts a floor under
        // every derived timeout an order of magnitude above what it describes.
        if (!_serviceTiming.isReady()) {
            return bb_setting<bb_time_t>(_config.timingFloorMs, BEEBUS_TIMING_FLOOR_MS);   // Nothing measured yet
        }
        return _serviceTiming.timeout(bb_setting<bb_time_t>(_config.timingFloorMinMs, BEEBUS_TIMING_FLOOR_MIN_MS));
    }

    // Silence this node leaves before transmitting. The configured gap is the
    // minimum; a bus slow enough that a byte would not fit inside it raises it,
    // because a node transmitting into a gap narrower than one byte transmits
    // over a frame whose first byte has not arrived.
    //
    // In whole milliseconds, rounded up so a sub-millisecond setting does not
    // truncate to no gap at all.
    bb_time_t interframeGapUs() const;
    bb_time_t interframeGapMs() const;

    bb_time_t getRotationTime() const;
    bb_time_t getRotationTimeout() const;
    bb_time_t getHopTime(uint8_t nodeID) const;
    bb_time_t getHopTimeout(uint8_t nodeID) const;

    // Whether a pass to this peer has ever completed.
    bool hasHopSample(uint8_t nodeID) const {
        const BeeBusEstimator* hop = _hopTiming.find(nodeID);
        return hop != nullptr && hop->hasSample();
    }

    // The most the token-pass path may ever wait, however a peer behaves.
    bb_time_t passAllowanceCeiling() const;

    // Karn's rule: an interval spanning a retry, a reclaim or a membership
    // change measured a recovery rather than a rotation, so it is discarded.
    void discardTimingSample() {
        _rotationSampleValid = false;
        _hopSampleValid = false;
    }

    // --- Join timing ---
    bb_time_t getJoinWaitTimeout() const;
    bb_time_t getInitialListenTime();

private:
    // --- Identity and membership ---
    uint8_t _nodeID;
    BeeBusState _state;
    BeeBusConfig _config;

    uint8_t _ringMembers[BEEBUS_MAX_KNOWN_NODES];
    uint8_t _ringMemberCount;

    // When each node ID was last heard from, member or not, indexed by ID.
    bb_time_t _lastHeardFrom[256];

    // The turn bit last seen on a pass from each node. 0 where none has been.
    uint8_t _lastTurnSeen[256] = {0};

    // --- Callbacks and subsystems ---
    BeeBusErrorHandler _errorHandler;
    BeeBusMemberHandler _memberJoinedHandler;
    BeeBusMemberHandler _memberLeftHandler;
    BeeBusIO* _io;
    BeeBusModbusDataMap _dataMap;
    BeeBusOperations _operations;

    // --- Topic layer ---
    BeeBusTopicMap _topicMap;
    BeeBusTopicChangedHandler _topicChangedHandler;
    BeeBusTopicWrittenHandler _topicWrittenHandler;
    BeeBusTopicStringChangedHandler _topicStringChangedHandler;
    BeeBusTopicStringWrittenHandler _topicStringWrittenHandler;
    BeeBusTopicGapHandler _topicGapHandler;
    BeeBusTopicCompleteHandler _topicCompleteHandler;
    BeeBusTopicListHandler _topicListHandler;

    // Subscriptions waiting for room in the operation queue, as
    // (nodeID << 16) | topicID. At most one per remote topic.
    BeeBusList<uint32_t> _deferredSubs;

    // Rotations since joining, for the store-and-forward settle gate.
    uint8_t _rotationsSinceJoin;

    // --- Reliable topic layer ---
    uint16_t _session = 0;
    uint8_t _sessionTurns = 0;                   // Turns left to announce the session on
    uint32_t _loopCount = 0;                     // loop() calls, for a read-back
    BeeBusList<BeeBusPeer> _peers;               // Sorted by node
    BeeBusList<BeeBusPendingResult> _results;    // Writes waiting for a decision
    BeeBusList<BeeBusServed> _served;            // Sorted by topic ID
    BeeBusList<BeeBusBinding> _bindings;         // Sorted by key
    BeeBusList<BeeBusResultRecord> _resultRecords;   // Writes of ours with a result
    BeeBusList<BeeBusSentResult> _sentResults;       // Decisions we sent, as owner
    // Asks a write's owner again for its decision, under the same request ID.
    bool reask(uint8_t nodeID, uint16_t topicID, uint16_t requestID, const BeeBusScalar& value,
               const char* text, BeeBusCompletion onComplete);
    static void reaskCompleted(void* ctx, uint8_t nodeID, uint16_t topicID, uint8_t opcode,
                               BeeBusResult result);
    BeeBusTopicResultHandler _topicResultHandler;
    BeeBusTopicDecidedHandler _topicDecidedHandler;
    BeeBusBindStateHandler _bindStateHandler;
    BeeBusResultRecord* resultRecord(uint8_t nodeID, uint16_t topicID, bool create);

    // Once a rotation: acknowledgements that stopped coming, and binds waiting
    // too long for a decision.
    void tickStreams();

    // Queues or refreshes the STREAM_ACK owed to a peer.
    void refreshStreamAck(uint8_t nodeID);

    // Read-backs that are due, every loop().
    void serviceResults();

    // Changes of bound variables, owner arrivals and departures, every loop().
    void serviceBindings();

    // The bind with this key, or the index it would go at.
    size_t bindingLowerBound(uint32_t key) const;
    BeeBusBinding* binding(uint8_t ownerID, uint16_t topicID);
    const BeeBusBinding* binding(uint8_t ownerID, uint16_t topicID) const;
    void setBindState(BeeBusBinding& bound, BeeBusBindState state);
    // Sends the caller's change of a bind to its owner.
    void sendBindChange(BeeBusBinding& bound);
    // The owner's value, from the shadow into the caller's variable.
    void copyShadow(BeeBusBinding& bound);
    static void bindWriteCompleted(void* ctx, uint8_t nodeID, uint16_t topicID,
                                   uint8_t opcode, BeeBusResult result);
    static void silentCompletion(void* ctx, uint8_t nodeID, uint16_t topicID,
                                 uint8_t opcode, BeeBusResult result);

    // Sends a subscription now. QueueFull when there is no room, for the
    // caller to hold it.
    BeeBusResult sendSubscribe(uint8_t nodeID, uint16_t topicID);

    // Gives a subscribe answer still queued to `destNodeID` for this topic the
    // value now, in place of a separate publish. False when there is none.
    bool refreshSubackValue(uint8_t destNodeID, uint16_t topicIndex, const BeeBusScalar& value);
    BeeBusTopicPdu _refreshPdu;   // Parsed there; large, so not on the stack
  public:
    BeeBusTopicPdu& scratchPdu() { return _refreshPdu; }
  private:

    // --- Token state ---
    bool isLowestMemberID() const;

    // Where this node sorts among the members it knows, 0 for the lowest. Sets
    // how long it waits before regenerating a token nobody else has.
    uint8_t memberRank() const;
    void noteTokenAcquired();

    // Start of the rotation currently being timed.
    bb_time_t _lastTokenSeen;

    // Ring-wide timing. _rotationTiming discards any rotation that spanned a
    // retry, so it reports the ring's true speed and drives operation and retry
    // timing. _observedRotation takes every rotation as it comes, so it reports
    // what the ring is managing now and drives the liveness windows.
    BeeBusEstimator _rotationTiming;
    BeeBusEstimator _observedRotation;
    BeeBusHopTable _hopTiming;

    // What one hop costs, taken from the chain rather than from holding the
    // token: a pass to B followed by a frame from B bounds one hop for whoever
    // is listening. Measurable before joining, so it sizes the contention slots
    // and stands in for an unmeasured rotation.
    BeeBusEstimator _observedHop;

    // How often this node gets to look at the bus. Local to the node and
    // needing no network, so it is measured from the second loop() onward and
    // is settled long before anything is joined.
    BeeBusEstimator _serviceTiming;
    bb_time_t _lastServiceAt;

    // The same interval in microseconds, for the one consumer whose own scale
    // is finer than a millisecond.
    bb_time_t _lastServiceUs;
    bb_time_t _serviceUs;

    bb_time_t _hopStart;             // When the token was last handed on
    uint8_t _hopSuccessor;           // Who to; 0 when nothing is open

    // The response window the frame that opened it stated, which the successor
    // spends waiting rather than working. Taken back off the sample: the window
    // is derived from these, so leaving it in feeds itself.
    bb_time_t _hopWindowMs;
    bool _rotationSampleValid;
    bool _hopSampleValid;

    // Cleared only by a token-pass retry. The observed estimate keeps rotations
    // lengthened by a membership change or reclaim, which are the ring running
    // slower, but not ones lengthened by this node's own retry backoff, which
    // the per-peer token timeout already bounds.
    bool _observedSampleValid;

    bb_time_t _lastHeldToken;

    // Rotations since the successor was last given the token, for a successor
    // slow enough to be worth skipping.
    uint8_t _rotationsSinceSuccessorVisit;

    bool _tokenReceivedForUs;
    bool _tokenConfirmed;

    // Who ended the pass. Any member transmitting releases this node from
    // PassingToken; only the successor's own frame times the link to it.
    uint8_t _confirmedBy;
    uint8_t _pendingSuccessor;
    uint8_t _tokenRetryCount;
    uint16_t _passesWithoutTurn;

    // The alternating turn bit. _turnParity is stamped on this node's own
    // passes. _turnHistory is the last few turns accepted, so a second copy is
    // recognised even when another node speaks between the two, which a ring
    // that turns quickly gives it every chance to. Short on purpose: a copy
    // arrives within whatever the link delays and duplicates it by, never long
    // after, and an entry kept past that would suppress the first genuine frame
    // of a node that had left and come back. Bounded by time and not by how
    // many entries are kept: the same node and turn comes round again every
    // second rotation quite legitimately, and a ring small enough for that to
    // fit inside the history would have every frame of it read as a copy. A
    // slot is 0 when unused, otherwise the source ID shifted up with the turn
    // bit below it; node 0 is not a valid source, so 0 cannot collide.
    bool _turnParity;
    uint16_t _turnHistory[BEEBUS_TURN_HISTORY];
    bb_time_t _turnHistoryAt[BEEBUS_TURN_HISTORY];
    uint8_t _turnHistoryNext;

    // Karn's rule, second half: back the timer off while a pass keeps failing,
    // or a link slower than the starting guess never completes one to measure.
    uint8_t _passFloorNode;
    bb_time_t _passFloor;

    // A successor whose retry budget ran out and which was routed round rather
    // than removed. Eviction costs an announcement, a solicitation cycle and a
    // join; going round costs one hop.
    uint8_t _skippedSuccessor;
    uint8_t _skippedSuccessorCount;

    // Whether this turn is a first attempt rather than a retry. Only a fresh
    // turn solicits and only a fresh turn moves the turn bit on.
    //
    // _retryingPass is set by the paths that retransmit a pass rather than
    // inferred from the previous state: a node handed the token again, or one
    // that lost its last peer, returns from PassingToken on a new turn.
    bool _retryingPass;
    bool _freshTurn;

    // --- Discovery state ---
    bb_time_t _joinWaitStart;
    bool _solicitedUs;

    // When this node answers the solicitation it accepted, spread across the
    // window it was given, and the round that answer belongs to. A round is
    // answered once: an invitation retransmitted is the same invitation.
    bb_time_t _joinAnswerAt;
    uint8_t _answeredRound;

    bb_time_t _initialListenTime;
    bool _initialListenSet;

    // Solicitation pacing. _solicitInterval counts rotations between
    // solicitations and doubles each time one finds nobody. Any membership
    // change resets it.
    uint8_t _solicitInterval;
    uint8_t _rotationsSinceSolicit;

    // The decision for the turn in hand, so retrying the send does not spend
    // the pacing again. 0 undecided, 1 not this turn, 2 this turn.
    uint8_t _solicitDecision;

    // Contention state, held only by the soliciting node: which round is out,
    // whether anybody answered, and whether the bus went corrupt meanwhile.
    uint8_t _solicitRound;

    // How far this node has widened its own response window, as a multiplier.
    uint8_t _solicitSpread;

    // Whether an announcement has ever arrived here, which is what says the
    // line can carry one.
    bool _announceHeard;

    bool _solicitPending;
    bool _corruptionSinceSolicit;

    // Announcements made since this node started waiting to join, and frames
    // still to let pass before the next one. Both cleared on entering Joining,
    // so a node that leaves and comes back says so again.
    uint8_t _announceAttempts;
    uint8_t _announceSkip;

    // Frames heard since the last announcement was spent, which is what earns
    // the next one.
    uint8_t _announceIdleFrames;

    // Every node hearing a solicitation stays off the bus until the window
    // closes, the successor included: its token is not usable until the gap has
    // had its slot.
    bb_time_t _solicitWindowEnds;
    bool _solicitWindowOpen;

    // When the window this node itself stated runs out. Separate from the one
    // above, which any member's invitation refreshes: holding the bus quiet is
    // owed to whoever asked, but contention is evidence about our own gap and
    // only what lands inside our own round is that. Sharing one field had a
    // collision in somebody else's gap widen every member's window at once.
    bb_time_t _ownWindowEnds;

    // Bytes seen outside any frame since our own round opened, and whether the
    // total has passed what a collision could account for. A pair with nothing
    // driving it delivers noise without end, and counting that as contention
    // pins the search at its widest setting for as long as the fault lasts.
    uint32_t _windowNoiseBytes;
    bool _windowLineFault;

    // The window this node last stated, as the gap will have decoded it.
    bb_time_t _statedWindowMs;

    // Whether the outstanding pass carried a solicitation. Such a pass does not
    // measure the link, since most of the successor's dwell was the window.
    bool _passCarriedSolicit;

    // Set when this node handed the token to itself, as a ring of one does.
    // Nothing will hand it back, so it is taken up again when the window closes.
    bool _loneToken;

    // --- Bus health ---
    // Time of the last valid frame from any node. _hasReceivedFrame is a
    // separate flag because zero is a legitimate timestamp after every wrap.
    bb_time_t _lastFrameReceived;

    // When a member was last heard passing the token within this ring. Distinct
    // from _lastFrameReceived: a node that has dropped out and is soliciting as
    // a ring of one puts frames on the bus without the ring running.
    bb_time_t _lastRingActivity;
    bool _hasReceivedFrame;

    // Per instance, not a function-local static: several nodes in one process
    // each run their own membership timeout check.
    bb_time_t _lastMemberTimeoutCheck;

    bb_time_t _stateEntryTime;

    // Set once another node has been heard using our ID, and when it was last
    // heard. The timestamp both paces the error report, which would otherwise
    // fire on every frame the duplicate sends, and times the stand-down.
    bool _duplicateNodeID;
    bb_time_t _duplicateSeenAt;

    // Whether the application wants this node on the network. Recovery from a
    // stand-down has to know that, because the stand-down itself forced the
    // state to Offline and lost it.
    bool _connectRequested;

    // Take part again after the duplicate has gone quiet.
    void resumeAfterDuplicate();

    // --- Ring helpers (BeeBusRing.cpp) ---
    void checkMemberTimeouts();

    // Whether a node has been heard from recently enough to count as present.
    // The one authority on membership.
    bool isMemberAlive(uint8_t nodeID) const;

    // The same over a shorter window, for the token-pass path, where the ring is
    // stalled while we decide and the peer has already refused several passes.
    bool isMemberResponsive(uint8_t nodeID) const;

    // Silence beyond this means we can transmit but no longer receive.
    bb_time_t getIsolationTimeout() const;

    // How long a node waits for its turn before concluding it has been dropped.
    bb_time_t getRejoinTimeout() const;
    void handleBusIsolation();

    // Two rings on one bus: the one holding the lowest node ID absorbs the
    // other, which both sides decide identically and neither has to say.
    void considerMerge(const BeeBusHeader& header);
    void standDown(const char* why);

    // Forgets every peer through the normal removal path, so failsafe handling
    // and queued operations are cleaned up as they are on a timeout.
    void dropAllPeers();

    // --- Topic helpers (BeeBusTopics.cpp) ---
    // Looks at a bounded slice of the local topics and queues what subscribers
    // are owed.
    void serviceTopics();

    // Called while holding the token, before the frame is built, so drained
    // samples ride this turn.
    void drainTopicBuffers();

    // --- Discovery helpers (BeeBusDiscovery.cpp) ---
    void resetJoiningState();

    // Records that an answer went unheard, and picks how many invitations to
    // stand back for before answering again.
    bool hasJoinWaitTimedOut() const;

    // Whether this turn carries a solicitation. Decides only; noteSolicitSent
    // commits, once the frame has gone out.
    bool shouldSolicitThisRotation();
    void noteSolicitSent();
    void resetSolicitPacing();

    // --- State machine (BeeBusProtocol.cpp) ---
    void transitionToState(BeeBusState newState);
};
