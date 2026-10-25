#pragma once
#include "BeeBusGlobal.h"

/**
 * BeeBusOperations - the queue of operations this node has issued and is
 * waiting to send or hear back about, and of responses it owes its peers.
 */
class BeeBusOperations {
public:
    // --- Constructor and destructor ---
    BeeBusOperations();
    ~BeeBusOperations();

    // --- Operation management ---
    // A completion for an operation that never entered the queue.
    //
    // An address on this node is read and written in place, so there is no
    // queued entry to retire — but the caller is owed the same single report as
    // any other accepted operation, and one that opens a slot per request and
    // waits to be told has no other way to close it.
    //
    // Held rather than reported on the spot: reporting inside the call that
    // accepted the operation runs application code before that call has
    // returned, and a caller has not yet recorded what it is waiting for.
    // Drained by loop(), which is where every other completion is reported
    // from. BeeBusModbusDataMap::dispatchChangeEvents holds register change events
    // for the same reason.
    void reportLocal(const BeeBusCompletion& handler, uint8_t nodeID, uint16_t address,
                     uint8_t function, BeeBusResult result);

    // Takes the queues' storage, once, when the protocol starts. Anything held
    // is dropped (the protocol has cancelled it first). False when the memory
    // is not there - every queue then refuses, so the owner must not start.
    bool begin(size_t maxOps, size_t maxResponses);
    void dispatchLocalCompletions();

    // Returns Queued on success, or QueueFull / Duplicate when refused.
    BeeBusResult addPendingOperation(const BeeBusPendingModbusOp& op, BeeBusProtocol& protocol);
    // False when there is no room: the response is not queued.
    bool addPendingResponse(const BeeBusPendingResponse& response);
    // Retire an operation that will get no reply. The result is stated rather
    // than defaulted: an operation dropped because it could not be encoded and
    // one finished because it was transmitted are different answers, and a
    // caller waiting on either needs to be told which.
    void removePendingOperation(const BeeBusPendingModbusOp& op, BeeBusResult result);

    // Marks a transmitted read as in flight, so it is not sent again while its
    // response is outstanding. Cleared by a retry or by the response arriving.
    void markAwaitingResponse(const BeeBusPendingModbusOp& op);
    void removePendingResponse(const BeeBusPendingResponse& sent);
    void clearPendingOperations();
    void clearPendingResponses();
    void clearNodeOperations(uint8_t nodeID);
    void applyFailsafeForNode(uint8_t nodeID);

    // --- Access methods ---
    BeeBusList<BeeBusPendingModbusOp>& getPendingOps();
    const BeeBusList<BeeBusPendingModbusOp>& getPendingOps() const;
    BeeBusList<BeeBusPendingResponse>& getPendingResponses();
    const BeeBusList<BeeBusPendingResponse>& getPendingResponses() const;

    // The same as the record forms, by the operation's key: what the frame
    // builder keeps of what it sent.
    void markAwaitingResponse(uint8_t destNodeID, uint16_t opId);
    void removePendingOperation(uint8_t destNodeID, uint16_t opId, BeeBusResult result);
    void removePendingResponse(uint8_t pduType, uint8_t destNodeID, uint8_t function,
                               uint16_t startAddr);

    // --- Status methods ---
    uint16_t getPendingOpCount() const;
    uint16_t getPendingResponseCount() const;
    void clearPendingOps();

    // Whether an operation may go out on the next token. A first attempt
    // always may; a retry has to wait out its backoff first.
    bool isOperationReady(const BeeBusPendingModbusOp& op, const BeeBusProtocol& protocol) const;

    // Whether this node has a read of its own outstanding to that peer, at that
    // function and address. Only a node that asked can be answered, so this is
    // what settles the one PDU shape a request and a response still share.
    bool hasOutstandingRead(uint8_t nodeID, uint8_t functionCode, uint16_t address) const;

    // --- Processing and cleanup ---
    void processPendingOperations(BeeBusProtocol& protocol);
    void cleanupTimedOutOperations(BeeBusProtocol& protocol);

    // --- Direct response matching and fulfillment ---
    bool matchAndFulfillResponse(const BeeBusModbusRequest& response, uint8_t srcNodeID);

    // Retires the request an exception response refers to, leaving the caller's
    // variable untouched. Returns false if no pending request matches.
    bool failPendingRequest(const BeeBusModbusRequest& response, uint8_t srcNodeID);

    // Next operation ID for this node. Wraps, which is harmless: the queue
    // holds at most 255 entries, so an ID cannot be reused
    // while its previous owner is still outstanding.
    uint16_t nextOpId() { return ++_nextOpId; }

    // Finds a pending operation by the ID carried on the wire, rather than by
    // destination and address, which are not unique.
    BeeBusPendingModbusOp* findByOpId(uint8_t srcNodeID, uint16_t opId);

    // Retires an acknowledged write. Returns false if nothing matches, which is
    // normal for a duplicate acknowledgement.
    bool completeAcknowledgedWrite(uint8_t srcNodeID, uint16_t opId, uint8_t status,
                                   BeeBusProtocol& protocol);

    void writeResponseToVariable(const BeeBusPendingModbusOp& op, const BeeBusModbusRequest& response);
    BeeBusPendingModbusOp* findMatchingRequest(uint8_t srcNodeID, const BeeBusModbusRequest& response);

    // --- Topic operations ---
    //
    // Kept apart from findMatchingRequest, which already carries three special
    // cases. Pass -1 for topicID or opcode to match any.
    BeeBusPendingModbusOp* findTopicOp(uint8_t srcNodeID, int topicID, int opcode);

    // Concludes a topic operation and drops it. False when nothing matches,
    // which is normal after its request timed out.
    bool completeTopicOp(uint8_t srcNodeID, int topicID, int opcode, BeeBusResult result);

    // Where the caller's request has got to. Pending while it is in the queue,
    // then the outcome until the next request for the same address replaces it.
    BeeBusRequestState requestState(uint8_t nodeID, uint16_t address,
                                    BeeBusResult& result) const;

    // Records the outcome of an operation that never entered the queue, so a
    // caller polling requestState sees a local address finish like any other.
    void noteCompleted(uint8_t nodeID, uint16_t address, BeeBusResult result);

    // What one data frame carries (BeeBusIO::sendDataFrame), by key: storage
    // taken by begin() with the queues, so building a frame neither allocates
    // nor holds a queue's worth of keys on the stack, whatever the depth.
    struct FrameOp { uint8_t node; uint16_t opId; bool keep; };
    struct FrameResponse { uint8_t type; uint8_t node; uint8_t function; uint16_t address; };
    BeeBusList<FrameOp> frameReady;
    BeeBusList<FrameOp> frameUnbuildable;
    BeeBusList<FrameOp> frameSent;
    BeeBusList<FrameResponse> frameResponses;

private:
    uint16_t _nextOpId = 0;

    // One outcome per (node, address), kept after the operation is retired so a
    // caller that only looks once a scan still sees it. Queueing a new
    // operation for the same address drops the old outcome, so what is here is
    // never from a request the caller has already moved past.
    struct FinishedOp {
        uint8_t nodeID;
        uint16_t address;
        BeeBusResult result;
    };
    BeeBusList<FinishedOp> _finished;

    void recordFinished(uint8_t nodeID, uint16_t address, BeeBusResult result);
    void dropFinished(uint8_t nodeID, uint16_t address);

    // --- Operation storage ---
    BeeBusList<BeeBusPendingModbusOp> _pendingOps;
    BeeBusList<BeeBusPendingResponse> _pendingResponses;

    // One outcome owed to a handler, with what the handler is called with.
    struct Owed {
        BeeBusCompletion handler;
        uint8_t nodeID;
        uint16_t address;
        uint8_t function;
        BeeBusResult result;
    };
    static Owed owed(const BeeBusCompletion& handler, const BeeBusPendingModbusOp& op,
                     BeeBusResult result);

    // --- Re-entrancy ---
    //
    // A completion handler is application code, and the natural thing for one
    // to do is queue the next operation: chain a request, retry a write that
    // failed, drive a state machine on. That call pushes onto _pendingOps and
    // moves its items, so any reference, pointer or iterator into the list held
    // across a handler can be left pointing at a different operation.
    //
    // The rule the whole class follows: an operation is removed from the queue
    // before its handler runs, and nothing reads the record afterwards. Every
    // path that concludes an operation goes through retireAt() so the rule is
    // stated once. BeeBusModbusDataMap::dispatchChangeEvents does the same for the
    // register change events.
    static const size_t NO_OP = (size_t)-1;

    // Removes the operation at `index` and then reports `result` to whoever
    // asked to be told. Safe for the handler to queue anything it likes.
    void retireAt(size_t index, BeeBusResult result);

    // Reports one outcome, and keeps the reporting flat.
    //
    // Removing the record before the handler runs makes a handler free to queue
    // work, but it does not bound the depth: a handler told its write was
    // superseded that writes the same address again supersedes the write that
    // replaced it, which calls the handler again, and the chain runs until the
    // stack is gone. So a conclusion reached while a handler is already running
    // is queued rather than nested, and drained by the outermost call. Every
    // handler then runs at the same depth, whatever it does.
    void dispatch(const Owed& owed);

    BeeBusList<Owed> _deferred;
    uint8_t _dispatchDepth = 0;

    // Outcomes of operations taken off the queue together (cleared, or lost
    // with their node), reported once all of them are off it.
    BeeBusList<Owed> _retiring;
    void dispatchRetiring();

    // Completions owed for operations served in place, waiting for the next
    // loop(). Distinct from _deferred, which exists to flatten a handler that
    // concludes another operation while it runs.
    BeeBusList<Owed> _localCompletions;

    // The index forms of the three lookups. An index is taken fresh each time
    // rather than held across a handler, so these are what the conclusion
    // paths use.
    size_t indexByOpId(uint8_t srcNodeID, uint16_t opId) const;
    size_t indexOfTopicOp(uint8_t srcNodeID, int topicID, int opcode) const;
    size_t indexOfMatchingRequest(uint8_t srcNodeID, const BeeBusModbusRequest& response) const;

    // --- Helper methods for direct response ---
    bool extractCoilData(const BeeBusModbusRequest& response, void* values, uint16_t maxValues,
                         uint16_t stride);
    bool extractRegisterData(const BeeBusModbusRequest& response, void* values, uint16_t maxValues,
                             uint16_t stride);

    // Unpacks a register response into a character buffer, terminated whatever
    // the sender supplied. Bounded in bytes, because the byte count the
    // response declares is what gets copied and it need not be even.
    bool extractStringData(const BeeBusModbusRequest& response, char* text, uint16_t maxBytes);
};
