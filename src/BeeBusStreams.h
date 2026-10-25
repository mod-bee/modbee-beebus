#pragma once

#include "BeeBusTypes.h"
#include "BeeBusTopicTypes.h"

#include <functional>

// --- The reliable topic layer (protocol version 2) ---
//
// Three things are kept per peer, in one record:
//
//   the RELIABLE stream each way: this node numbers every RELIABLE value it
//     sends the peer (one count for all topics), and the peer says in its own
//     next frame what it has (STREAM_ACK). A number that never arrived makes
//     that topic due again, and it goes as its current value: an old value is
//     never retried, only the latest;
//   the peer's session, drawn when it started, so a restart is told apart from
//     the instance before it however quickly it came back;
//   the request window: every write that asks for an acknowledgement or a
//     result carries a request ID, and each is applied once.
//
// Layer 3, serve / bind, is built from the same pieces: a bind is a RELIABLE
// subscription and writes that ask for a result.

// Defaults for the stream, overridable in BeeBusConfig.
#define BEEBUS_STREAM_WINDOW              64   // RELIABLE values in flight to one peer
#define BEEBUS_ROTATIONS_STREAM_RESEND     4   // No acknowledgement progress for this long

// Token turns a node announces its session on, after starting or after
// dropping its view of the ring.
#define BEEBUS_SESSION_TURNS               3

// Writes held for a decision (read back after a scan, or a queued write not yet
// taken). Past it a write that asks for a result is answered BUSY.
#define BEEBUS_MAX_PENDING_RESULTS        16

// --- One peer ---
struct BeeBusPeer {
    uint64_t rxMissing = 0;   // Bit i: stream number (rxHigh - 1 - i) never arrived
    uint32_t reqSeen = 0;     // Bit i: request (reqHigh - 1 - i) was applied
    uint16_t session = 0;     // Its session, 0 until it has said
    uint16_t nextRequest = 1; // The request ID our next write to it carries, never 0
    uint16_t reqHigh = 0;     // The highest request ID applied from it
    uint16_t resultHigh = 0;  // The highest request of ours it has answered
    uint32_t resultSeen = 0;  // Bit i: request (resultHigh - 1 - i) answered
    uint8_t node = 0;
    uint8_t txNext = 1;       // The stream number our next RELIABLE value carries
    uint8_t txAcked = 0;      // The highest it said it has
    uint8_t txQuiet = 0;      // Rotations without progress while something is awaited
    uint8_t txQueued = 0;     // RELIABLE values queued and not yet numbered (scratch)
    uint8_t rxHigh = 0;       // The highest stream number received from it
    uint8_t flags = 0;
};

#define BEEBUS_PEER_RX_STARTED     0x01   // rxHigh means something
#define BEEBUS_PEER_REQ_STARTED    0x02   // reqHigh means something
#define BEEBUS_PEER_ACK_DUE        0x04   // A STREAM_ACK is owed to it
#define BEEBUS_PEER_RESULT_STARTED 0x10   // resultHigh means something

// RELIABLE values sent and not yet acknowledged.
inline uint8_t bb_peerInFlight(const BeeBusPeer& peer) {
    return (uint8_t)(peer.txNext - 1 - peer.txAcked);
}

// --- A write waiting for its result, on the owner ---
struct BeeBusPendingResult {
    BeeBusScalar written;      // What the writer asked for
    bb_time_t appliedAt = 0;   // When it reached the variable
    uint32_t appliedLoop = 0;  // The loop() it did, so a scan has run since
    uint16_t topicID = 0;
    uint16_t requestID = 0;
    uint16_t settleMs = 0;     // Given when a queued write is taken (topicApplied)
    uint8_t writer = 0;
    bool applied = false;      // A queued topic's write, once the program took it
    bool command = false;      // Taken as a command: answered once taken, not read back
};

// --- A write of ours that asked for a result, on the writer ---
//
// Polled (topicResult) by a caller that does not take events, as requestState
// is for completions. One record per (peer, topic) last written with a result,
// the oldest finished one given up when the table is full.
#define BEEBUS_MAX_RESULT_RECORDS         16

// Not on the wire: no decision came back within the result wait, or the write
// itself failed.
#define BEEBUS_OUTCOME_NONE        0x7F

struct BeeBusResultRecord {
    BeeBusScalar kept;         // The owner's value with the decision
    uint16_t topicID = 0;
    uint16_t requestID = 0;    // The newest request, once numbered
    uint16_t count = 0;        // Decisions so far, wrapping: a new one is a change
    uint8_t node = 0;
    uint8_t outcome = BEEBUS_OUTCOME_NONE;
    bb_time_t ackedAt = 0;     // When the owner acknowledged it
    bb_time_t since = 0;       // When it was first sent, for the overall limit
    BeeBusScalar asked;        // The value asked for, to ask again with
    uint8_t reasks = 0;
    uint8_t flags = 0;
};

#define BEEBUS_RESULT_F_WAITING    0x01   // A decision is owed
#define BEEBUS_RESULT_F_ACKED      0x02   // The owner has the write

// A writer that hears no decision within the wait asks again, under the same
// request ID, this many times before it gives up: the owner answers a repeat
// from what it decided, without applying it again. A decision is lost for good
// when the owner drops the writer from its ring for a moment (it cannot send
// to a node that is not a member), so waiting longer alone does not do.
#define BEEBUS_RESULT_REASKS               2

// Decisions an owner remembers, to answer a repeated request with.
#define BEEBUS_MAX_SENT_RESULTS           16

struct BeeBusSentResult {
    uint16_t topicID = 0;
    uint16_t requestID = 0;
    uint8_t writer = 0;
    uint8_t outcome = 0;
};

// --- Layer 3: serve ---
//
// What a write to a served topic does. VALUE: the owner keeps a value and says
// what it kept. COMMAND: each write is one event for the program, taken once.
enum class BeeBusServeKind : uint8_t {
    Value = 0,
    Command,
};

// The owner's decision on a write, from a validate callback. ACCEPTED stores
// the value; CHANGED stores the value the callback left in `value`; REFUSED
// stores nothing. Any other outcome is treated as REFUSED.
typedef std::function<uint8_t(uint8_t writer, uint16_t topicID, BeeBusScalar& value)>
    BeeBusValidateHandler;

struct BeeBusServe {
    bool writable = true;
    BeeBusServeKind kind = BeeBusServeKind::Value;
    // Refused outside [min, max] when min < max. In the value's own units.
    double min = 0.0;
    double max = 0.0;
    // Decides at once. Without one, the owner reads the variable back after
    // the next loop() and settleMs, and answers ACCEPTED if it still holds what
    // was written or CHANGED with what it holds.
    BeeBusValidateHandler validate;
    uint16_t settleMs = 0;
};

// The record a served topic keeps beside its binding.
struct BeeBusServed {
    BeeBusServe terms;
    uint16_t topicID = 0;
};

// --- Layer 3: bind ---
enum class BeeBusBindState : uint8_t {
    Offline = 0,   // The owner is not in the ring
    Syncing,       // Subscribed, waiting for the owner's value
    InSync,        // Holds the owner's value
    Pending,       // A change is with the owner, waiting for its decision
    Refused,       // The owner kept another value (now in the variable)
    Failed,        // No decision came back; the variable holds the owner's value
};

inline const char* bb_bindStateName(BeeBusBindState state) {
    switch (state) {
        case BeeBusBindState::Offline: return "OFFLINE";
        case BeeBusBindState::Syncing: return "SYNCING";
        case BeeBusBindState::InSync:  return "IN_SYNC";
        case BeeBusBindState::Pending: return "PENDING";
        case BeeBusBindState::Refused: return "REFUSED";
        case BeeBusBindState::Failed:  return "FAILED";
    }
    return "?";
}

// LIVE by default, like every primitive: RELIABLE is
// asked for where it is meant, on setpoints, settings and configuration.
struct BeeBusBindTerms {
    BeeBusQoS qos = BeeBusQoS::Live;
    float deadband = 0.0f;     // LIVE only: a RELIABLE bind carries every change
};

// One bind: the owner's value lands in `shadow` (the remote topic's variable)
// and is copied to the caller's variable whenever no change of the caller's is
// with the owner. A change of the caller's variable is noticed by comparing it
// with `synced`, what the bind last put there. The shadow is a block of its
// own, not part of the record, because records move when the table grows and
// the topic map holds a pointer to it.
struct BeeBusBinding {
    BeeBusScalar synced;       // What the caller's variable was last set to
    BeeBusScalar requested;    // The change with the owner
    BeeBusScalar kept;         // The owner's value with the last decision
    void* var = nullptr;       // The caller's variable
    uint8_t* shadow = nullptr; // Owned: 8 bytes, or textCap for a string
    char* syncedText = nullptr;  // Owned, string binds: what the variable was last set to
    uint32_t key = 0;          // (owner << 16) | topicID
    uint16_t textCap = 0;
    uint16_t requestID = 0;    // The request carrying the change, once numbered
    uint8_t type = 0;
    BeeBusBindState state = BeeBusBindState::Offline;
    uint8_t outcome = BEEBUS_OUTCOME_ACCEPTED;   // The last decision
    bb_time_t ackedAt = 0;     // When the owner acknowledged the change
    uint8_t reasks = 0;        // Times the change was asked again
    uint8_t flags = 0;
};

#define BEEBUS_BIND_F_VALUE        0x01   // The shadow holds a value from the owner
#define BEEBUS_BIND_F_ACKED        0x04   // The owner has the pending change
#define BEEBUS_BIND_F_RESEND       0x08   // The change is to be (re)sent
#define BEEBUS_BIND_F_EVENT        0x10   // A state change not yet raised

// --- Events ---

// What an owner decided about a write this node made with a result asked for.
// `text` is the kept text for a string topic, "" otherwise.
typedef std::function<void(uint8_t nodeID, uint16_t topicID, uint8_t outcome,
                           const BeeBusScalar& value, const char* text)>
    BeeBusTopicResultHandler;

// The owner decided a write that asked (this node is the owner).
typedef std::function<void(uint8_t writer, uint16_t topicID, uint8_t outcome)>
    BeeBusTopicDecidedHandler;

// A bind changed state. Raised once per change.
typedef std::function<void(uint8_t ownerID, uint16_t topicID, BeeBusBindState state)>
    BeeBusBindStateHandler;

// The digest a held string write is read back against.
uint32_t bb_resultTextDigest(const char* text, size_t length);
