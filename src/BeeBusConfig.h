#pragma once

#include "BeeBusPlatform.h"
#include "BeeBusTypes.h"
#include "BeeBusTiming.h"

// --- Per-instance protocol configuration ---
//
// `BeeBusConfig{}` is the intended way to start a node. Every timing field
// defaults to BEEBUS_AUTO, meaning the node measures it: how long the token
// takes to come round, and how long each individual peer takes to accept it.
// That is the only way one configuration can suit a hardwired hop at 115200
// baud and the same hop over a radio, and the only way adding a node later does
// not mean revisiting every node already in the field.
//
// Setting a field to a number pins it and turns adaptation off for that field
// alone; the rest of the node still tunes itself. Nothing here has to be set
// for a network to work.
//
// Settings belong to the instance rather than the process, so several nodes in
// one process can each carry their own. Durations are milliseconds unless the
// name says otherwise.
struct BeeBusConfig {
    // --- Bus timing ---

    // Minimum silence between frames, taken from the line: two byte times, and
    // never shorter than the interval this node looks at the bus in. That gap
    // is also the room a node waiting to join announces itself in, so it is not
    // merely dead air.
    //
    // There is no ordinary reason to set this. It exists for a transceiver that
    // needs longer to turn around than the line alone accounts for, and even
    // then a value here is a floor rather than a replacement: it is still
    // raised if the line turns out slower than it assumed.
    bb_time_t interframeGapUs = BEEBUS_AUTO;

    // How long a node listens for an existing ring before speaking. Staggered
    // per node ID so nodes powered up together do not all speak at once.
    bb_time_t initialListenPeriod = 2000;

    // --- Join and token ring ---

    // What to assume about joining before anything has been measured. There is
    // no ring to time before one has been built, so these start the phase that
    // creates the thing every other timeout is derived from.
    //
    // Neither is the operative figure for long. joinResponseTimeout is where
    // the contention window starts, and the window then widens on evidence that
    // answers here need more room and narrows when they do not; joinCycleInterval
    // paces a node that is still alone. BEEBUS_AUTO uses
    // BEEBUS_JOIN_CYCLE_MS and BEEBUS_JOIN_RESPONSE_MS, both 50 ms, which is
    // the floor under every measured timeout as well.
    bb_time_t joinCycleInterval    = BEEBUS_AUTO;
    bb_time_t joinResponseTimeout  = BEEBUS_AUTO;

    // Adaptive by default. Pinning one states that quantity directly, in
    // milliseconds; the rest of its family keeps its proportion to it.
    bb_time_t tokenResponseTimeout = BEEBUS_AUTO;   // One peer accepting the token
    bb_time_t tokenReclaimTimeout  = BEEBUS_AUTO;   // Dead air before regenerating it
    bb_time_t nodeTimeout          = BEEBUS_AUTO;   // Silence before a peer is gone

    // --- Operations ---

    bb_time_t operationTimeout = BEEBUS_AUTO;
    bb_time_t responseTimeout  = BEEBUS_AUTO;
    bb_time_t retryDelay       = BEEBUS_AUTO;

    // Attempts after the first, for a token pass and for an operation alike.
    // Held to BEEBUS_MAX_RETRIES: the counters that check it are single bytes,
    // and a larger figure would be compared against a counter that wraps before
    // reaching it, so a silent peer would never be removed and a lost operation
    // would never report a timeout. Anything above the ceiling is read as it.
    uint32_t  maxRetries       = 2;

    // --- Ring timing, in rotations ---
    //
    // Dimensionless on purpose: a rotation is measured, so none of these
    // changes when a node is added or a link slows down. BEEBUS_AUTO uses the
    // built-in figure named beside each. Pin one only to suit a segment that
    // has shown it needs it; they are a set, and moving one alone can leave a
    // node declared dead sooner than the ring can hand it the token.
    uint8_t rotationsMembership   = BEEBUS_AUTO;   // Silence before a peer is gone
    uint8_t rotationsIsolation    = BEEBUS_AUTO;   // Silence before we are the problem
    uint8_t rotationsTokenGiveUp  = BEEBUS_AUTO;   // One wait for a successor
    uint8_t rotationsStillThere   = BEEBUS_AUTO;   // Before a refusing peer counts as gone
    uint8_t rotationsReclaim      = BEEBUS_AUTO;   // Dead air before regenerating a token
    uint8_t rotationsRejoin       = BEEBUS_AUTO;   // Grace while a ring heals
    uint8_t rotationsOperation    = BEEBUS_AUTO;   // Round trip for a request
    uint8_t rotationsRetry        = BEEBUS_AUTO;   // Backoff between attempts
    uint8_t rotationsFlushSettle  = BEEBUS_AUTO;   // Settling before a buffer empties
    uint8_t passesWithoutTurn     = BEEBUS_AUTO;   // Passes heard, per member, before rejoining
    uint8_t rotationsStreamResend = BEEBUS_AUTO;   // RELIABLE: no ack progress before re-sending
    uint8_t rotationsResult       = BEEBUS_AUTO;   // A wait for the owner's decision (AUTO: its retries)

    // --- Reliable topics ---
    //
    // RELIABLE values in flight to one peer before further changes wait and
    // coalesce, 1..127 (more is taken as 127). BEEBUS_AUTO is
    // BEEBUS_STREAM_WINDOW (64).
    uint8_t streamWindow = BEEBUS_AUTO;

    // This node's session, which tells a restart apart from the instance
    // before it. BEEBUS_AUTO draws one at begin(), from the platform's random
    // source; pin it where a reproducible session is wanted.
    uint16_t sessionId = BEEBUS_AUTO;

    // --- Discovery ---
    //
    // How a node that wants in gets noticed, and how hard the ring looks for
    // one that cannot be. BEEBUS_AUTO uses the built-in figure.
    uint8_t solicitBackupRotations = BEEBUS_AUTO;  // Rotations between sweeps
    uint8_t solicitSpreadMax       = BEEBUS_AUTO;  // How far the window may widen
    uint8_t announceAttempts       = BEEBUS_AUTO;  // Announcements in one burst
    uint8_t announceRearmFrames    = BEEBUS_AUTO;  // Quiet frames earning the next burst
    uint8_t announceSpreadFrames   = BEEBUS_AUTO;  // Frames an announcement backs off over

    // --- Estimator behaviour ---
    //
    // How measurement itself behaves. timingFloorMs is what stands in until the
    // service interval is known and timingFloorMinMs is the point below which a
    // millisecond clock says nothing. estimatorWarmup is samples before an
    // estimate is trusted, and retrySpreadPercent is the jitter on a retry, so
    // two nodes backing off together do not collide again on every attempt.
    bb_time_t timingFloorMs     = BEEBUS_AUTO;
    bb_time_t timingFloorMinMs  = BEEBUS_AUTO;
    uint8_t   estimatorWarmup   = BEEBUS_AUTO;
    uint8_t   retrySpreadPercent = BEEBUS_AUTO;

    // --- Token share ---
    //
    // A hop this many times the average of the rest is visited once every that
    // many rotations, up to the maximum, so one slow radio on a fast ring costs
    // the ring less than it costs itself. BEEBUS_AUTO uses the built-in figure.
    uint8_t tokenShareMinRatio = BEEBUS_AUTO;
    uint8_t tokenShareMax      = BEEBUS_AUTO;

    // --- Failsafe ---

    // Clears I/O data belonging to a timed-out node, so a stale command cannot
    // hold a physical output energised after the commanding node vanishes.
    // Recommended for anything driving real outputs.
    bool enableFailSafe = false;

    // How many operations and how many responses this node can have queued,
    // 1..255. The queues take this much storage once, at begin(): a slot holds
    // its payload in place (BEEBUS_MAX_PDU_BYTES), so each costs about 300
    // bytes of RAM, and begin() fails, saying so, when the memory is not there.
    // BEEBUS_AUTO is BEEBUS_MAX_PENDING_OPS (50) each on a host and 16 each on
    // a board. Raise it for a node that keeps many requests in flight at once:
    // past the depth, a request is refused with QueueFull. Raise the responses
    // on a node that answers many requests at once as well: an answer past
    // that depth is not queued.
    uint8_t maxPendingOps       = BEEBUS_AUTO;
    uint8_t maxPendingResponses = BEEBUS_AUTO;
};
