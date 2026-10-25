#pragma once
#include "BeeBusPlatform.h"
#include "BeeBusTypes.h"

// Measured network timing.
//
// Every timeout in the protocol is a multiple of how long the ring takes to
// come round, and that figure is measured rather than configured. Two
// quantities are tracked, because a mixed ring has two:
//
//   T_rot     a full rotation. Ring-wide, and the basis for membership,
//             isolation, operation and response timeouts.
//   T_hop[n]  how long node n takes to accept the token. One link, so a fast
//             peer and a slow peer get the timeouts they each deserve.

// Samples before an estimate is trusted.
#define BEEBUS_ESTIMATOR_WARMUP     4

// Bounds on any derived timeout.
//
// The floor is what this node can be relied on to schedule within: a timeout
// shorter than the interval between its own looks at the bus cannot be
// enforced, whatever the network is doing. It is measured from that interval,
// so a node running a tight loop derives a tighter floor than one carrying
// heavy application code. BEEBUS_TIMING_FLOOR_MS is what stands in until the
// interval has been measured, and MIN is the point below which millisecond
// clocks on two nodes no longer resolve the difference.
//
// The ceiling stops one pathological sample from stretching failure detection
// out of usefulness.
#define BEEBUS_TIMING_FLOOR_MS      50
#define BEEBUS_TIMING_FLOOR_MIN_MS  5
#define BEEBUS_TIMING_CEILING_MS    60000

// Silence between frames assumed before the line rate is known from either
// the transport or the spacing of arrivals. Long enough to be safe on a slow
// line, and replaced the moment either says otherwise.
#define BEEBUS_GAP_COLD_US          5000

// What to assume about joining before anything has been measured. Named so the
// config can defer to them rather than repeating the figure.
#define BEEBUS_JOIN_CYCLE_MS        50
#define BEEBUS_JOIN_RESPONSE_MS     50

// Bounds on a byte-time sample taken from an arriving frame. A frame shorter
// than the minimum can arrive whole inside one buffer read and would time the
// read rather than the bus. The maximum is what a 16-bit microsecond figure
// holds, which reaches down to roughly 150 bps.
#define BEEBUS_BYTE_TIME_MIN_BYTES  6
#define BEEBUS_BYTE_TIME_MAX_US     65535

// Runs the byte time is the lowest of. A minimum rather than a mean, because a
// late read can only ever make the line look slower than it is, so the fastest
// run in the window is the truthful one. Over a window rather than over all
// time, because a line's rate is not a constant: a repeater inserted, a radio
// module swapped, a port reopened at a different baud all make it slower, and
// an all-time minimum can never rise to meet that. Wide enough that one
// unusually fast run does not set the figure alone, short enough that a
// segment which really has slowed is believed within a rotation or two.
#define BEEBUS_BYTE_TIME_WINDOW     8

// Silence that ends a run of bytes arriving outside any frame. Long enough to
// hold a run together at any rate worth supporting; a run that swallows a gap
// only reports the line as slower than it is, and the lowest sample wins.
#define BEEBUS_BYTE_RUN_GAP_US      100000

// Bytes a run may reach before it stops being read as a transmission. Two
// frames' worth on the wire covers the longest collision worth timing; beyond
// that the line is stuck or floating and is carrying nothing.
#define BEEBUS_BYTE_RUN_MAX_BYTES   2048

// How far apart the widest and narrowest gaps inside a run may be before it
// stops reading as one transmission. A line is even; separate arrivals are not.
#define BEEBUS_BYTE_RUN_SPREAD      4

// A timing field left at BEEBUS_AUTO is derived from measurement. Any other
// value pins that field and switches adaptation off for it alone. Zero was
// never a legal setting, so the sentinel is unambiguous.
#define BEEBUS_AUTO                 0

// Highest node ID the per-peer hop table keeps an entry for, at twelve bytes
// each. The default covers the whole address space, which is what a node needs
// to know every link in a ring that may hold any address.
//
// Lower it to buy that memory back on a small target, and assign node IDs from
// 1 upward so the ring fits inside it: an address above this still joins, is
// still timed and is still a full member, but shares the pooled estimate rather
// than holding one of its own. Leave headroom for nodes added later, since a
// ring that outgrows the table degrades to less exact timing rather than
// failing, but only the addresses inside it are timed individually.
#ifndef BEEBUS_HOP_TABLE_NODES
#define BEEBUS_HOP_TABLE_NODES  BEEBUS_NODE_ID_MAX
#endif

// A configured setting, or the built-in figure where none was given. Every
// tunable reads through this, so BEEBUS_AUTO means the same thing everywhere:
// use what the protocol was built with.
template <typename T>
inline T bb_setting(T configured, T builtIn) {
    return configured != (T)BEEBUS_AUTO ? configured : builtIn;
}

// How many rotations each derived timeout allows. Dimensionless on purpose:
// none of them changes when a node is added or a link slows down.
//
// A node transmits at least once per rotation, so surviving k rotations of
// silence means surviving k consecutive losses.
#define BEEBUS_ROTATIONS_MEMBERSHIP    16   // Silence before a peer is gone
#define BEEBUS_ROTATIONS_ISOLATION     32   // Silence before we conclude we are

// How long the token-pass path waits before giving up on a successor refusing
// the token. Stays a small fraction of the membership window: the ring is
// silent while this runs, and a value near that window would have every node
// time every other one out at the same instant.
#define BEEBUS_ROTATIONS_TOKEN_GIVEUP   2

// Silence before a peer that is refusing the token counts as gone rather than
// unlucky. A different question from the one above, which sizes a single wait;
// this one weighs evidence once the retry budget is spent. Kept at a quarter of
// the membership window so a stalled pass never approaches it.
#define BEEBUS_ROTATIONS_STILL_THERE    4

// Dead air before a member regenerates the token, over and above the whole
// retry budget the holder is entitled to. Regenerating while that node is still
// legitimately retrying puts a second token on the bus and resets the retry
// count that was about to conclude its successor had gone.
#define BEEBUS_ROTATIONS_RECLAIM        4

#define BEEBUS_ROTATIONS_REJOIN        30   // Grace for a ring healing itself

// Token passes a node may hear go by without one being addressed to it before
// it concludes the ring is running without it. Evidence rather than a timeout:
// a member is handed the token once per rotation. Comfortably more than
// BEEBUS_TOKEN_SHARE_MAX, since a slow node is given the token less often.
#define BEEBUS_PASSES_WITHOUT_TURN      8

// Rotations a node waits after joining before emptying a store-and-forward
// ring. A sensor that has just woken is joining a ring whose membership is
// still settling, and a history poured into that is a history poured away.
#define BEEBUS_ROTATIONS_FLUSH_SETTLE   4

// An operation needs one rotation out and another back, plus whatever the ring
// spends healing around a node that has just left.
#define BEEBUS_ROTATIONS_OPERATION     16   // Round trip for a request
#define BEEBUS_ROTATIONS_RETRY          1   // Backoff between attempts

// Safety factor on the pre-measurement rotation estimate. A ring being built
// carries solicitations and response windows, so its rotations run longer and
// less evenly than the steady-state ones later measured.
#define BEEBUS_JOINING_ROTATION_MARGIN  2

// How far a single observed rotation may exceed what the ring has demonstrably
// achieved before it stops counting as a rotation. The liveness family follows
// observed rotations, so without a bound a stall inflates the estimate and the
// inflated estimate tolerates a longer stall.
#define BEEBUS_OBSERVED_ROTATION_BOUND  8

// What a pinned configuration field is taken to describe, in rotations. A
// pinned field states one member of its family and the others keep their
// proportion to it, so pinning nodeTimeout does not collapse "is this peer
// gone" and "has this peer refused the token twice" onto one test.
#define BEEBUS_REFERENCE_NODE_TIMEOUT       BEEBUS_ROTATIONS_MEMBERSHIP
#define BEEBUS_REFERENCE_TOKEN_RECLAIM      BEEBUS_ROTATIONS_RECLAIM
#define BEEBUS_REFERENCE_OPERATION          BEEBUS_ROTATIONS_OPERATION

// Uneven token distribution: a node whose hop costs at least MIN_RATIO times
// the average of the rest is visited every Nth rotation instead of every one,
// capped at MAX. The ratio comes from the per-hop estimates, so a ring where
// every hop costs the same skips nobody. The cap keeps a skipped node's turn
// well inside its own membership window.
#define BEEBUS_TOKEN_SHARE_MIN_RATIO  4
#define BEEBUS_TOKEN_SHARE_MAX        4

// Per-hop allowance for the token-pass timeout before that peer's link has been
// timed. A starting point only: four hops replace it.
#define BEEBUS_BOOTSTRAP_TOKEN_RESPONSE_MS   50

// How much of a retry allowance is jittered, as a percentage. A backoff that
// doubles deterministically leaves two colliding nodes in step.
#define BEEBUS_RETRY_SPREAD_PERCENT          50

// A deterministic spread in [0, span), keyed on whatever the caller passes.
// Deterministic because a node has no entropy at start-up; all that is needed
// is that two nodes, or two attempts, land on different numbers.
inline bb_time_t bb_spread(uint32_t seed, bb_time_t span) {
    if (span == 0) return 0;
    uint32_t hash = seed * 2654435761u;      // Knuth multiplicative
    hash ^= hash >> 13;
    hash *= 2246822519u;
    hash ^= hash >> 16;
    return (bb_time_t)(hash % (uint32_t)span);
}

#define BEEBUS_ANNOUNCE_ATTEMPTS             3

// Frames a node hears while still waiting before it may announce again. The
// ring's own traffic sets the rate, so it means the same on any medium and at
// any size, where an interval in milliseconds would not. Without it a node
// arriving at a ring that is not soliciting says so three times and then waits
// in silence for however long the ring takes to come round to its gap.
// Well inside the backup sweep, because announcing is how a node normally gets
// in and the sweep is only what catches one that was not heard.
#define BEEBUS_ANNOUNCE_REARM_FRAMES         8

// Frames an announcement is spread over on a retry. Two nodes woken by the
// same frame announce on the same one; picking a different frame each
// separates them without needing a timer finer than one pass through loop().
#define BEEBUS_ANNOUNCE_SPREAD_FRAMES        4

// Smoothed estimate of a recurring interval: TCP's round-trip estimator
// (RFC 6298) in integer arithmetic. The variance term is what makes it fit a
// radio, whose collision avoidance gives a modest mean and a wide spread.
class BeeBusEstimator {
public:
    void reset() {
        _srtt = 0;
        _rttvar = 0;
        _samples = 0;
    }

    void addSample(bb_time_t sample) {
        if (sample > BEEBUS_TIMING_CEILING_MS) sample = BEEBUS_TIMING_CEILING_MS;

        if (_samples == 0) {
            _srtt = sample;
            _rttvar = sample / 2;
            _samples = 1;
            return;
        }

        if (_samples < BEEBUS_ESTIMATOR_WARMUP) {
            // Still warming up. Count the sample before smoothing it in, so a
            // ring whose rotations all measure zero still reaches readiness.
            _samples++;
        }

        // Signed, because a sample below the current estimate has to pull it
        // down as well as up.
        const int32_t diff = (int32_t)sample - (int32_t)_srtt;
        const int32_t magnitude = diff < 0 ? -diff : diff;

        _rttvar = (bb_time_t)((int32_t)_rttvar + (magnitude - (int32_t)_rttvar) / 4);

        // Asymmetric: derived timeouts feed back into the rotation they
        // measure, so growing at a quarter and shrinking at an eighth keeps
        // the loop from oscillating.
        _srtt = (bb_time_t)((int32_t)_srtt + (diff > 0 ? diff / 4 : diff / 8));
    }

    // True once enough samples have been seen to derive timing from.
    bool isReady() const { return _samples >= BEEBUS_ESTIMATOR_WARMUP; }

    // True once there is anything to go on. RTTVAR starts at half the first
    // sample, so the first derived timeout is three times what was measured and
    // narrows as evidence accumulates.
    bool hasSample() const { return _samples > 0; }

    // The mean interval, without any allowance for spread.
    bb_time_t smoothed() const { return _srtt; }

    // How long to wait before concluding something did not happen: the mean
    // plus four times the observed spread.
    // The floor is the caller's to state: it describes the host the estimate
    // will be acted on by, which an estimator of network timing cannot know.
    bb_time_t timeout(bb_time_t floorMs) const {
        const bb_time_t margin = _rttvar * 4;
        bb_time_t value = _srtt + (margin > 1 ? margin : 1);

        if (value < floorMs) value = floorMs;
        if (value > BEEBUS_TIMING_CEILING_MS) value = BEEBUS_TIMING_CEILING_MS;
        return value;
    }

private:
    bb_time_t _srtt = 0;
    bb_time_t _rttvar = 0;
    uint8_t _samples = 0;
};

// Per-peer hop estimates, indexed by node ID.
//
// Every node times every hop it hears, not only the one it hands the token to,
// so each holds the same view of how long the token takes to reach each member.
// That view is the one source the rest of the timing is drawn from: a rotation
// is the hops around the ring added up, and a peer whose hop is lengthening is
// a link going bad before it is a member gone.
//
// Indexed rather than cached, at the cost of an estimator per address, because
// an estimate evicted to make room is the one the ring is about to need.
class BeeBusHopTable {
public:
    // Whether this node ID has an entry of its own. Above the table's reach a
    // peer is still a full member and still timed; what it loses is its own
    // estimate, and the pooled one stands in. Timing is then less exact for
    // that peer, not absent.
    static bool holds(uint8_t nodeID) {
        return nodeID != 0 && nodeID <= BEEBUS_HOP_TABLE_NODES;
    }

    void reset() {
        for (uint16_t i = 0; i <= BEEBUS_HOP_TABLE_NODES; i++) _entries[i].reset();
    }

    void addSample(uint8_t nodeID, bb_time_t sample) {
        if (!holds(nodeID)) return;
        _entries[nodeID].addSample(sample);
    }

    // The estimate for a peer, or null if this node has never timed it or has
    // no room to.
    const BeeBusEstimator* find(uint8_t nodeID) const {
        if (!holds(nodeID) || !_entries[nodeID].hasSample()) return nullptr;
        return &_entries[nodeID];
    }

    // Discards a peer's history. A node that leaves and comes back may be a
    // different device on the same ID, or the same device on a different link.
    void forget(uint8_t nodeID) {
        if (holds(nodeID)) _entries[nodeID].reset();
    }

private:
    BeeBusEstimator _entries[BEEBUS_HOP_TABLE_NODES + 1];
};

