#pragma once

#include "BeeBusTopicFrame.h"

// --- Topic storage ---
//
// Two roles, stored apart: a topic this node publishes needs a subscriber list
// and a sequence counter, a peer's topic needs neither.
//
// Sorted lists rather than trees: insert is an O(n) move, lookup a binary
// search, and nothing is allocated per entry. The lists grow only as topics are
// bound or peers subscribe, never from a publish.
//
// Insert moves entries, so lookups return indices and the accessors are used
// immediately. Nothing holds a record pointer across an add.

#define BEEBUS_TOPIC_NONE          0xFFFF

// Lists double up to this many entries and then grow by a fixed step, so a
// large map never needs half its own size again in free memory to grow once.
#define BEEBUS_TOPIC_GROWTH_STEP   256

// Contracts are indexed by a byte, so a node holds at most 255 distinct sets of
// subscription terms.
#define BEEBUS_SUB_NONE            0xFF

// Deepest ring. One byte of sequence cannot express a longer flush: a step over
// 127 reads as stale.
#define BEEBUS_TOPIC_BUFFER_MAX    127

// Local topics examined per loop(). The cursor wraps, so the whole map is swept
// every localCount() / this calls.
#define BEEBUS_TOPIC_SCAN_PER_LOOP 32

#if BEEBUS_TOPIC_NAMES
// Glob match: `*` is any run of characters, `?` exactly one. Topic names are
// flat, so there are no level separators and no level wildcards.
bool bb_globMatch(const char* pattern, const char* text);
#endif

// --- Records ---

// A topic this node owns and publishes. Fields widest first, to avoid padding.
struct BeeBusLocalTopic {
    void* var = nullptr;
#if BEEBUS_TOPIC_NAMES
    const char* name = nullptr;   // A literal or otherwise static string, never owned
#endif
    BeeBusScalar last;        // Value at the last scan, for change detection
    bb_time_t lastPubAt = 0;
    uint16_t writeCount = 0;  // Writes accepted from a peer, wrapping
    uint16_t id = 0;
    uint16_t subHead = BEEBUS_TOPIC_NONE;   // First subscriber slot
    uint8_t seq = 0;          // Stream to a destination with no slot of its own
    uint8_t flags = 0;
    uint8_t subCount = 0;
    uint8_t bufferDepth = 0;  // 0 when the topic is not buffered
    uint8_t localFlags = 0;   // BEEBUS_TOPIC_L_*: this node only, never on the wire

    // A string topic: `var` is the caller's buffer of textCap bytes. lastText
    // is the text at the last scan (owned, textCap bytes), and textVersion
    // counts its changes, so a subscriber slot only keeps the version it got.
    char* lastText = nullptr;
    uint16_t textCap = 0;
    uint16_t textVersion = 0;
};

// A peer's topic, bound to a local variable. The variable holds the value; the
// record holds what the variable cannot say.
struct BeeBusRemoteTopic {
    void* var = nullptr;
#if BEEBUS_TOPIC_NAMES
    const char* name = nullptr;
#endif
    uint32_t key = 0;         // (nodeID << 16) | topicID
    bb_time_t rxAt = 0;       // Back-dated by the age a buffered sample carried
    uint16_t rxCount = 0;     // Accepted publishes, wrapping
    uint16_t missedCount = 0; // Publishes the sequence says never arrived, saturating
    uint8_t type = 0;
    uint8_t lastSeq = 0;
    uint8_t contractId = BEEBUS_SUB_NONE;   // Terms we subscribed under
    uint8_t flags = 0;
    uint16_t textCap = 0;     // A string topic: bytes in the caller's buffer
};

// Set once a value has arrived, so quality can tell "never received" from a
// value that happens to be zero. Survives the publisher leaving.
#define BEEBUS_REMOTE_F_RECEIVED   0x01

// Set while a subscription to this topic is outstanding or established.
#define BEEBUS_REMOTE_F_SUBSCRIBED 0x02

// Set while lastSeq means something. Cleared when the publisher leaves, since
// one that comes back numbers its stream from the beginning again.
#define BEEBUS_REMOTE_F_SEQ        0x04

// The subscription asks for QoS RELIABLE.
#define BEEBUS_REMOTE_F_RELIABLE   0x08

// One destination for one local topic. Allocated from an arena with a free
// list, so subscribing and unsubscribing cost no allocation after the first.
struct BeeBusSubSlot {
    BeeBusScalar lastSent;
    bb_time_t lastSentAt = 0;
    uint16_t next = BEEBUS_TOPIC_NONE;
    uint8_t nodeID = 0;
    uint8_t contractId = BEEBUS_SUB_NONE;
    uint8_t seq = 0;          // This destination's stream, so a gap means a loss
    uint8_t streamSeq = 0;    // RELIABLE: the stream number the last value left under
    uint8_t flags = 0;
    uint16_t textSent = 0;    // A string topic: the textVersion this destination has
};

// The destination asked to be published to, rather than merely having been.
#define BEEBUS_SUB_F_SUBSCRIBED    0x01

// Something has been sent, so lastSent and lastSentAt mean something.
#define BEEBUS_SUB_F_SENT          0x02

// The destination subscribed under QoS RELIABLE: every live value it is sent
// goes on this node's stream to it, numbered in `streamSeq`. `seq` keeps the
// topic's own count for what is not streamed (a buffered flush, a read reply).
#define BEEBUS_SUB_F_RELIABLE      0x04

// A RELIABLE value has gone and its acknowledgement has not come back.
#define BEEBUS_SUB_F_AWAIT         0x08

// Subscription terms, interned: subscribers wanting the same deadband and
// interval share one record.
//
// `used` says whether the slot holds anything. Without it the table only ever
// grew: a peer that subscribed 255 times with slightly different deadbands, or
// an application computing a deadband from a float reading, filled all 255
// slots permanently, and every later subscription from anyone was refused with
// no room even after the node responsible had left.
struct BeeBusContract {
    float deadband = 0.0f;
    bb_time_t maxInterval = 0;    // Milliseconds; 0 is report by exception
    bool used = false;
};

// A sample held while the network was unreachable. The sequence is stamped when
// it goes out, from the destination's own stream.
struct BeeBusTopicSample {
    BeeBusScalar value;
    bb_time_t at = 0;
};

// A topic learnt from a peer's commissioning list. Only nodes that were asked
// appear here. The name is held at its own length, owned by the map; null
// until a fetch with names has carried it.
struct BeeBusDiscoveredTopic {
    uint8_t type = 0;
    uint8_t flags = 0;
    char* name = nullptr;
};

// A topic and the destination it is due to be published to.
struct BeeBusDuePublish {
    uint16_t topicIndex = 0;
    uint16_t slotIndex = 0;
};

class BeeBusTopicMap {
public:
    BeeBusTopicMap();
    ~BeeBusTopicMap();
    BeeBusTopicMap(BeeBusTopicMap&&) = default;
    BeeBusTopicMap& operator=(BeeBusTopicMap&&) = default;

    void clearAll();

    // --- Local topics ---
    //
    // False if the type is not one the wire carries, or the ID is out of range
    // or already bound. A string topic binds a char buffer of textCap bytes.
    bool addLocal(uint16_t id, uint8_t type, void* variable, bool writable,
                  const char* name, uint16_t textCap = 0);
    bool removeLocal(uint16_t id);

    uint16_t localIndex(uint16_t id) const;
    size_t localCount() const { return _local.size(); }
    BeeBusLocalTopic& localAt(size_t index) { return _local[index]; }
    const BeeBusLocalTopic& localAt(size_t index) const { return _local[index]; }

    // Reserves room up front; each insert is otherwise a move of everything
    // above it.
    void reserveLocal(size_t count);

    // --- Remote topics ---
    bool addRemote(uint8_t nodeID, uint16_t id, uint8_t type, void* variable,
                   const char* name, uint16_t textCap = 0);
    bool removeRemote(uint8_t nodeID, uint16_t id);
    void removeNode(uint8_t nodeID);

    uint16_t remoteIndex(uint8_t nodeID, uint16_t id) const;
    size_t remoteCount() const { return _remote.size(); }
    BeeBusRemoteTopic& remoteAt(size_t index) { return _remote[index]; }
    const BeeBusRemoteTopic& remoteAt(size_t index) const { return _remote[index]; }
    void reserveRemote(size_t count);

    // --- Names ---
#if BEEBUS_TOPIC_NAMES
    // The topic ID a name refers to, or BEEBUS_TOPIC_NONE. A remote name is
    // looked up among this node's own bindings first, then among what a fetch
    // discovered.
    uint16_t localByName(const char* name) const;
    uint16_t remoteByName(uint8_t nodeID, const char* name) const;

    // --- Discovered topics ---
    // False when the memory for the entry or its name is not there.
    bool noteDiscovered(uint8_t nodeID, const BeeBusTopicEntry& entry);
    void clearDiscovered(uint8_t nodeID);
    size_t discoveredCount() const { return _discovered.size(); }
    // The i-th discovered topic, in (node, ID) order; `key` is (nodeID << 16) | ID.
    const BeeBusDiscoveredTopic& discoveredAt(size_t i, uint32_t& key) const {
        key = _discovered[i].key;
        return _discovered[i].topic;
    }
    const BeeBusDiscoveredTopic* findDiscovered(uint8_t nodeID, uint16_t id) const;
#endif

    // --- Subscribers, on the publishing node ---
    //
    // A slot exists per (topic, destination) ever published to, subscribed or
    // not, because the sequence counter belongs to that stream. Subscribing
    // sets a flag on the slot.
    uint16_t slotFor(uint16_t topicIndex, uint8_t nodeID, bool create);
    BeeBusSubSlot& slotAt(uint16_t slotIndex) { return _slots[slotIndex]; }
    const BeeBusSubSlot& slotAt(uint16_t slotIndex) const { return _slots[slotIndex]; }

    // The subscribed nodes, copied out (at most `max`): a reference into the
    // arena would be invalidated by a subscription arriving during the walk.
    size_t subscribersOf(uint16_t topicIndex, uint8_t* nodes, size_t max) const;

    // Records that every subscriber of a topic has had `value`, because a
    // broadcast reached all of them at once. Without it each subscriber slot
    // still reads as due and the scan publishes the same value again, once per
    // subscriber, behind the broadcast that already delivered it.
    void noteAllSlotsSent(uint16_t topicIndex, const BeeBusScalar& value, bb_time_t now);

    bool subscribeSlot(uint16_t topicIndex, uint8_t nodeID, uint8_t contractId,
                       bool reliable = false);
    bool unsubscribeSlot(uint16_t topicIndex, uint8_t nodeID);
    void dropNodeSlots(uint8_t nodeID);
    uint16_t subscriberCount(uint16_t topicIndex) const;

    // --- RELIABLE streams, on the publishing node ---
    //
    // A RELIABLE value left for `nodeID` under stream number `seq`.
    void noteStreamSent(uint16_t slotIndex, uint8_t seq);

    // The destination has everything up to `highest` except the numbers
    // `missing` marks (bit i = highest - 1 - i). What arrived is no longer
    // awaited; what did not is due again, and goes as the current value.
    // Returns how many topics were made due again.
    uint16_t streamAcked(uint8_t nodeID, uint8_t highest, uint64_t missing);

    // Every value still awaited by `nodeID` is due again: its acknowledgements
    // stopped. Returns how many.
    uint16_t streamResend(uint8_t nodeID);

    // Whether any local topic has a subscriber, so the scan can be skipped.
    bool hasSubscribers() const { return _subscribedTopics > 0; }

    // --- String topics ---
    //
    // A topic is a string when its bound type is BEEBUS_T_STRING, so nothing
    // extra records which ones are; these only reach the bound variable and
    // carry the text change detection needs.
    //
    // Null when the index is out of range or the topic is not a string.
    const char* localString(uint16_t topicIndex) const;
    const char* remoteString(uint16_t remoteIndex) const;

    // Writes a peer's value into a bound string, local or remote: cut to the
    // buffer and always terminated.
    bool setLocalString(uint16_t topicIndex, const char* text, uint16_t length);
    bool setRemoteString(uint16_t remoteIndex, const char* text, uint16_t length);

    // Records the bound text into the ring and marks it seen, so an explicit
    // publish of a buffered string topic is not recorded a second time by the
    // next scan. False when the ring overwrote its oldest.
    bool captureStringSample(uint16_t topicIndex, bb_time_t at);

    // Records what a destination was last sent, so the next scan can tell
    // whether the text moved. Called once the publish has been accepted.
    void noteStringSent(uint16_t topicID, uint16_t slotIndex, const char* text);

    // --- Contracts ---
    uint8_t internContract(float deadband, bb_time_t maxInterval);
    const BeeBusContract& contract(uint8_t id) const;

    // Contracts nothing refers to any more. Called when a subscription ends, so
    // the table is a cache of what is in use rather than a log of everything
    // ever asked for.
    void releaseUnusedContracts();

    // How many of the 255 slots are taken, for tests and diagnostics.
    uint8_t contractsInUse() const;

    // --- Publish scheduling ---
    //
    // Advances the cursor over a bounded slice of the local topics and reports
    // every (topic, destination) pair due: the value moved beyond the deadband,
    // or the maximum interval elapsed. The list is the map's own, and holds a
    // pair per subscriber slot at most, so filling it never allocates.
    void scanDue(bb_time_t now);
    size_t dueCount() const { return _due.size(); }
    const BeeBusDuePublish& dueAt(size_t i) const { return _due[i]; }

    // --- Store and forward ---
    bool setBuffer(uint16_t id, uint8_t depth);
    uint8_t bufferDepth(uint16_t id) const;

    // Appends a sample, overwriting the oldest when full. False when something
    // was overwritten.
    bool pushSample(uint16_t id, const BeeBusScalar& value, bb_time_t at);
    bool pushStringSample(uint16_t id, const char* text, bb_time_t at);

    // The oldest sample, and separately its retirement: one sample goes to
    // every subscriber and may only be dropped once they have all had it.
    bool peekSample(uint16_t id, BeeBusTopicSample& out) const;
    // `text` points into the ring and holds until the sample is dropped.
    bool peekStringSample(uint16_t id, BeeBusTopicSample& out, const char*& text) const;
    void dropSample(uint16_t id);
    uint16_t bufferedCount(uint16_t id) const;

    // Samples overwritten and not yet declared, and their retirement.
    uint16_t droppedCount(uint16_t id) const;
    void clearDropped(uint16_t id);
    uint16_t bufferedTotal() const;

    // Topic IDs with a ring, in order, so a drain need not scan the map.
    size_t bufferedTopicCount() const { return _rings.size(); }
    uint16_t bufferedTopicAt(size_t i) const { return _rings[i].id; }

    // --- Footprint ---
    // Bytes held by the lists, rings and texts.
    size_t heapBytes() const;

private:
    BeeBusList<BeeBusLocalTopic> _local;       // Sorted by id
    BeeBusList<BeeBusRemoteTopic> _remote;     // Sorted by key
    BeeBusList<BeeBusSubSlot> _slots;          // Arena
    BeeBusList<BeeBusContract> _contracts;     // Interned

    uint16_t _slotFree;          // Free-list head
    uint16_t _scanCursor;
    uint16_t _subscribedTopics;  // Local topics with at least one subscriber

    // A store-and-forward ring, sized once when the topic opts in so a push
    // never allocates. Plain data in a table sorted by topic ID; the samples
    // (and a string topic's texts, textCap bytes each) are owned blocks.
    struct Ring {
        uint16_t id;
        // Samples overwritten since the last flush. Spent as sequence numbers
        // on the next send, so the loss reaches the receiver as a gap.
        uint16_t dropped;
        uint16_t textCap;
        uint8_t head;            // Oldest sample
        uint8_t count;
        uint8_t depth;
        BeeBusTopicSample* samples;
        char* texts;
    };
    BeeBusAddrTable<Ring> _rings;
    static void freeRing(Ring& ring);

    // Filled by scanDue(); room for one entry per subscriber slot.
    BeeBusList<BeeBusDuePublish> _due;

#if BEEBUS_TOPIC_NAMES
    struct Discovered {
        uint32_t key;
        BeeBusDiscoveredTopic topic;
    };
    BeeBusList<Discovered> _discovered;      // Sorted by key
    size_t discoveredLowerBound(uint32_t key) const;
#endif

    static uint32_t remoteKey(uint8_t nodeID, uint16_t id) {
        return ((uint32_t)nodeID << 16) | id;
    }

    // Doubling up to BEEBUS_TOPIC_GROWTH_STEP entries, then a fixed step.
    template <typename T>
    static bool growFor(BeeBusList<T>& list, size_t needed);

    uint16_t allocSlot();
    void freeSlot(uint16_t slotIndex);
    void detachSlot(BeeBusLocalTopic& topic, uint16_t slotIndex);

    // Whether this destination is owed a publish now.
    bool slotIsDue(const BeeBusLocalTopic& topic, const BeeBusSubSlot& slot,
                   const BeeBusScalar& current, bb_time_t now) const;

    // The same for a string topic, which has no deadband to judge by: a text
    // version it has not had is a different message.
    bool stringSlotIsDue(const BeeBusLocalTopic& topic, const BeeBusSubSlot& slot,
                         bb_time_t now) const;

    // One topic's worth of the scan, for a string topic.
    void scanStringTopic(uint16_t index, bb_time_t now);

    // The scan's text check: the bound text against lastText, copied and the
    // version moved on when it differs. True when it changed.
    static bool takeText(BeeBusLocalTopic& topic);

    // Where the next sample goes, advancing the ring. Sets `overwrote` when the
    // oldest was displaced. Shared by the scalar and string pushes.
    static uint8_t ringPushSlot(Ring& ring, bool& overwrote);
};
