#include "BeeBusTopicMap.h"
#include "BeeBusNoExcept.h"

namespace {

// Returns where the entry is, or where it would go.
template <typename T, typename Key, typename KeyOf>
size_t lowerBound(const BeeBusList<T>& list, Key key, KeyOf keyOf) {
    size_t low = 0;
    size_t high = list.size();
    while (low < high) {
        const size_t mid = low + (high - low) / 2;
        if (keyOf(list[mid]) < key) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return low;
}

uint16_t localKeyOf(const BeeBusLocalTopic& topic) { return topic.id; }
uint32_t remoteKeyOf(const BeeBusRemoteTopic& topic) { return topic.key; }

// Copies text into a buffer of `capacity` bytes, cut short and terminated.
void copyText(char* out, uint16_t capacity, const char* text, size_t length) {
    if (!out || capacity == 0) return;
    if (length > (size_t)(capacity - 1)) length = (size_t)(capacity - 1);
    if (length) memcpy(out, text, length);
    memset(out + length, 0, (size_t)(capacity - length));
}

} // namespace

template <typename T>
bool BeeBusTopicMap::growFor(BeeBusList<T>& list, size_t needed) {
    if (list.capacity() >= needed) return true;
    size_t next = list.capacity() ? list.capacity() : 8;
    while (next < needed) {
        next = next < BEEBUS_TOPIC_GROWTH_STEP ? next * 2
                                               : next + BEEBUS_TOPIC_GROWTH_STEP;
    }
    return list.grow(next);
}

BeeBusTopicMap::BeeBusTopicMap()
    : _slotFree(BEEBUS_TOPIC_NONE),
      _scanCursor(0),
      _subscribedTopics(0) {
}

BeeBusTopicMap::~BeeBusTopicMap() {
    clearAll();
}

void BeeBusTopicMap::freeRing(Ring& ring) {
    delete[] ring.samples;
    ::operator delete(ring.texts);
    ring.samples = nullptr;
    ring.texts = nullptr;
}

void BeeBusTopicMap::clearAll() {
    for (size_t i = 0; i < _local.size(); i++) {
        ::operator delete(_local[i].lastText);
    }
    for (size_t i = 0; i < _rings.size(); i++) {
        freeRing(_rings[i]);
    }
    _local.clear();
    _remote.clear();
    _slots.clear();
    _contracts.clear();
    _rings.clear();
    _due.clear();
    _slotFree = BEEBUS_TOPIC_NONE;
    _scanCursor = 0;
    _subscribedTopics = 0;
#if BEEBUS_TOPIC_NAMES
    for (size_t i = 0; i < _discovered.size(); i++) delete[] _discovered[i].topic.name;
    _discovered.clear();
#endif
}

// --- Local topics ---

bool BeeBusTopicMap::addLocal(uint16_t id, uint8_t type, void* variable, bool writable,
                              const char* name, uint16_t textCap) {
    if (id < BEEBUS_TOPIC_ID_MIN || id > BEEBUS_TOPIC_ID_MAX ||
        !bb_topicTypeIsValid(type)) {
        return false;
    }
    const bool isText = bb_topicTypeIsString(type);
    if (isText && (!variable || textCap < 1)) {
        return false;
    }
    // BEEBUS_TOPIC_NONE is the index sentinel, so it cannot also be an index.
    if (_local.size() >= BEEBUS_TOPIC_NONE) {
        return false;
    }
    const size_t at = lowerBound(_local, id, localKeyOf);
    if (at < _local.size() && _local[at].id == id) {
        return false;
    }

#if BEEBUS_TOPIC_NAMES
    // Longer than the wire's length byte can say: never sent cut short.
    if (name && strnlen(name, BEEBUS_TOPIC_NAME_MAX) >= BEEBUS_TOPIC_NAME_MAX) {
        return false;
    }
#endif

    BeeBusLocalTopic topic;
    topic.id = id;
    topic.var = variable;
    topic.last = bb_scalarLoad(type, variable);
    topic.flags = (uint8_t)(writable ? BEEBUS_TOPIC_F_WRITABLE : 0);
#if BEEBUS_TOPIC_NAMES
    topic.name = name;
#else
    (void)name;
#endif

    if (isText) {
        // Seeded with what is bound now, exactly as topic.last is for a scalar,
        // so the first scan does not report a change that never happened.
        topic.textCap = textCap;
        topic.lastText = static_cast<char*>(::operator new(textCap, std::nothrow));
        if (!topic.lastText) return false;
        const char* bound = (const char*)variable;
        copyText(topic.lastText, textCap, bound, strnlen(bound, textCap));
    }

    if (!growFor(_local, _local.size() + 1) || !_local.insertAt(at, topic)) {
        ::operator delete(topic.lastText);
        return false;
    }
    return true;
}

bool BeeBusTopicMap::removeLocal(uint16_t id) {
    const uint16_t index = localIndex(id);
    if (index == BEEBUS_TOPIC_NONE) {
        return false;
    }

    BeeBusLocalTopic& topic = _local[index];
    if (subscriberCount(index) > 0) {
        _subscribedTopics--;
    }
    uint16_t slot = topic.subHead;
    while (slot != BEEBUS_TOPIC_NONE) {
        const uint16_t next = _slots[slot].next;
        freeSlot(slot);
        slot = next;
    }

    setBuffer(id, 0);
    ::operator delete(_local[index].lastText);
    _local.eraseAt(index);

    // The cursor indexes the list, so an erase below it would skip a topic.
    _scanCursor = 0;
    return true;
}

uint16_t BeeBusTopicMap::localIndex(uint16_t id) const {
    const size_t at = lowerBound(_local, id, localKeyOf);
    if (at < _local.size() && _local[at].id == id) {
        return (uint16_t)at;
    }
    return BEEBUS_TOPIC_NONE;
}

void BeeBusTopicMap::reserveLocal(size_t count) {
    growFor(_local, count);
}

// --- Remote topics ---

bool BeeBusTopicMap::addRemote(uint8_t nodeID, uint16_t id, uint8_t type, void* variable,
                               const char* name, uint16_t textCap) {
    if (nodeID == 0 || id < BEEBUS_TOPIC_ID_MIN || id > BEEBUS_TOPIC_ID_MAX ||
        !bb_topicTypeIsValid(type)) {
        return false;
    }
    if (bb_topicTypeIsString(type) && variable && textCap < 1) {
        return false;
    }
    if (_remote.size() >= BEEBUS_TOPIC_NONE) {
        return false;
    }
    const uint32_t key = remoteKey(nodeID, id);
    const size_t at = lowerBound(_remote, key, remoteKeyOf);
    if (at < _remote.size() && _remote[at].key == key) {
        return false;
    }

#if BEEBUS_TOPIC_NAMES
    if (name && strnlen(name, BEEBUS_TOPIC_NAME_MAX) >= BEEBUS_TOPIC_NAME_MAX) {
        return false;
    }
#endif

    BeeBusRemoteTopic topic;
    topic.key = key;
    topic.var = variable;
    topic.type = (uint8_t)(type & BEEBUS_T_MASK);
    topic.textCap = textCap;
#if BEEBUS_TOPIC_NAMES
    topic.name = name;
#else
    (void)name;
#endif
    return growFor(_remote, _remote.size() + 1) && _remote.insertAt(at, topic);
}

bool BeeBusTopicMap::removeRemote(uint8_t nodeID, uint16_t id) {
    const uint16_t index = remoteIndex(nodeID, id);
    if (index == BEEBUS_TOPIC_NONE) {
        return false;
    }
    _remote.eraseAt(index);
    return true;
}

uint16_t BeeBusTopicMap::remoteIndex(uint8_t nodeID, uint16_t id) const {
    const uint32_t key = remoteKey(nodeID, id);
    const size_t at = lowerBound(_remote, key, remoteKeyOf);
    if (at < _remote.size() && _remote[at].key == key) {
        return (uint16_t)at;
    }
    return BEEBUS_TOPIC_NONE;
}

void BeeBusTopicMap::reserveRemote(size_t count) {
    growFor(_remote, count);
}

void BeeBusTopicMap::removeNode(uint8_t nodeID) {
    dropNodeSlots(nodeID);
    for (size_t i = 0; i < _remote.size(); i++) {
        if ((uint8_t)(_remote[i].key >> 16) != nodeID) continue;
        // What the node told us, not what this application bound. The bindings
        // stay; their quality reports that the values cannot be acted on.
        _remote[i].flags = (uint8_t)(_remote[i].flags &
                                     ~(BEEBUS_REMOTE_F_SUBSCRIBED | BEEBUS_REMOTE_F_SEQ));
    }
#if BEEBUS_TOPIC_NAMES
    clearDiscovered(nodeID);
#endif
}

// --- Names ---
#if BEEBUS_TOPIC_NAMES

bool bb_globMatch(const char* pattern, const char* text) {
    if (!pattern || !text) return false;
    // Iterative with one backtrack point, so several stars cannot cost
    // exponential time.
    const char* star = nullptr;
    const char* resume = nullptr;
    while (*text) {
        if (*pattern == '?' || *pattern == *text) {
            pattern++;
            text++;
        } else if (*pattern == '*') {
            star = pattern++;
            resume = text;
        } else if (star) {
            pattern = star + 1;
            text = ++resume;
        } else {
            return false;
        }
    }
    while (*pattern == '*') pattern++;
    return *pattern == '\0';
}

uint16_t BeeBusTopicMap::localByName(const char* name) const {
    if (!name) return BEEBUS_TOPIC_NONE;
    for (size_t i = 0; i < _local.size(); i++) {
        if (_local[i].name && strcmp(_local[i].name, name) == 0) {
            return _local[i].id;
        }
    }
    return BEEBUS_TOPIC_NONE;
}

size_t BeeBusTopicMap::discoveredLowerBound(uint32_t key) const {
    size_t low = 0;
    size_t high = _discovered.size();
    while (low < high) {
        const size_t mid = low + (high - low) / 2;
        if (_discovered[mid].key < key) low = mid + 1; else high = mid;
    }
    return low;
}

uint16_t BeeBusTopicMap::remoteByName(uint8_t nodeID, const char* name) const {
    if (!name) return BEEBUS_TOPIC_NONE;
    // Bindings made by hand first, then what a fetch discovered.
    for (size_t i = 0; i < _remote.size(); i++) {
        if ((uint8_t)(_remote[i].key >> 16) != nodeID) continue;
        if (_remote[i].name && strcmp(_remote[i].name, name) == 0) {
            return (uint16_t)(_remote[i].key & 0xFFFF);
        }
    }
    const uint32_t high = remoteKey(nodeID, BEEBUS_TOPIC_ID_MAX);
    for (size_t i = discoveredLowerBound(remoteKey(nodeID, 0));
         i < _discovered.size() && _discovered[i].key <= high; i++) {
        if (_discovered[i].topic.name && strcmp(_discovered[i].topic.name, name) == 0) {
            return (uint16_t)(_discovered[i].key & 0xFFFF);
        }
    }
    return BEEBUS_TOPIC_NONE;
}

bool BeeBusTopicMap::noteDiscovered(uint8_t nodeID, const BeeBusTopicEntry& entry) {
    if (nodeID == 0 || entry.topicID > BEEBUS_TOPIC_ID_MAX) return true;
    const uint32_t key = remoteKey(nodeID, entry.topicID);
    size_t at = discoveredLowerBound(key);
    if (at >= _discovered.size() || _discovered[at].key != key) {
        Discovered fresh;
        fresh.key = key;
        if (!growFor(_discovered, _discovered.size() + 1) || !_discovered.insertAt(at, fresh)) {
            return false;
        }
    }
    BeeBusDiscoveredTopic& discovered = _discovered[at].topic;
    discovered.type = entry.type;
    discovered.flags = entry.flags;
    // A fetch for IDs and types alone carries no names, and must not erase the
    // ones an earlier fetch learnt. A name is held at its own length.
    if (entry.name && entry.name[0] != '\0' &&
        !(discovered.name && strcmp(discovered.name, entry.name) == 0)) {
        const size_t length = strlen(entry.name);
        char* copy = new (std::nothrow) char[length + 1];
        if (!copy) return false;
        memcpy(copy, entry.name, length + 1);
        delete[] discovered.name;
        discovered.name = copy;
    }
    return true;
}

void BeeBusTopicMap::clearDiscovered(uint8_t nodeID) {
    const uint32_t high = remoteKey(nodeID, BEEBUS_TOPIC_ID_MAX);
    const size_t at = discoveredLowerBound(remoteKey(nodeID, 0));
    while (at < _discovered.size() && _discovered[at].key <= high) {
        delete[] _discovered[at].topic.name;
        _discovered.eraseAt(at);
    }
}

const BeeBusDiscoveredTopic* BeeBusTopicMap::findDiscovered(uint8_t nodeID, uint16_t id) const {
    const uint32_t key = remoteKey(nodeID, id);
    const size_t at = discoveredLowerBound(key);
    if (at < _discovered.size() && _discovered[at].key == key) {
        return &_discovered[at].topic;
    }
    return nullptr;
}

#endif

// --- Subscriber slots ---

uint16_t BeeBusTopicMap::allocSlot() {
    if (_slotFree != BEEBUS_TOPIC_NONE) {
        const uint16_t index = _slotFree;
        _slotFree = _slots[index].next;
        _slots[index] = BeeBusSubSlot();
        return index;
    }
    if (_slots.size() >= BEEBUS_TOPIC_NONE) {
        return BEEBUS_TOPIC_NONE;
    }
    // A new subscriber needs a slot, and the scan room to report it due.
    if (!growFor(_slots, _slots.size() + 1) || !growFor(_due, _slots.size() + 1)) {
        return BEEBUS_TOPIC_NONE;
    }
    _slots.push_back(BeeBusSubSlot());
    return (uint16_t)(_slots.size() - 1);
}

void BeeBusTopicMap::freeSlot(uint16_t slotIndex) {
    _slots[slotIndex] = BeeBusSubSlot();
    _slots[slotIndex].next = _slotFree;
    _slotFree = slotIndex;
}

void BeeBusTopicMap::detachSlot(BeeBusLocalTopic& topic, uint16_t slotIndex) {
    uint16_t prev = BEEBUS_TOPIC_NONE;
    uint16_t at = topic.subHead;
    while (at != BEEBUS_TOPIC_NONE && at != slotIndex) {
        prev = at;
        at = _slots[at].next;
    }
    if (at == BEEBUS_TOPIC_NONE) return;

    if (prev == BEEBUS_TOPIC_NONE) {
        topic.subHead = _slots[at].next;
    } else {
        _slots[prev].next = _slots[at].next;
    }
    topic.subCount--;

    // The index goes back on the free list and may be handed to another topic;
    // freeSlot() clears what it held, the text version included.
    freeSlot(slotIndex);
}

uint16_t BeeBusTopicMap::slotFor(uint16_t topicIndex, uint8_t nodeID, bool create) {
    if (topicIndex >= _local.size()) return BEEBUS_TOPIC_NONE;

    for (uint16_t at = _local[topicIndex].subHead; at != BEEBUS_TOPIC_NONE;
         at = _slots[at].next) {
        if (_slots[at].nodeID == nodeID) return at;
    }
    if (!create) return BEEBUS_TOPIC_NONE;

    // At most one destination per addressable node.
    if (_local[topicIndex].subCount >= BEEBUS_NODE_ID_MAX) return BEEBUS_TOPIC_NONE;

    const uint16_t slot = allocSlot();
    if (slot == BEEBUS_TOPIC_NONE) return BEEBUS_TOPIC_NONE;

    BeeBusLocalTopic& topic = _local[topicIndex];
    _slots[slot].nodeID = nodeID;
    _slots[slot].next = topic.subHead;
    topic.subHead = slot;
    topic.subCount++;
    return slot;
}

size_t BeeBusTopicMap::subscribersOf(uint16_t topicIndex, uint8_t* nodes, size_t max) const {
    size_t count = 0;
    if (topicIndex >= _local.size()) return 0;
    for (uint16_t at = _local[topicIndex].subHead; at != BEEBUS_TOPIC_NONE && count < max;
         at = _slots[at].next) {
        if (_slots[at].flags & BEEBUS_SUB_F_SUBSCRIBED) nodes[count++] = _slots[at].nodeID;
    }
    return count;
}

void BeeBusTopicMap::noteAllSlotsSent(uint16_t topicIndex, const BeeBusScalar& value,
                                      bb_time_t now) {
    if (topicIndex >= _local.size()) return;
    for (uint16_t at = _local[topicIndex].subHead; at != BEEBUS_TOPIC_NONE;
         at = _slots[at].next) {
        // A RELIABLE subscriber is owed its value on its own stream, where it
        // is acknowledged: a broadcast is not.
        if (_slots[at].flags & BEEBUS_SUB_F_RELIABLE) continue;
        _slots[at].lastSent = value;
        _slots[at].lastSentAt = now;
        _slots[at].flags = (uint8_t)(_slots[at].flags | BEEBUS_SUB_F_SENT);
    }
}

bool BeeBusTopicMap::subscribeSlot(uint16_t topicIndex, uint8_t nodeID, uint8_t contractId,
                                   bool reliable) {
    const uint16_t slot = slotFor(topicIndex, nodeID, true);
    if (slot == BEEBUS_TOPIC_NONE) return false;
    const bool wasSubscribed = subscriberCount(topicIndex) > 0;
    _slots[slot].flags = (uint8_t)(_slots[slot].flags | BEEBUS_SUB_F_SUBSCRIBED);
    _slots[slot].contractId = contractId;
    // Owed its first report immediately, whatever the deadband. Nothing is
    // awaited any more either: the subscription starts the stream afresh.
    _slots[slot].flags = (uint8_t)(_slots[slot].flags &
                                   ~(BEEBUS_SUB_F_SENT | BEEBUS_SUB_F_AWAIT | BEEBUS_SUB_F_RELIABLE));
    if (reliable) {
        _slots[slot].flags = (uint8_t)(_slots[slot].flags | BEEBUS_SUB_F_RELIABLE);
    }
    if (!wasSubscribed) _subscribedTopics++;
    return true;
}

void BeeBusTopicMap::noteStreamSent(uint16_t slotIndex, uint8_t seq) {
    if (slotIndex >= _slots.size()) return;
    _slots[slotIndex].streamSeq = seq;
    _slots[slotIndex].flags = (uint8_t)(_slots[slotIndex].flags | BEEBUS_SUB_F_AWAIT);
}

uint16_t BeeBusTopicMap::streamAcked(uint8_t nodeID, uint8_t highest, uint64_t missing) {
    uint16_t due = 0;
    for (size_t i = 0; i < _local.size(); i++) {
        const uint16_t at = slotFor((uint16_t)i, nodeID, false);
        if (at == BEEBUS_TOPIC_NONE) continue;
        BeeBusSubSlot& slot = _slots[at];
        if ((slot.flags & BEEBUS_SUB_F_AWAIT) == 0) continue;

        // How far behind the acknowledged point this value left. Half the
        // range ahead is a value sent after the receiver wrote its answer.
        const uint8_t behind = (uint8_t)(highest - slot.streamSeq);
        if (behind >= 128) continue;

        // Within the mask the receiver says exactly; past it, it cannot, and
        // the value goes again rather than being assumed delivered.
        const bool lost = behind != 0 &&
                          (behind > 64 || ((missing >> (behind - 1)) & 1u) != 0);
        slot.flags = (uint8_t)(slot.flags & ~BEEBUS_SUB_F_AWAIT);
        if (lost) {
            slot.flags = (uint8_t)(slot.flags & ~BEEBUS_SUB_F_SENT);
            due++;
        }
    }
    return due;
}

uint16_t BeeBusTopicMap::streamResend(uint8_t nodeID) {
    uint16_t due = 0;
    for (size_t i = 0; i < _local.size(); i++) {
        const uint16_t at = slotFor((uint16_t)i, nodeID, false);
        if (at == BEEBUS_TOPIC_NONE) continue;
        BeeBusSubSlot& slot = _slots[at];
        if ((slot.flags & BEEBUS_SUB_F_AWAIT) == 0) continue;
        slot.flags = (uint8_t)(slot.flags & ~(BEEBUS_SUB_F_AWAIT | BEEBUS_SUB_F_SENT));
        due++;
    }
    return due;
}

bool BeeBusTopicMap::unsubscribeSlot(uint16_t topicIndex, uint8_t nodeID) {
    const uint16_t slot = slotFor(topicIndex, nodeID, false);
    if (slot == BEEBUS_TOPIC_NONE) return false;
    if ((_slots[slot].flags & BEEBUS_SUB_F_SUBSCRIBED) == 0) return false;
    _slots[slot].flags = (uint8_t)(_slots[slot].flags & ~BEEBUS_SUB_F_SUBSCRIBED);
    _slots[slot].contractId = BEEBUS_SUB_NONE;
    // A subscription ending is the one moment terms can become unreferenced.
    releaseUnusedContracts();
    if (subscriberCount(topicIndex) == 0) _subscribedTopics--;
    return true;
}

void BeeBusTopicMap::dropNodeSlots(uint8_t nodeID) {
    for (size_t i = 0; i < _local.size(); i++) {
        const uint16_t slot = slotFor((uint16_t)i, nodeID, false);
        if (slot == BEEBUS_TOPIC_NONE) continue;
        const bool wasSubscribed = subscriberCount((uint16_t)i) > 0;
        detachSlot(_local[i], slot);
        if (wasSubscribed && subscriberCount((uint16_t)i) == 0) _subscribedTopics--;
    }
}

uint16_t BeeBusTopicMap::subscriberCount(uint16_t topicIndex) const {
    if (topicIndex >= _local.size()) return 0;
    uint16_t count = 0;
    for (uint16_t at = _local[topicIndex].subHead; at != BEEBUS_TOPIC_NONE;
         at = _slots[at].next) {
        if (_slots[at].flags & BEEBUS_SUB_F_SUBSCRIBED) count++;
    }
    return count;
}

// --- String topics ---

const char* BeeBusTopicMap::localString(uint16_t topicIndex) const {
    if (topicIndex >= _local.size()) return nullptr;
    const BeeBusLocalTopic& topic = _local[topicIndex];
    if (!bb_topicTypeIsString(topic.last.type) || !topic.var) return nullptr;
    return (const char*)topic.var;
}

const char* BeeBusTopicMap::remoteString(uint16_t remoteIndex) const {
    if (remoteIndex >= _remote.size()) return nullptr;
    const BeeBusRemoteTopic& topic = _remote[remoteIndex];
    if (!bb_topicTypeIsString(topic.type) || !topic.var) return nullptr;
    return (const char*)topic.var;
}

bool BeeBusTopicMap::setLocalString(uint16_t topicIndex, const char* text, uint16_t length) {
    if (topicIndex >= _local.size()) return false;
    BeeBusLocalTopic& topic = _local[topicIndex];
    if (!bb_topicTypeIsString(topic.last.type) || !topic.var) return false;
    // Length-driven, not terminator-driven, so the value is whatever the sender
    // said it was, cut to the buffer.
    copyText((char*)topic.var, topic.textCap, text ? text : "", text ? length : 0);
    return true;
}

bool BeeBusTopicMap::takeText(BeeBusLocalTopic& topic) {
    const char* current = (const char*)topic.var;
    if (strncmp(current, topic.lastText, topic.textCap) == 0) {
        return false;
    }
    copyText(topic.lastText, topic.textCap, current, strnlen(current, topic.textCap));
    topic.textVersion++;
    return true;
}

bool BeeBusTopicMap::captureStringSample(uint16_t topicIndex, bb_time_t at) {
    const char* text = localString(topicIndex);
    if (!text) return true;
    BeeBusLocalTopic& topic = _local[topicIndex];
    takeText(topic);
    return pushStringSample(topic.id, topic.lastText, at);
}

bool BeeBusTopicMap::setRemoteString(uint16_t remoteIndex, const char* text, uint16_t length) {
    if (remoteIndex >= _remote.size()) return false;
    BeeBusRemoteTopic& topic = _remote[remoteIndex];
    if (!bb_topicTypeIsString(topic.type) || !topic.var) return false;
    copyText((char*)topic.var, topic.textCap, text ? text : "", text ? length : 0);
    return true;
}

void BeeBusTopicMap::noteStringSent(uint16_t topicID, uint16_t slotIndex, const char* text) {
    const uint16_t index = localIndex(topicID);
    if (index == BEEBUS_TOPIC_NONE || !bb_topicTypeIsString(_local[index].last.type) ||
        slotIndex >= _slots.size() || !text) {
        return;
    }
    BeeBusLocalTopic& topic = _local[index];
    // The text sent is the bound text: this destination has its version, which
    // the scan would otherwise move on when it next looks. Any other text (a
    // stored sample) leaves the destination owed the bound one.
    const char* bound = (const char*)topic.var;
    if (strncmp(text, bound, topic.textCap) == 0) {
        takeText(topic);
        _slots[slotIndex].textSent = topic.textVersion;
    } else {
        _slots[slotIndex].textSent = (uint16_t)(topic.textVersion + 1);
    }
}

// --- Contracts ---

uint8_t BeeBusTopicMap::internContract(float deadband, bb_time_t maxInterval) {
    const uint32_t wanted = bb_floatBits(deadband);
    size_t freeSlot = BEEBUS_SUB_NONE;
    for (size_t i = 0; i < _contracts.size(); i++) {
        if (!_contracts[i].used) {
            if (freeSlot == BEEBUS_SUB_NONE) freeSlot = i;
            continue;
        }
        if (bb_floatBits(_contracts[i].deadband) == wanted &&
            _contracts[i].maxInterval == maxInterval) {
            return (uint8_t)i;
        }
    }

    // A slot released by an ended subscription before growing the table.
    if (freeSlot == BEEBUS_SUB_NONE && _contracts.size() >= BEEBUS_SUB_NONE) {
        // Full. Sweep for terms nothing refers to any more and try once more,
        // so a table filled by subscriptions that have since ended recovers
        // instead of refusing everything from here on.
        releaseUnusedContracts();
        for (size_t i = 0; i < _contracts.size(); i++) {
            if (!_contracts[i].used) { freeSlot = i; break; }
        }
        if (freeSlot == BEEBUS_SUB_NONE) {
            return BEEBUS_SUB_NONE;
        }
    }

    BeeBusContract contract;
    contract.deadband = deadband;
    contract.maxInterval = maxInterval;
    contract.used = true;
    if (freeSlot != BEEBUS_SUB_NONE) {
        _contracts[freeSlot] = contract;
        return (uint8_t)freeSlot;
    }
    if (!growFor(_contracts, _contracts.size() + 1) || !_contracts.push_back(contract)) {
        return BEEBUS_SUB_NONE;
    }
    return (uint8_t)(_contracts.size() - 1);
}

const BeeBusContract& BeeBusTopicMap::contract(uint8_t id) const {
    // An unset contract reports by exception with no deadband.
    static const BeeBusContract none;
    if (id >= _contracts.size() || !_contracts[id].used) return none;
    return _contracts[id];
}

uint8_t BeeBusTopicMap::contractsInUse() const {
    uint8_t count = 0;
    for (size_t i = 0; i < _contracts.size(); i++) {
        if (_contracts[i].used) count++;
    }
    return count;
}

void BeeBusTopicMap::releaseUnusedContracts() {
    // Swept rather than reference counted. A count has to be right at every
    // one of the several places a subscription can end -- a slot detached, a
    // node dropped, a topic removed -- and a count that drifts either leaks the
    // slot or frees terms still in use. The table is 255 entries and this runs
    // only when a subscription ends, so walking it is the cheaper correctness.
    bool referenced[BEEBUS_SUB_NONE];
    for (size_t i = 0; i < BEEBUS_SUB_NONE; i++) referenced[i] = false;
    for (size_t i = 0; i < _slots.size(); i++) {
        const uint8_t id = _slots[i].contractId;
        if (id < BEEBUS_SUB_NONE) referenced[id] = true;
    }
    for (size_t i = 0; i < _remote.size(); i++) {
        const uint8_t id = _remote[i].contractId;
        if (id < BEEBUS_SUB_NONE) referenced[id] = true;
    }
    for (size_t i = 0; i < _contracts.size() && i < BEEBUS_SUB_NONE; i++) {
        if (!referenced[i]) _contracts[i].used = false;
    }
}

// --- Publish scheduling ---

bool BeeBusTopicMap::slotIsDue(const BeeBusLocalTopic& topic, const BeeBusSubSlot& slot,
                               const BeeBusScalar& current, bb_time_t now) const {
    (void)topic;
    if ((slot.flags & BEEBUS_SUB_F_SUBSCRIBED) == 0) return false;
    if ((slot.flags & BEEBUS_SUB_F_SENT) == 0) return true;

    const BeeBusContract& terms = contract(slot.contractId);
    if (terms.maxInterval != 0 && bb_elapsed(slot.lastSentAt, now) >= terms.maxInterval) {
        return true;
    }
    if (bb_scalarEqual(slot.lastSent, current)) return false;
    if (terms.deadband <= 0.0f) return true;

    return bb_scalarDistance(current, slot.lastSent) >= terms.deadband;
}

bool BeeBusTopicMap::stringSlotIsDue(const BeeBusLocalTopic& topic, const BeeBusSubSlot& slot,
                                     bb_time_t now) const {
    if ((slot.flags & BEEBUS_SUB_F_SUBSCRIBED) == 0) return false;
    if ((slot.flags & BEEBUS_SUB_F_SENT) == 0) return true;

    const BeeBusContract& terms = contract(slot.contractId);
    if (terms.maxInterval != 0 && bb_elapsed(slot.lastSentAt, now) >= terms.maxInterval) {
        return true;
    }
    // No deadband for text: a message that differs at all is a different
    // message. The scan compares the text itself, never a digest, so two fault
    // messages can never be mistaken for one another; a destination is owed
    // every version it has not had.
    return slot.textSent != topic.textVersion;
}

void BeeBusTopicMap::scanStringTopic(uint16_t index, bb_time_t now) {
    BeeBusLocalTopic& topic = _local[index];
    if (!topic.var || !topic.lastText) return;

    const bool changed = takeText(topic);

    // A buffered topic records every change into its ring and is published from
    // there, network up or down, exactly as a scalar one is.
    if (topic.bufferDepth != 0) {
        if (changed) {
            pushStringSample(topic.id, topic.lastText, now);
        }
        return;
    }

    for (uint16_t at = topic.subHead; at != BEEBUS_TOPIC_NONE; at = _slots[at].next) {
        if (stringSlotIsDue(topic, _slots[at], now)) {
            BeeBusDuePublish entry;
            entry.topicIndex = index;
            entry.slotIndex = at;
            _due.push_back(entry);
        }
    }
}

void BeeBusTopicMap::scanDue(bb_time_t now) {
    _due.clear();
    if (_local.empty()) return;

    size_t examine = _local.size() < BEEBUS_TOPIC_SCAN_PER_LOOP
                         ? _local.size()
                         : (size_t)BEEBUS_TOPIC_SCAN_PER_LOOP;
    while (examine-- > 0) {
        if (_scanCursor >= _local.size()) _scanCursor = 0;
        const uint16_t index = _scanCursor++;

        if (bb_topicTypeIsString(_local[index].last.type)) {
            scanStringTopic(index, now);
            continue;
        }

        BeeBusLocalTopic& topic = _local[index];
        const BeeBusScalar current = bb_scalarLoad(topic.last.type, topic.var);

        if (topic.bufferDepth != 0) {
            // A buffered topic records every change into its ring and is
            // published from there. One path, network up or down: a live
            // publish jumping the queue would arrive ahead of the history
            // behind it.
            if (!bb_scalarEqual(topic.last, current)) {
                topic.last = current;
                pushSample(topic.id, current, now);
            }
            continue;
        }

        topic.last = current;
        for (uint16_t at = topic.subHead; at != BEEBUS_TOPIC_NONE; at = _slots[at].next) {
            if (slotIsDue(topic, _slots[at], current, now)) {
                BeeBusDuePublish entry;
                entry.topicIndex = index;
                entry.slotIndex = at;
                _due.push_back(entry);
            }
        }
    }
}

// --- Store and forward ---

bool BeeBusTopicMap::setBuffer(uint16_t id, uint8_t depth) {
    const uint16_t index = localIndex(id);
    if (index == BEEBUS_TOPIC_NONE || depth > BEEBUS_TOPIC_BUFFER_MAX) {
        return false;
    }

    if (depth == 0) {
        const size_t at = _rings.indexOf(id);
        if (at < _rings.size()) {
            freeRing(_rings[at]);
            _rings.eraseAt(at);
        }
        _local[index].bufferDepth = 0;
        _local[index].flags = (uint8_t)(_local[index].flags & ~BEEBUS_TOPIC_F_BUFFERED);
        return true;
    }

    // Sized once, so a later push allocates nothing. The texts are sized only
    // for a string topic, so a scalar ring stays 12 bytes a sample.
    const bool isText = bb_topicTypeIsString(_local[index].last.type);
    const uint16_t textCap = isText ? _local[index].textCap : 0;
    BeeBusTopicSample* samples = new (std::nothrow) BeeBusTopicSample[depth];
    char* texts = isText ? static_cast<char*>(::operator new((size_t)depth * textCap,
                                                             std::nothrow))
                         : nullptr;
    if (!samples || (isText && !texts)) {
        delete[] samples;
        ::operator delete(texts);
        return false;
    }
    if (texts) memset(texts, 0, (size_t)depth * textCap);

    Ring* ring = _rings.find(id);
    if (ring) {
        freeRing(*ring);
    } else {
        ring = _rings.make(id);
        if (!ring) {
            delete[] samples;
            ::operator delete(texts);
            return false;
        }
    }
    ring->samples = samples;
    ring->texts = texts;
    ring->textCap = textCap;
    ring->head = 0;
    ring->count = 0;
    ring->depth = depth;
    ring->dropped = 0;

    _local[index].bufferDepth = depth;
    _local[index].flags = (uint8_t)(_local[index].flags | BEEBUS_TOPIC_F_BUFFERED);
    return true;
}

uint8_t BeeBusTopicMap::bufferDepth(uint16_t id) const {
    const uint16_t index = localIndex(id);
    return index == BEEBUS_TOPIC_NONE ? 0 : _local[index].bufferDepth;
}

uint8_t BeeBusTopicMap::ringPushSlot(Ring& ring, bool& overwrote) {
    const uint8_t slot = (uint8_t)((ring.head + ring.count) % ring.depth);
    if (ring.count < ring.depth) {
        ring.count++;
        overwrote = false;
        return slot;
    }
    // Full: the newest overwrote the oldest, so the oldest moves on.
    ring.head = (uint8_t)((ring.head + 1) % ring.depth);
    if (ring.dropped < 0xFFFF) ring.dropped++;
    overwrote = true;
    return slot;
}

bool BeeBusTopicMap::pushSample(uint16_t id, const BeeBusScalar& value, bb_time_t at) {
    Ring* ring = _rings.find(id);
    if (!ring || ring->depth == 0) return true;

    bool overwrote = false;
    const uint8_t slot = ringPushSlot(*ring, overwrote);
    ring->samples[slot].value = value;
    ring->samples[slot].at = at;
    return !overwrote;
}

bool BeeBusTopicMap::pushStringSample(uint16_t id, const char* text, bb_time_t at) {
    Ring* ring = _rings.find(id);
    if (!ring || ring->depth == 0 || !ring->texts || !text) return true;

    bool overwrote = false;
    const uint8_t slot = ringPushSlot(*ring, overwrote);
    ring->samples[slot].value = BeeBusScalar();
    ring->samples[slot].value.type = BEEBUS_T_STRING;
    ring->samples[slot].at = at;
    copyText(ring->texts + (size_t)slot * ring->textCap, ring->textCap, text,
             strnlen(text, ring->textCap));
    return !overwrote;
}

uint16_t BeeBusTopicMap::droppedCount(uint16_t id) const {
    const Ring* ring = _rings.find(id);
    return ring ? ring->dropped : 0;
}

void BeeBusTopicMap::clearDropped(uint16_t id) {
    Ring* ring = _rings.find(id);
    if (ring) ring->dropped = 0;
}

bool BeeBusTopicMap::peekSample(uint16_t id, BeeBusTopicSample& out) const {
    const Ring* ring = _rings.find(id);
    if (!ring || ring->count == 0) return false;
    out = ring->samples[ring->head];
    return true;
}

bool BeeBusTopicMap::peekStringSample(uint16_t id, BeeBusTopicSample& out,
                                      const char*& text) const {
    const Ring* ring = _rings.find(id);
    if (!ring || ring->count == 0 || !ring->texts) return false;
    out = ring->samples[ring->head];
    text = ring->texts + (size_t)ring->head * ring->textCap;
    return true;
}

void BeeBusTopicMap::dropSample(uint16_t id) {
    Ring* ring = _rings.find(id);
    if (!ring || ring->count == 0) return;
    ring->head = (uint8_t)((ring->head + 1) % ring->depth);
    ring->count--;
}

uint16_t BeeBusTopicMap::bufferedCount(uint16_t id) const {
    const Ring* ring = _rings.find(id);
    return ring ? ring->count : 0;
}

uint16_t BeeBusTopicMap::bufferedTotal() const {
    uint16_t total = 0;
    for (size_t i = 0; i < _rings.size(); i++) {
        total = (uint16_t)(total + _rings[i].count);
    }
    return total;
}

// --- Footprint ---

size_t BeeBusTopicMap::heapBytes() const {
    size_t bytes = _local.capacity() * sizeof(BeeBusLocalTopic) +
                   _remote.capacity() * sizeof(BeeBusRemoteTopic) +
                   _slots.capacity() * sizeof(BeeBusSubSlot) +
                   _contracts.capacity() * sizeof(BeeBusContract) +
                   _due.capacity() * sizeof(BeeBusDuePublish);
    for (size_t i = 0; i < _rings.size(); i++) {
        bytes += sizeof(Ring) + _rings[i].depth * sizeof(BeeBusTopicSample) +
                 (size_t)_rings[i].depth * _rings[i].textCap;
    }
    // String topics only, so a map of scalars adds nothing here.
    for (size_t i = 0; i < _local.size(); i++) {
        bytes += _local[i].lastText ? _local[i].textCap : 0;
    }
#if BEEBUS_TOPIC_NAMES
    bytes += _discovered.capacity() * sizeof(Discovered);
#endif
    return bytes;
}
