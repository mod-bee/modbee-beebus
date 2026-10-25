#pragma once

// Fixed-size storage for the protocol's running path.
//
// Nothing here allocates once a node is running: a byte buffer has its
// capacity built in, and a list takes its storage once, when its owner starts
// (reserve), and never again. The same code runs on a host and on a board; only
// the capacities differ. The interfaces are the small parts of std::vector the
// protocol uses, so the code above reads the same.

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <new>
#include <initializer_list>

// The largest PDU any queued operation carries: a topic string value (255
// bytes and its header), a Modbus PDU (253), a topic list page (200), a batch of
// acknowledgements (96). A section on the wire could be longer; nothing builds
// one.
#ifndef BEEBUS_MAX_PDU_BYTES
#define BEEBUS_MAX_PDU_BYTES 272
#endif

// --- A byte buffer of fixed capacity, held in place ---
class BeeBusBytes {
public:
    BeeBusBytes() : _len(0), _overflow(false) {}

    size_t size() const { return _len; }
    bool empty() const { return _len == 0; }
    static size_t capacity() { return BEEBUS_MAX_PDU_BYTES; }
    // Set when something asked for more than the capacity; what fitted is kept.
    bool overflow() const { return _overflow; }

    uint8_t* data() { return _bytes; }
    const uint8_t* data() const { return _bytes; }
    uint8_t* begin() { return _bytes; }
    uint8_t* end() { return _bytes + _len; }
    const uint8_t* begin() const { return _bytes; }
    const uint8_t* end() const { return _bytes + _len; }
    uint8_t& operator[](size_t i) { return _bytes[i]; }
    const uint8_t& operator[](size_t i) const { return _bytes[i]; }
    uint8_t& back() { return _bytes[_len - 1]; }

    void clear() { _len = 0; _overflow = false; }
    void reserve(size_t) {}

    void resize(size_t n, uint8_t fill = 0) {
        if (n > BEEBUS_MAX_PDU_BYTES) { n = BEEBUS_MAX_PDU_BYTES; _overflow = true; }
        if (n > _len) memset(_bytes + _len, fill, n - _len);
        _len = (uint16_t)n;
    }
    void assign(size_t n, uint8_t fill) { _len = 0; _overflow = false; resize(n, fill); }
    void assign(const uint8_t* src, size_t n) { _len = 0; _overflow = false; append(src, n); }
    BeeBusBytes& operator=(std::initializer_list<uint8_t> list) {
        assign(list.begin(), list.size());
        return *this;
    }
    void push_back(uint8_t value) {
        if (_len >= BEEBUS_MAX_PDU_BYTES) { _overflow = true; return; }
        _bytes[_len++] = value;
    }
    void append(const uint8_t* src, size_t n) {
        const size_t room = (size_t)BEEBUS_MAX_PDU_BYTES - _len;
        if (n > room) { n = room; _overflow = true; }
        if (n) memmove(_bytes + _len, src, n);
        _len = (uint16_t)(_len + n);
    }
    // Only appending is supported, which is all the protocol does.
    void insert(const uint8_t* at, const uint8_t* first, const uint8_t* last) {
        if (at == end()) append(first, (size_t)(last - first));
        else _overflow = true;
    }
    void insert(const uint8_t* at, const char* first, const char* last) {
        insert(at, reinterpret_cast<const uint8_t*>(first), reinterpret_cast<const uint8_t*>(last));
    }

    bool operator==(const BeeBusBytes& other) const {
        return _len == other._len && memcmp(_bytes, other._bytes, _len) == 0;
    }
    bool operator!=(const BeeBusBytes& other) const { return !(*this == other); }

private:
    uint8_t _bytes[BEEBUS_MAX_PDU_BYTES];
    uint16_t _len;
    bool _overflow;
};

// --- A list of fixed capacity, in insertion order ---
//
// The storage is taken once by reserve(), when the owner starts; until then the
// list holds nothing. push_back refuses when it is full. erase keeps the order
// of what is left, as the queues' order is what goes on the wire.
template <typename T>
class BeeBusList {
public:
    BeeBusList() : _items(nullptr), _count(0), _capacity(0) {}
    ~BeeBusList() { release(); }
    BeeBusList(const BeeBusList&) = delete;
    BeeBusList& operator=(const BeeBusList&) = delete;
    BeeBusList(BeeBusList&& other) : _items(other._items), _count(other._count),
                                     _capacity(other._capacity) {
        other._items = nullptr; other._count = 0; other._capacity = 0;
    }
    BeeBusList& operator=(BeeBusList&& other) {
        if (this != &other) {
            release();
            _items = other._items; _count = other._count; _capacity = other._capacity;
            other._items = nullptr; other._count = 0; other._capacity = 0;
        }
        return *this;
    }

    // Takes storage for `capacity` items. Anything held is dropped. The same
    // capacity again keeps the storage it has. False when the memory is not
    // there; the list then holds nothing and refuses every push_back.
    bool reserve(size_t capacity) {
        clear();
        if (capacity == _capacity && _items) return true;
        release();
        if (capacity == 0) return true;
        if (capacity > 0xFFFF) return false;
        _items = static_cast<T*>(::operator new(capacity * sizeof(T), std::nothrow));
        _capacity = _items ? (uint16_t)capacity : 0;
        return _items != nullptr;
    }

    // Makes room for at least `capacity` items, keeping what is held. For
    // tables that grow as an application sets things up, or a few entries at a
    // time as peers appear.
    bool grow(size_t capacity) {
        if (capacity <= _capacity) return true;
        T* items = static_cast<T*>(::operator new(capacity * sizeof(T), std::nothrow));
        if (!items) return false;
        for (size_t i = 0; i < _count; i++) {
            new (&items[i]) T(static_cast<T&&>(_items[i]));
            _items[i].~T();
        }
        ::operator delete(_items);
        _items = items;
        _capacity = (uint16_t)capacity;
        return true;
    }

    size_t size() const { return _count; }
    bool empty() const { return _count == 0; }
    size_t capacity() const { return _capacity; }
    bool full() const { return _count >= _capacity; }

    T& operator[](size_t i) { return _items[i]; }
    const T& operator[](size_t i) const { return _items[i]; }
    T* begin() { return _items; }
    T* end() { return _items + _count; }
    const T* begin() const { return _items; }
    const T* end() const { return _items + _count; }
    T& front() { return _items[0]; }
    T& back() { return _items[_count - 1]; }

    bool push_back(const T& value) {
        if (_count >= _capacity) return false;
        new (&_items[_count]) T(value);
        _count++;
        return true;
    }

    // Puts `value` at `index`, moving the rest up. Refused when full.
    bool insertAt(size_t index, const T& value) {
        if (_count >= _capacity || index > _count) return false;
        if (index == _count) return push_back(value);
        new (&_items[_count]) T(static_cast<T&&>(_items[_count - 1]));
        for (size_t i = _count - 1; i > index; i--) {
            _items[i] = static_cast<T&&>(_items[i - 1]);
        }
        _items[index] = value;
        _count++;
        return true;
    }

    // Removes the item at `index`, keeping the order of the rest.
    void eraseAt(size_t index) {
        if (index >= _count) return;
        for (size_t i = index; i + 1 < _count; i++) {
            _items[i] = static_cast<T&&>(_items[i + 1]);
        }
        _count--;
        _items[_count].~T();
    }
    T* erase(T* at) {
        const size_t index = (size_t)(at - _items);
        eraseAt(index);
        return _items + index;
    }

    void clear() {
        for (size_t i = 0; i < _count; i++) _items[i].~T();
        _count = 0;
    }

private:
    void release() {
        clear();
        ::operator delete(_items);
        _items = nullptr;
        _capacity = 0;
    }

    T* _items;
    uint16_t _count;
    uint16_t _capacity;
};

// --- A table of plain entries sorted by a 16-bit address ---
//
// Each entry starts with its address (uint16_t). Entries are plain data, moved
// with memmove, so one copy of the search / insert / erase code serves every
// entry type. A table grows only when an application binds or configures
// something, never from traffic; lookups are a binary search.
class BeeBusAddrTableBase {
public:
    BeeBusAddrTableBase(size_t elemSize)
        : _bytes(nullptr), _count(0), _capacity(0), _elem((uint16_t)elemSize) {}
    ~BeeBusAddrTableBase() { ::operator delete(_bytes); }
    BeeBusAddrTableBase(const BeeBusAddrTableBase&) = delete;
    BeeBusAddrTableBase& operator=(const BeeBusAddrTableBase&) = delete;
    BeeBusAddrTableBase(BeeBusAddrTableBase&& other)
        : _bytes(other._bytes), _count(other._count), _capacity(other._capacity),
          _elem(other._elem) {
        other._bytes = nullptr; other._count = 0; other._capacity = 0;
    }
    BeeBusAddrTableBase& operator=(BeeBusAddrTableBase&& other) {
        if (this != &other) {
            ::operator delete(_bytes);
            _bytes = other._bytes; _count = other._count; _capacity = other._capacity;
            other._bytes = nullptr; other._count = 0; other._capacity = 0;
        }
        return *this;
    }

    size_t size() const { return _count; }
    void clear() { _count = 0; }

    // The first entry at or above `address` (size() when none).
    size_t lowerBound(uint16_t address) const {
        size_t lo = 0, hi = _count;
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (keyAt(mid) < address) lo = mid + 1; else hi = mid;
        }
        return lo;
    }
    // The entry for `address`, or size().
    size_t indexOf(uint16_t address) const {
        const size_t i = lowerBound(address);
        return (i < _count && keyAt(i) == address) ? i : _count;
    }
    // The entry for `address`, made (zeroed, keyed) if there is none. size()
    // when there is no memory for it.
    size_t insert(uint16_t address) {
        const size_t i = lowerBound(address);
        if (i < _count && keyAt(i) == address) return i;
        if (_count >= _capacity) {
            const size_t capacity = _capacity ? (size_t)_capacity * 2 : 8;
            uint8_t* bytes = static_cast<uint8_t*>(::operator new(capacity * _elem, std::nothrow));
            if (!bytes) return _count;
            if (_count) memcpy(bytes, _bytes, (size_t)_count * _elem);
            ::operator delete(_bytes);
            _bytes = bytes;
            _capacity = (uint16_t)capacity;
        }
        memmove(at(i + 1), at(i), (size_t)(_count - i) * _elem);
        memset(at(i), 0, _elem);
        memcpy(at(i), &address, sizeof(address));
        _count++;
        return i;
    }
    void eraseAt(size_t i) {
        if (i >= _count) return;
        memmove(at(i), at(i + 1), (size_t)(_count - i - 1) * _elem);
        _count--;
    }

protected:
    uint8_t* at(size_t i) const { return _bytes + i * _elem; }
    uint16_t keyAt(size_t i) const {
        uint16_t key;
        memcpy(&key, at(i), sizeof(key));
        return key;
    }

private:
    uint8_t* _bytes;
    uint16_t _count;
    uint16_t _capacity;
    uint16_t _elem;
};

template <typename T>
class BeeBusAddrTable : public BeeBusAddrTableBase {
public:
    BeeBusAddrTable() : BeeBusAddrTableBase(sizeof(T)) {}
    BeeBusAddrTable(BeeBusAddrTable&&) = default;
    BeeBusAddrTable& operator=(BeeBusAddrTable&&) = default;
    T& operator[](size_t i) { return *reinterpret_cast<T*>(at(i)); }
    const T& operator[](size_t i) const { return *reinterpret_cast<const T*>(at(i)); }
    T* find(uint16_t address) {
        const size_t i = indexOf(address);
        return i < size() ? &(*this)[i] : nullptr;
    }
    const T* find(uint16_t address) const {
        const size_t i = indexOf(address);
        return i < size() ? &(*this)[i] : nullptr;
    }
    T* make(uint16_t address) {
        const size_t i = insert(address);
        return i < size() ? &(*this)[i] : nullptr;
    }
};
