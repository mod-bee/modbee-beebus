#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// --- Constructor and destructor ---
BeeBusModbusDataMap::BeeBusModbusDataMap() {
}

BeeBusModbusDataMap::~BeeBusModbusDataMap() {
    clearAll();
}

void BeeBusModbusDataMap::clearAll() {
    // Every binding, record and failsafe goes. The counters stay: they say what
    // happened, and nothing here takes that back.
    for (size_t i = 0; i < _coils.size();) {
        BitEntry& e = _coils[i];
        e.var = nullptr; e.lastWriter = 0; e.flags = 0;
        if (e.writeCount == 0 && e.failsafeCount == 0 && e.writeSerial == 0) _coils.eraseAt(i);
        else i++;
    }
    for (size_t i = 0; i < _hregs.size();) {
        WordEntry& e = _hregs[i];
        e.var = nullptr; e.lastWriter = 0; e.flags = 0; e.failsafe = 0;
        if (e.writeCount == 0 && e.failsafeCount == 0 && e.writeSerial == 0) _hregs.eraseAt(i);
        else i++;
    }
    _ists.clear();
    _iregs.clear();
    for (size_t i = 0; i < _strings.size(); i++) {
        ::operator delete(_strings[i].failsafe);
    }
    _strings.clear();
    _inputStrings.clear();
    _failsafeHandler = nullptr;
    _coilChanged.clear();
    _hregChanged.clear();
    _stringChanged.clear();
    _pendingChanges.clear();
}

// --- Entry helpers ---
BeeBusModbusDataMap::BitEntry* BeeBusModbusDataMap::boundBit(BitTable& table, uint16_t address) {
    BitEntry* e = table.find(address);
    return (e && e->var) ? e : nullptr;
}

const BeeBusModbusDataMap::BitEntry* BeeBusModbusDataMap::boundBit(const BitTable& table,
                                                                    uint16_t address) {
    const BitEntry* e = table.find(address);
    return (e && e->var) ? e : nullptr;
}

BeeBusModbusDataMap::WordEntry* BeeBusModbusDataMap::boundWord(WordTable& table, uint16_t address) {
    WordEntry* e = table.find(address);
    return (e && e->var) ? e : nullptr;
}

const BeeBusModbusDataMap::WordEntry* BeeBusModbusDataMap::boundWord(const WordTable& table,
                                                                      uint16_t address) {
    const WordEntry* e = table.find(address);
    return (e && e->var) ? e : nullptr;
}

void BeeBusModbusDataMap::tidyBit(BitTable& table, uint16_t address) {
    const size_t i = table.indexOf(address);
    if (i >= table.size()) return;
    const BitEntry& e = table[i];
    if (!e.var && !e.lastWriter && !e.flags &&
        e.writeCount == 0 && e.failsafeCount == 0 && e.writeSerial == 0) {
        table.eraseAt(i);
    }
}

void BeeBusModbusDataMap::tidyWord(WordTable& table, uint16_t address) {
    const size_t i = table.indexOf(address);
    if (i >= table.size()) return;
    const WordEntry& e = table[i];
    if (!e.var && !e.lastWriter && !e.flags &&
        e.writeCount == 0 && e.failsafeCount == 0 && e.writeSerial == 0) {
        table.eraseAt(i);
    }
}

// --- Span helpers ---
BeeBusModbusDataMap::StringEntry* BeeBusModbusDataMap::findSpan(StringTable& spans,
                                                                uint16_t address) {
    return const_cast<StringEntry*>(findSpan(static_cast<const StringTable&>(spans), address));
}

const BeeBusModbusDataMap::StringEntry* BeeBusModbusDataMap::findSpan(const StringTable& spans,
                                                                      uint16_t address) {
    // The only span that can cover an address is the last one starting at or
    // below it.
    const size_t above = (address == 0xFFFF) ? spans.size() : spans.lowerBound((uint16_t)(address + 1));
    if (above == 0) {
        return nullptr;
    }
    const StringEntry& span = spans[above - 1];
    if ((uint32_t)span.address + span.regCount > address) {
        return &span;
    }
    return nullptr;
}

bool BeeBusModbusDataMap::spansCollide(const StringTable& spans, uint16_t address, uint16_t regCount) {
    const uint32_t end = (uint32_t)address + regCount;
    const size_t above = (address == 0xFFFF) ? spans.size() : spans.lowerBound((uint16_t)(address + 1));

    // One starting at or below the new span that reaches into it.
    if (above > 0) {
        const StringEntry& prev = spans[above - 1];
        if ((uint32_t)prev.address + prev.regCount > address) {
            return true;
        }
    }

    // One starting inside the new span.
    return above < spans.size() && (uint32_t)spans[above].address < end;
}

bool BeeBusModbusDataMap::rangeIsBound(const WordTable& bound, uint16_t address, uint16_t regCount) {
    const uint32_t end = (uint32_t)address + regCount;
    for (size_t i = bound.lowerBound(address); i < bound.size(); i++) {
        if ((uint32_t)bound[i].address >= end) break;
        if (bound[i].var) return true;
    }
    return false;
}

// With an odd capacity the span's last byte lies past the buffer. It reads as
// 0 and is never written, so neither path touches memory that is not the
// caller's.
int16_t BeeBusModbusDataMap::spanRegister(const StringEntry& span, uint16_t offset) {
    const uint16_t at = (uint16_t)(offset * 2);
    const uint8_t* bytes = (const uint8_t*)span.buf;
    const uint8_t high = bytes[at];
    const uint8_t low = ((uint16_t)(at + 1) < span.capacity) ? bytes[at + 1] : 0;
    return (int16_t)(((uint16_t)high << 8) | low);
}

bool BeeBusModbusDataMap::setSpanRegister(const StringEntry& span, uint16_t offset, int16_t value) {
    uint8_t* bytes = (uint8_t*)span.buf;
    const uint16_t at = (uint16_t)(offset * 2);
    const uint16_t before = (uint16_t)spanRegister(span, offset);

    bytes[at] = (uint8_t)((uint16_t)value >> 8);
    if ((uint16_t)(at + 1) < span.capacity) {
        bytes[at + 1] = (uint8_t)((uint16_t)value & 0xFF);
    }

    // The last byte of the buffer is reserved for the terminator, so text
    // arriving without one is still a string rather than a run off the end.
    span.buf[span.capacity - 1] = '\0';
    return (uint16_t)spanRegister(span, offset) != before;
}

bool BeeBusModbusDataMap::writeSpan(const StringEntry& span, const char* text) {
    if (!text) {
        return false;
    }
    // The whole buffer is cleared first, so a shorter message replacing a
    // longer one leaves no tail of the old one to go out on the next read.
    memset(span.buf, 0, span.capacity);
    strncpy(span.buf, text, (size_t)(span.capacity - 1));
    return true;
}

bool BeeBusModbusDataMap::readSpan(const StringEntry& span, char* out, uint16_t size) {
    if (!out || size == 0) {
        return false;
    }
    // Bounded by the span as well as by `out`: the terminator is the buffer's
    // last byte, so the text can never be longer than capacity - 1.
    const size_t room = (size_t)(size - 1) < (size_t)(span.capacity - 1)
        ? (size_t)(size - 1) : (size_t)(span.capacity - 1);
    strncpy(out, span.buf, room);
    out[room] = '\0';
    return true;
}

// --- Register binding methods ---
//
// A bind is refused rather than silently replacing an existing one: rebinding
// an address is a programming mistake, and the caller cannot see it happen.
// There is no budget: a table holds every address the application binds, and
// a bind is refused only when the memory for it is not there.
bool BeeBusModbusDataMap::bindBit(BitTable& table, uint16_t address, bool* variable) {
    if (!variable || boundBit(table, address)) {
        return false;
    }
    BitEntry* e = table.make(address);
    if (!e) {
        return false;
    }
    e->var = variable;
    return true;
}

bool BeeBusModbusDataMap::addCoil(uint16_t address, bool* variable) {
    return bindBit(_coils, address, variable);
}

bool BeeBusModbusDataMap::addHreg(uint16_t address, int16_t* variable) {
    return bindValue(_hregs, _strings, address, variable, BEEBUS_T_I16);
}

bool BeeBusModbusDataMap::addHregValue(uint16_t address, void* variable, uint8_t type) {
    return bindValue(_hregs, _strings, address, variable, type);
}

bool BeeBusModbusDataMap::addIregValue(uint16_t address, void* variable, uint8_t type) {
    return bindValue(_iregs, _inputStrings, address, variable, type);
}

// Every register of the value is checked before any is bound, so a value is
// either wholly bound or not at all.
bool BeeBusModbusDataMap::bindValue(WordTable& regs, const StringTable& spans, uint16_t address,
                                    void* variable, uint8_t type) {
    const uint8_t words = bb_registersOf(type);
    if (!variable || words == 0) {
        return false;
    }
    if ((uint32_t)address + words > 0x10000u) {
        return false;   // Would wrap to address 0
    }
    for (uint8_t w = 0; w < words; w++) {
        const uint16_t at = (uint16_t)(address + w);
        if (boundWord(regs, at) || findSpan(spans, at)) {
            return false;   // Already bound, or covered by a string
        }
    }
    for (uint8_t w = 0; w < words; w++) {
        WordEntry* e = regs.make((uint16_t)(address + w));
        if (!e) {
            unbindValue(regs, address, true);
            return false;
        }
        e->var = variable;
        e->type = (uint8_t)(type & BEEBUS_T_MASK);
        e->word = w;
    }
    return true;
}

// The first register of the value covering `address`, or `address` itself
// when nothing is bound there.
uint16_t BeeBusModbusDataMap::valueStart(const WordTable& regs, uint16_t address) {
    const WordEntry* e = boundWord(regs, address);
    if (!e) {
        return address;
    }
    return (uint16_t)(address - e->word);
}

uint16_t BeeBusModbusDataMap::unbindValue(WordTable& regs, uint16_t address, bool keepRecords) {
    const uint16_t start = valueStart(regs, address);
    const WordEntry* first = boundWord(regs, start);
    if (!first) {
        return start;
    }
    const uint8_t words = bb_registersOf(first->type);
    for (uint8_t w = 0; w < words; w++) {
        const uint16_t at = (uint16_t)(start + w);
        WordEntry* e = regs.find(at);
        if (!e || !e->var) continue;
        e->var = nullptr;
        e->type = 0;
        e->word = 0;
        if (keepRecords) {
            tidyWord(regs, at);
        } else {
            regs.eraseAt(regs.indexOf(at));
        }
    }
    return start;
}

bool BeeBusModbusDataMap::addIsts(uint16_t address, bool* variable) {
    return bindBit(_ists, address, variable);
}

bool BeeBusModbusDataMap::addIreg(uint16_t address, int16_t* variable) {
    return bindValue(_iregs, _inputStrings, address, variable, BEEBUS_T_I16);
}

// --- String binding ---
bool BeeBusModbusDataMap::bindString(StringTable& spans, const WordTable& scalars,
                               uint16_t address, char* buffer, uint16_t capacity) {
    if (!buffer || capacity < 2) {
        return false;
    }

    // Rounded up: an odd capacity's last register is half buffer, half nothing.
    const uint16_t regCount = (uint16_t)((capacity + 1u) / 2u);

    // A span that ran off the top of the address space would reappear at the
    // bottom and answer for registers belonging to something else.
    if ((uint32_t)address + regCount > 0x10000u) {
        return false;
    }

    if (spansCollide(spans, address, regCount) || rangeIsBound(scalars, address, regCount)) {
        return false;
    }

    // A holding string's counters are kept with the holding registers, on its
    // first address. The entry is made now, so a peer's write never has to.
    if (&spans == &_strings && !_hregs.make(address)) {
        return false;
    }
    StringEntry* entry = spans.make(address);
    if (!entry) {
        return false;
    }
    entry->buf = buffer;
    entry->capacity = capacity;
    entry->regCount = regCount;

    // Terminated from the moment it is bound, so a buffer the application has
    // not filled in yet still reads as a string rather than as whatever the
    // memory held.
    buffer[capacity - 1] = '\0';
    return true;
}

void BeeBusModbusDataMap::unbindString(StringTable& spans, uint16_t address) {
    const size_t i = spans.indexOf(address);
    if (i >= spans.size()) {
        return;
    }
    ::operator delete(spans[i].failsafe);
    spans.eraseAt(i);
}

bool BeeBusModbusDataMap::addString(uint16_t address, char* buffer, uint16_t capacity) {
    return bindString(_strings, _hregs, address, buffer, capacity);
}

bool BeeBusModbusDataMap::addInputString(uint16_t address, char* buffer, uint16_t capacity) {
    return bindString(_inputStrings, _iregs, address, buffer, capacity);
}

void BeeBusModbusDataMap::removeString(uint16_t address) {
    unbindString(_strings, address);
    for (size_t i = 0; i < _stringChanged.size(); i++) {
        if (_stringChanged[i].address == address) { _stringChanged.eraseAt(i); break; }
    }
    tidyWord(_hregs, address);
}

void BeeBusModbusDataMap::removeInputString(uint16_t address) {
    unbindString(_inputStrings, address);
}

bool BeeBusModbusDataMap::setString(uint16_t address, const char* text) {
    const StringEntry* span = _strings.find(address);
    return span && writeSpan(*span, text);
}

bool BeeBusModbusDataMap::setInputString(uint16_t address, const char* text) {
    const StringEntry* span = _inputStrings.find(address);
    return span && writeSpan(*span, text);
}

bool BeeBusModbusDataMap::getString(uint16_t address, char* out, uint16_t size) const {
    const StringEntry* span = _strings.find(address);
    return span && readSpan(*span, out, size);
}

bool BeeBusModbusDataMap::getInputString(uint16_t address, char* out, uint16_t size) const {
    const StringEntry* span = _inputStrings.find(address);
    return span && readSpan(*span, out, size);
}

uint16_t BeeBusModbusDataMap::stringRegisters(uint16_t address) const {
    const StringEntry* span = _strings.find(address);
    return span ? span->regCount : 0;
}

uint16_t BeeBusModbusDataMap::inputStringRegisters(uint16_t address) const {
    const StringEntry* span = _inputStrings.find(address);
    return span ? span->regCount : 0;
}

// --- Register existence checks ---
bool BeeBusModbusDataMap::hasCoil(uint16_t address) const {
    return boundBit(_coils, address) != nullptr;
}

bool BeeBusModbusDataMap::hasHreg(uint16_t address) const {
    return boundWord(_hregs, address) || findSpan(_strings, address);
}

bool BeeBusModbusDataMap::hasIsts(uint16_t address) const {
    return boundBit(_ists, address) != nullptr;
}

bool BeeBusModbusDataMap::hasIreg(uint16_t address) const {
    return boundWord(_iregs, address) || findSpan(_inputStrings, address);
}

// --- Register read methods ---
bool BeeBusModbusDataMap::getCoil(uint16_t address) const {
    const BitEntry* e = boundBit(_coils, address);
    return e ? *(e->var) : false;
}

int16_t BeeBusModbusDataMap::getHreg(uint16_t address) const {
    const WordEntry* e = boundWord(_hregs, address);
    if (e) {
        return (int16_t)bb_valueRegister(e->var, e->type, e->word);
    }
    const StringEntry* span = findSpan(_strings, address);
    if (span) {
        return spanRegister(*span, (uint16_t)(address - span->address));
    }
    return 0;
}

bool BeeBusModbusDataMap::getIsts(uint16_t address) const {
    const BitEntry* e = boundBit(_ists, address);
    return e ? *(e->var) : false;
}

int16_t BeeBusModbusDataMap::getIreg(uint16_t address) const {
    const WordEntry* e = boundWord(_iregs, address);
    if (e) {
        return (int16_t)bb_valueRegister(e->var, e->type, e->word);
    }
    const StringEntry* span = findSpan(_inputStrings, address);
    if (span) {
        return spanRegister(*span, (uint16_t)(address - span->address));
    }
    return 0;
}

// --- Register write methods ---
bool BeeBusModbusDataMap::setCoil(uint16_t address, bool value, uint8_t sourceNodeID) {
    BitEntry* e = boundBit(_coils, address);
    if (!e) {
        return false;
    }

    const bool changed = (*(e->var) != value);
    *(e->var) = value;

    // The last writer is not cleared here: applyFailsafeForNode calls this with
    // no source and clears it itself.
    if (sourceNodeID != 0) {
        e->lastWriter = sourceNodeID;
        e->writeCount++;
        e->writeSerial = ++_writeSerial;
        if (changed) {
            noteChange(ChangeKind::Coil, address, sourceNodeID);
        }
    }
    return true;
}

bool BeeBusModbusDataMap::setHreg(uint16_t address, int16_t value, uint8_t sourceNodeID) {
    WordEntry* e = boundWord(_hregs, address);
    if (e) {
        bool changed = false;
        if (!bb_setValueRegister(e->var, e->type, e->word, (uint16_t)value, &changed)) {
            return false;   // A number the type cannot hold
        }
        if (sourceNodeID != 0) {
            e->lastWriter = sourceNodeID;
            // Counted on the value's first register, like a string, so a
            // caller asking about the value finds it there.
            WordEntry* first = _hregs.find((uint16_t)(address - e->word));
            if (first) {
                first->writeCount++;
                first->writeSerial = ++_writeSerial;
            }
            if (changed) {
                noteChange(ChangeKind::Hreg, address, sourceNodeID);
            }
        }
        return true;
    }

    // Inside a string, the register is two characters of it. The event and the
    // last-writer record belong to the whole string, so both are keyed on where
    // it starts and a multi-register write raises one of each.
    StringEntry* span = findSpan(_strings, address);
    if (span) {
        const bool changed = setSpanRegister(*span, (uint16_t)(address - span->address), value);
        if (sourceNodeID != 0) {
            const uint16_t start = span->address;
            span->lastWriter = sourceNodeID;
            // Counted on the span's first address, like the event, so a write
            // covering several registers counts once. The entry was made when
            // the string was bound.
            WordEntry* counts = _hregs.make(start);
            if (counts) {
                counts->writeCount++;
                counts->writeSerial = ++_writeSerial;
            }
            if (changed) {
                noteChange(ChangeKind::String, start, sourceNodeID);
            }
        }
        return true;
    }

    return false;
}

bool BeeBusModbusDataMap::setIsts(uint16_t address, bool value) {
    BitEntry* e = boundBit(_ists, address);
    if (e) {
        *(e->var) = value;
        return true;
    }
    return false;
}

bool BeeBusModbusDataMap::setIreg(uint16_t address, int16_t value) {
    const WordEntry* e = boundWord(_iregs, address);
    if (e) {
        return bb_setValueRegister(e->var, e->type, e->word, (uint16_t)value, nullptr);
    }
    const StringEntry* span = findSpan(_inputStrings, address);
    if (span) {
        setSpanRegister(*span, (uint16_t)(address - span->address), value);
        return true;
    }
    return false;
}

void BeeBusModbusDataMap::getIsts(uint16_t address, bool* values, uint16_t quantity) const {
    if (!values) return;
    for (uint16_t i = 0; i < quantity; i++) {
        values[i] = getIsts(address + i);
    }
}

// --- Remove register bindings ---
void BeeBusModbusDataMap::removeCoil(uint16_t address) {
    // The binding, last writer, failsafe and change handler go; the counters
    // stay with the address.
    BitEntry* e = _coils.find(address);
    if (e) {
        e->var = nullptr;
        e->lastWriter = 0;
        e->flags = 0;
        tidyBit(_coils, address);
    }
    for (size_t i = 0; i < _coilChanged.size(); i++) {
        if (_coilChanged[i].address == address) { _coilChanged.eraseAt(i); break; }
    }
}

void BeeBusModbusDataMap::removeHreg(uint16_t address) {
    // A value over two registers goes as a whole, whichever of them is named.
    const WordEntry* bound = boundWord(_hregs, address);
    const uint8_t words = bound ? bb_registersOf(bound->type) : 1;
    const uint16_t start = unbindValue(_hregs, address, true);
    for (uint8_t w = 0; w < words; w++) {
        const uint16_t at = (uint16_t)(start + w);
        WordEntry* e = _hregs.find(at);
        if (e) {
            e->lastWriter = 0;
            e->flags = 0;
            e->failsafe = 0;
            tidyWord(_hregs, at);
        }
        for (size_t i = 0; i < _hregChanged.size(); i++) {
            if (_hregChanged[i].address == at) { _hregChanged.eraseAt(i); break; }
        }
    }
}

void BeeBusModbusDataMap::removeIsts(uint16_t address) {
    const size_t i = _ists.indexOf(address);
    if (i < _ists.size()) {
        _ists.eraseAt(i);
    }
}

void BeeBusModbusDataMap::removeIreg(uint16_t address) {
    unbindValue(_iregs, address, false);
}

// --- Change notification ---
template <typename H>
void BeeBusModbusDataMap::watch(BeeBusList<Watch<H> >& list, uint16_t address, H handler) {
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].address != address) continue;
        if (handler) list[i].handler = handler;
        else list.eraseAt(i);
        return;
    }
    if (!handler) {
        return;
    }
    if (list.full() && !list.grow(list.capacity() ? list.capacity() * 2 : 4)) {
        return;
    }
    Watch<H> entry;
    entry.address = address;
    entry.handler = handler;
    list.push_back(entry);
}

template <typename H>
const H* BeeBusModbusDataMap::watcher(const BeeBusList<Watch<H> >& list, uint16_t address) {
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].address == address) return &list[i].handler;
    }
    return nullptr;
}

void BeeBusModbusDataMap::roomForChanges() {
    // One pending change per watched address at most, for the dispatch running
    // and the one after it.
    const size_t watched = _coilChanged.size() + _hregChanged.size() + _stringChanged.size();
    _pendingChanges.grow(2 * watched);
}

void BeeBusModbusDataMap::onCoilChanged(uint16_t address, BeeBusCoilChangedHandler handler) {
    watch(_coilChanged, address, handler);
    roomForChanges();
}

void BeeBusModbusDataMap::onHregChanged(uint16_t address, BeeBusHregChangedHandler handler) {
    watch(_hregChanged, address, handler);
    roomForChanges();
}

void BeeBusModbusDataMap::onStringChanged(uint16_t address, BeeBusStringChangedHandler handler) {
    watch(_stringChanged, address, handler);
    roomForChanges();
}

bool BeeBusModbusDataMap::isWatched(ChangeKind kind, uint16_t address) const {
    switch (kind) {
        case ChangeKind::Coil:   return watcher(_coilChanged, address) != nullptr;
        case ChangeKind::Hreg:   return watcher(_hregChanged, address) != nullptr;
        case ChangeKind::String: return watcher(_stringChanged, address) != nullptr;
    }
    return false;
}

void BeeBusModbusDataMap::noteChange(ChangeKind kind, uint16_t address, uint8_t fromNodeID) {
    if (!isWatched(kind, address)) {
        return;
    }

    // Every register of a string reports the same span, so one write raises one
    // event. Only the changes not yet being reported are looked at.
    for (size_t i = _dispatching; i < _pendingChanges.size(); i++) {
        if (_pendingChanges[i].kind == kind && _pendingChanges[i].address == address) {
            return;
        }
    }

    PendingChange change;
    change.kind = kind;
    change.address = address;
    change.fromNodeID = fromNodeID;
    _pendingChanges.push_back(change);
}

void BeeBusModbusDataMap::dispatchChangeEvents() {
    if (_pendingChanges.empty() || _dispatching != 0) {
        return;
    }

    // Only those noted before this dispatch: a handler writing back into the
    // map notes changes of its own, and those belong to the next dispatch.
    const size_t count = _pendingChanges.size();
    _dispatching = count;
    for (size_t i = 0; i < count; i++) {
        const PendingChange change = _pendingChanges[i];

        // The handler is copied before it runs, because a handler is allowed to
        // unbind the address it was called for.
        switch (change.kind) {
            case ChangeKind::Coil: {
                const BeeBusCoilChangedHandler* found = watcher(_coilChanged, change.address);
                if (!found) break;
                BeeBusCoilChangedHandler handler = *found;
                handler(change.address, getCoil(change.address), change.fromNodeID);
                break;
            }
            case ChangeKind::Hreg: {
                const BeeBusHregChangedHandler* found = watcher(_hregChanged, change.address);
                if (!found) break;
                BeeBusHregChangedHandler handler = *found;
                handler(change.address, getHreg(change.address), change.fromNodeID);
                break;
            }
            case ChangeKind::String: {
                const BeeBusStringChangedHandler* found = watcher(_stringChanged, change.address);
                if (!found) break;
                const StringEntry* span = _strings.find(change.address);
                if (!span) break;
                BeeBusStringChangedHandler handler = *found;
                handler(change.address, span->buf, change.fromNodeID);
                break;
            }
        }
    }
    for (size_t i = 0; i < count && !_pendingChanges.empty(); i++) {
        _pendingChanges.eraseAt(0);
    }
    _dispatching = 0;
}

// --- Fail-safe support ---
void BeeBusModbusDataMap::roomForFailsafe() {
    // One failsafe applied per configured setting at most, so the list that
    // collects them is sized as the settings are made.
    _failsafeSettings++;
    _applied.grow(_failsafeSettings);
}

void BeeBusModbusDataMap::setCoilFailsafe(uint16_t address, BeeBusFailsafe action, bool value) {
    if (action == BeeBusFailsafe::ResetToValue) {
        BitEntry* e = _coils.make(address);
        if (!e) return;
        e->flags = (uint8_t)(F_FAILSAFE | (value ? F_FAILSAFE_ON : 0));
        roomForFailsafe();
    } else {
        BitEntry* e = _coils.find(address);
        if (!e) return;
        e->flags = 0;
        tidyBit(_coils, address);
    }
}

void BeeBusModbusDataMap::setHregFailsafe(uint16_t address, BeeBusFailsafe action, int16_t value) {
    if (action == BeeBusFailsafe::ResetToValue) {
        WordEntry* e = _hregs.make(address);
        if (!e) return;
        e->flags = F_FAILSAFE;
        e->failsafe = value;
        roomForFailsafe();
    } else {
        WordEntry* e = _hregs.find(address);
        if (!e) return;
        e->flags = 0;
        e->failsafe = 0;
        tidyWord(_hregs, address);
    }
}

bool BeeBusModbusDataMap::setStringFailsafe(uint16_t address, BeeBusFailsafe action, const char* text) {
    StringEntry* span = _strings.find(address);
    if (!span) {
        return false;
    }
    ::operator delete(span->failsafe);
    span->failsafe = nullptr;
    span->flags = 0;
    if (action != BeeBusFailsafe::ResetToValue) {
        return true;
    }

    // Copied: the caller's text need not outlive the call. Kept for as long as
    // the string is bound.
    const char* safe = text ? text : "";
    const size_t bytes = strlen(safe) + 1;
    span->failsafe = static_cast<char*>(::operator new(bytes, std::nothrow));
    if (!span->failsafe) {
        return false;
    }
    memcpy(span->failsafe, safe, bytes);
    span->flags = F_FAILSAFE;
    roomForFailsafe();
    return true;
}

void BeeBusModbusDataMap::onFailsafeApplied(BeeBusFailsafeHandler handler) {
    _failsafeHandler = handler;
}

uint16_t BeeBusModbusDataMap::writeCount(BeeBusRegisterType table, uint16_t address,
                                   uint16_t count) const {
    // A wide sum, so a range of busy addresses saturates rather than wraps.
    const uint32_t end = (uint32_t)address + count;
    uint32_t total = 0;
    if (bb_tableIsBits(table)) {
        for (size_t i = _coils.lowerBound(address); i < _coils.size(); i++) {
            if ((uint32_t)_coils[i].address >= end) break;
            total += _coils[i].writeCount;
        }
    } else {
        for (size_t i = _hregs.lowerBound(address); i < _hregs.size(); i++) {
            if ((uint32_t)_hregs[i].address >= end) break;
            total += _hregs[i].writeCount;
        }
    }
    return total > 0xFFFFu ? 0xFFFFu : (uint16_t)total;
}

uint16_t BeeBusModbusDataMap::writtenSince(BeeBusRegisterType table, uint16_t address,
                                          uint16_t count, uint32_t& since,
                                          uint16_t* firstAddress) const {
    // Walks the entries in the range in address order; one never written has
    // serial 0 and is passed over.
    const uint32_t end = (uint32_t)address + count;
    uint16_t values = 0;
    if (bb_tableIsBits(table)) {
        for (size_t i = _coils.lowerBound(address); i < _coils.size(); i++) {
            const BitEntry& e = _coils[i];
            if ((uint32_t)e.address >= end) break;
            if (e.writeSerial <= since) continue;
            if (values == 0 && firstAddress) *firstAddress = e.address;
            values++;
        }
    } else {
        for (size_t i = _hregs.lowerBound(address); i < _hregs.size(); i++) {
            const WordEntry& e = _hregs[i];
            if ((uint32_t)e.address >= end) break;
            if (e.writeSerial <= since) continue;
            if (values == 0 && firstAddress) *firstAddress = e.address;
            values++;
        }
    }
    since = _writeSerial;
    return values;
}

uint16_t BeeBusModbusDataMap::failsafeCount(BeeBusRegisterType table, uint16_t address) const {
    if (bb_tableIsBits(table)) {
        const BitEntry* e = _coils.find(address);
        return e ? e->failsafeCount : 0;
    }
    const WordEntry* e = _hregs.find(address);
    return e ? e->failsafeCount : 0;
}

void BeeBusModbusDataMap::applyFailsafeForNode(uint8_t nodeID) {
    // Collected rather than reported as they are found: a handler is free to
    // touch the data map, so the reports wait until every table is done.
    _applied.clear();

    // Only the addresses that were given a safe state are driven anywhere. The
    // rest keep the value the lost node left, which is what an operator who has
    // said nothing about an output is entitled to expect.
    //
    // The last-writer record is dropped either way: the node that set it has
    // gone, so nothing about that address is attributable to it any more.
    for (size_t i = 0; i < _coils.size(); i++) {
        BitEntry& e = _coils[i];
        if (e.lastWriter != nodeID) continue;
        e.lastWriter = 0;
        if (e.flags & F_FAILSAFE) {
            const uint16_t address = e.address;
            setCoil(address, (e.flags & F_FAILSAFE_ON) != 0, 0);
            _coils[i].failsafeCount++;
            Applied applied; applied.table = MB_OUTPUT_COIL; applied.address = address;
            _applied.push_back(applied);
        }
    }

    for (size_t i = 0; i < _hregs.size(); i++) {
        WordEntry& e = _hregs[i];
        if (e.lastWriter != nodeID) continue;
        e.lastWriter = 0;
        if (e.flags & F_FAILSAFE) {
            const uint16_t address = e.address;
            setHreg(address, e.failsafe, 0);
            _hregs[i].failsafeCount++;
            Applied applied; applied.table = MB_HOLDING_REGISTER; applied.address = address;
            _applied.push_back(applied);
        }
    }

    for (size_t i = 0; i < _strings.size(); i++) {
        StringEntry& span = _strings[i];
        if (span.lastWriter != nodeID) continue;
        span.lastWriter = 0;
        if ((span.flags & F_FAILSAFE) && span.failsafe) {
            const uint16_t address = span.address;
            writeSpan(span, span.failsafe);
            WordEntry* counts = _hregs.make(address);
            if (counts) counts->failsafeCount++;
            Applied applied; applied.table = MB_HOLDING_REGISTER; applied.address = address;
            _applied.push_back(applied);
        }
    }

    if (_applied.empty()) {
        return;
    }

    BEEBUS_DEBUG_OPERATIONS("FAILSAFE: drove %d addresses to safe values after losing Node %d",
        (int)_applied.size(), nodeID);

    if (_failsafeHandler) {
        BeeBusFailsafeHandler handler = _failsafeHandler;
        for (size_t i = 0; i < _applied.size(); i++) {
            handler(_applied[i].table, _applied[i].address, nodeID);
        }
    }
}
