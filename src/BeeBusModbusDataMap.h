#pragma once
#include "BeeBusGlobal.h"

// --- Modbus data map class ---
// Stores and manages local register data with variable binding.
//
// A binding stores the address of the caller's variable and reads or writes
// through it from the receive path for as long as the node runs. The variable
// must therefore outlive the node, or be unbound with the matching remove call
// before it is destroyed. Static, global or member storage is the usual answer;
// a local in a function that returns, or an element of a container that
// reallocates, leaves a live pointer to memory that is gone, and nothing here
// can detect it.
//
// A string is one entry spanning several registers rather than one entry per
// register, so the single-address accessors below are span aware and the wire
// path, which reads and writes register by register, needs no special case.
class BeeBusModbusDataMap {
public:
    // --- Constructor and destructor ---
    BeeBusModbusDataMap();
    ~BeeBusModbusDataMap();

    // Movable, so BeeBusProtocol::adoptApplication() hands a whole map to a
    // new protocol by moving its tables rather than copying them. The
    // user-declared destructor would otherwise leave only the copy.
    BeeBusModbusDataMap(BeeBusModbusDataMap&&) = default;
    BeeBusModbusDataMap& operator=(BeeBusModbusDataMap&&) = default;
    BeeBusModbusDataMap(const BeeBusModbusDataMap&) = delete;
    BeeBusModbusDataMap& operator=(const BeeBusModbusDataMap&) = delete;

    // --- Initialization ---
    void clearAll();

    // --- Register binding ---
    // False if the pointer is null, the address is already bound or covered by
    // a string, or the memory for the binding is not there. There is no other
    // limit: every address of every table can be bound.
    bool addCoil(uint16_t address, bool* variable);
    bool addHreg(uint16_t address, int16_t* variable);
    bool addIsts(uint16_t address, bool* variable);
    bool addIreg(uint16_t address, int16_t* variable);

    // --- Typed values ---
    //
    // Binds a value of any fixed-width wire type over as many registers as it
    // needs (bb_registersOf): a BOOL, SINT or INT takes one, a DINT, DWORD or
    // REAL takes two, an LINT, ULINT, LWORD or LREAL four, high word first. A peer reads and writes the registers;
    // the value is read and written in place. A write of a number the type
    // cannot hold (300 into a USINT) is refused. False on the same grounds as
    // addHreg, for any register of the value, and when the value would run
    // past address 65535. removeHreg / removeIreg on any of its registers
    // unbinds the whole value.
    bool addHregValue(uint16_t address, void* variable, uint8_t type);
    bool addIregValue(uint16_t address, void* variable, uint8_t type);

    // --- String binding ---
    //
    // Text packed two characters to a register, big-endian, NUL terminated. The
    // buffer's last byte is always the terminator, so it carries capacity - 1
    // characters, and it occupies (capacity + 1) / 2 registers. With an odd
    // capacity the last register's second byte lies past the buffer: it reads
    // as 0 and a write to it is dropped. That is what lets an IEC STRING(n),
    // whose buffer is n + 1 bytes, carry all n characters in n / 2 + 1
    // registers.
    //
    // addString binds over holding registers, so peers may write it.
    // addInputString binds over input registers, which are read-only remotely.
    bool addString(uint16_t address, char* buffer, uint16_t capacity);
    bool addInputString(uint16_t address, char* buffer, uint16_t capacity);
    void removeString(uint16_t address);
    void removeInputString(uint16_t address);

    // False if no string starts at this address. Text longer than the span will
    // hold is truncated; the remainder of the buffer is cleared either way.
    bool setString(uint16_t address, const char* text);
    bool setInputString(uint16_t address, const char* text);

    // Copies the text out. False if no string starts at this address or `out`
    // is null; the copy is truncated to fit `size` and always terminated.
    bool getString(uint16_t address, char* out, uint16_t size) const;
    bool getInputString(uint16_t address, char* out, uint16_t size) const;

    // Registers occupied by the string starting exactly at this address, or 0
    // if none starts there.
    uint16_t stringRegisters(uint16_t address) const;
    uint16_t inputStringRegisters(uint16_t address) const;

    // --- Register existence checks ---
    bool hasCoil(uint16_t address) const;
    bool hasHreg(uint16_t address) const;
    bool hasIsts(uint16_t address) const;
    bool hasIreg(uint16_t address) const;

    // --- Register read methods - read from bound variables ---
    bool getCoil(uint16_t address) const;
    int16_t getHreg(uint16_t address) const;
    bool getIsts(uint16_t address) const;
    int16_t getIreg(uint16_t address) const;

    // --- Register write methods - write to bound variables ---
    bool setCoil(uint16_t address, bool value, uint8_t sourceNodeID = 0);
    bool setHreg(uint16_t address, int16_t value, uint8_t sourceNodeID = 0);
    bool setIsts(uint16_t address, bool value);
    bool setIreg(uint16_t address, int16_t value);

    // --- Multiple register operations - operate on bound variables ---
    void getIsts(uint16_t address, bool* values, uint16_t quantity) const;

    // --- Remove register bindings ---
    void removeCoil(uint16_t address);
    void removeHreg(uint16_t address);
    void removeIsts(uint16_t address);
    void removeIreg(uint16_t address);

    // Times this address has been driven to its safe state.
    uint16_t failsafeCount(BeeBusRegisterType table, uint16_t address) const;

    // Writes a peer has made across `count` addresses from `address`, as a
    // running total. A range is one binding to its caller, so one number.
    uint16_t writeCount(BeeBusRegisterType table, uint16_t address,
                        uint16_t count = 1) const;

    // Values a peer has written in the range since `since`, a serial this
    // map keeps: pass 0 the first time, and the same variable afterwards, which
    // this advances. A value over two registers, and a string, count once, at
    // their first register; `firstAddress`, when given, is set to the lowest
    // written. A caller that keeps its own `since` sees every write exactly
    // once, whoever else is asking.
    uint16_t writtenSince(BeeBusRegisterType table, uint16_t address, uint16_t count,
                          uint32_t& since, uint16_t* firstAddress = nullptr) const;

    // --- Change notification ---
    //
    // Handlers are collected while a request is applied and delivered once it
    // has been, by dispatchChangeEvents(), so a handler sees the whole request
    // rather than a half-written one. Only writes from a peer raise them: a
    // local set is the application writing to itself.
    void onCoilChanged(uint16_t address, BeeBusCoilChangedHandler handler);
    void onHregChanged(uint16_t address, BeeBusHregChangedHandler handler);
    void onStringChanged(uint16_t address, BeeBusStringChangedHandler handler);

    // Delivers the changes collected since the last call.
    void dispatchChangeEvents();

    // --- Fail-safe support ---
    // Applies the failsafe action of every register the lost node last wrote.
    void applyFailsafeForNode(uint8_t nodeID);

    // What should happen to an address if the node commanding it goes away.
    // Unset addresses hold their last value.
    void setCoilFailsafe(uint16_t address, BeeBusFailsafe action, bool value);
    void setHregFailsafe(uint16_t address, BeeBusFailsafe action, int16_t value);

    // The same for a string. False if no string starts at this address.
    bool setStringFailsafe(uint16_t address, BeeBusFailsafe action, const char* text);

    // Called once per address driven to its safe value.
    //
    // A failsafe write does NOT raise the change handler for that address:
    // onCoilChanged and onHregChanged report a peer writing a value, and a
    // failsafe is this node acting because a peer stopped. An application that
    // centralises "an output changed, log it and update the display" on the
    // change handler alone will record every commanded change and miss every
    // safety-driven one, which is the wrong way round for an incident review.
    // Register both.
    void onFailsafeApplied(BeeBusFailsafeHandler handler);

private:
    // What the map keeps is three kinds of table, sorted by address, each
    // entry holding a binding and what the protocol records about it. An
    // entry is made when an application binds or configures an address, never
    // by traffic: a peer's write updates the entry that is there. Unbinding
    // keeps an entry's counters (they answer writeCount() and failsafeCount()
    // afterwards) and drops the rest.
    enum : uint8_t {
        F_FAILSAFE    = 0x01,   // A safe value is set (ResetToValue)
        F_FAILSAFE_ON = 0x02,   // ...and for a coil it is TRUE
    };

    // A coil or discrete input. The counters and failsafe are coils' only.
    struct BitEntry {
        uint16_t address;
        uint8_t lastWriter;       // The peer that last wrote it, 0 = none
        uint8_t flags;            // F_*
        bool* var;                // The bound variable, nullptr = not bound
        uint16_t writeCount;
        uint16_t failsafeCount;
        uint32_t writeSerial;     // _writeSerial at its last write, 0 = never
    };

    // One register of a value. A value over two registers has an entry for
    // each, the second with word 1. The holding table also keeps a string's
    // counters, on the string's first address.
    struct WordEntry {
        uint16_t address;
        uint8_t type;             // BEEBUS_T_*
        uint8_t word;             // Which register of the value this is, 0 = first
        void* var;                // The value's variable, nullptr = not bound
        uint8_t lastWriter;
        uint8_t flags;
        int16_t failsafe;
        uint16_t writeCount;      // Counted on a value's first register
        uint16_t failsafeCount;
        uint32_t writeSerial;
    };

    // A string: one entry spanning its registers.
    struct StringEntry {
        uint16_t address;
        uint16_t regCount;        // Registers it occupies: (capacity + 1) / 2
        uint16_t capacity;        // Bytes in the caller's buffer
        uint8_t lastWriter;
        uint8_t flags;
        char* buf;
        char* failsafe;           // The safe text, owned, while F_FAILSAFE
    };

    typedef BeeBusAddrTable<BitEntry> BitTable;
    typedef BeeBusAddrTable<WordEntry> WordTable;
    typedef BeeBusAddrTable<StringEntry> StringTable;

    BitTable _coils;
    BitTable _ists;
    WordTable _hregs;
    WordTable _iregs;
    StringTable _strings;
    StringTable _inputStrings;

    uint32_t _writeSerial = 0;

    static BitEntry* boundBit(BitTable& table, uint16_t address);
    static const BitEntry* boundBit(const BitTable& table, uint16_t address);
    static WordEntry* boundWord(WordTable& table, uint16_t address);
    static const WordEntry* boundWord(const WordTable& table, uint16_t address);

    bool bindBit(BitTable& table, uint16_t address, bool* variable);
    bool bindValue(WordTable& regs, const StringTable& spans, uint16_t address,
                   void* variable, uint8_t type);
    static uint16_t valueStart(const WordTable& regs, uint16_t address);
    // Unbinds the value covering `address`; returns its first register. With
    // `keepRecords` the counters stay (holding registers).
    uint16_t unbindValue(WordTable& regs, uint16_t address, bool keepRecords);
    // Drops what an unbound entry no longer needs, and the entry once nothing
    // about it is left.
    static void tidyBit(BitTable& table, uint16_t address);
    static void tidyWord(WordTable& table, uint16_t address);

    static StringEntry* findSpan(StringTable& spans, uint16_t address);
    static const StringEntry* findSpan(const StringTable& spans, uint16_t address);
    static bool spansCollide(const StringTable& spans, uint16_t address, uint16_t regCount);
    static bool rangeIsBound(const WordTable& bound, uint16_t address, uint16_t regCount);
    bool bindString(StringTable& spans, const WordTable& scalars,
                    uint16_t address, char* buffer, uint16_t capacity);
    void unbindString(StringTable& spans, uint16_t address);

    static bool writeSpan(const StringEntry& span, const char* text);
    static bool readSpan(const StringEntry& span, char* out, uint16_t size);
    static int16_t spanRegister(const StringEntry& span, uint16_t offset);
    static bool setSpanRegister(const StringEntry& span, uint16_t offset, int16_t value);

    // --- Fail-safe ---
    BeeBusFailsafeHandler _failsafeHandler;
    // The addresses one applyFailsafeForNode() drove, reported once all are
    // done. Room for every failsafe configured, taken as each is set.
    struct Applied {
        BeeBusRegisterType table;
        uint16_t address;
    };
    BeeBusList<Applied> _applied;
    uint16_t _failsafeSettings = 0;
    void roomForFailsafe();

    // --- Change notification ---
    //
    // A handler per watched address. Few, so a short list searched in turn.
    template <typename H>
    struct Watch {
        uint16_t address;
        H handler;
    };
    BeeBusList<Watch<BeeBusCoilChangedHandler> > _coilChanged;
    BeeBusList<Watch<BeeBusHregChangedHandler> > _hregChanged;
    BeeBusList<Watch<BeeBusStringChangedHandler> > _stringChanged;
    template <typename H>
    static void watch(BeeBusList<Watch<H> >& list, uint16_t address, H handler);
    template <typename H>
    static const H* watcher(const BeeBusList<Watch<H> >& list, uint16_t address);

    // Changes waiting for dispatchChangeEvents(). The handler runs from loop()
    // rather than from the receive path, so it can do anything. One per watched
    // address at most, so the room is taken as each watch is set.
    enum class ChangeKind : uint8_t { Coil, Hreg, String };
    struct PendingChange {
        ChangeKind kind;
        uint16_t address;
        uint8_t fromNodeID;
    };
    BeeBusList<PendingChange> _pendingChanges;
    // While a dispatch runs, the changes before this index are being reported;
    // those after it are for the next one.
    size_t _dispatching = 0;
    void roomForChanges();
    void noteChange(ChangeKind kind, uint16_t address, uint8_t fromNodeID);
    bool isWatched(ChangeKind kind, uint16_t address) const;
};
