#pragma once
#include "BeeBusGlobal.h"

#if defined(BEEBUS_PLATFORM_HOST)
#include <mutex>
#endif

/**
 * Serialises one node's public surface against its own loop().
 *
 * The API is safe to call whenever suits the caller -- work is queued and
 * carried out on a later turn, so nothing depends on when a call is made. On a
 * single-threaded board that is the whole story.
 *
 * A host runtime may drive one node from several threads. OpenPLC gives each
 * IEC task its own thread and runs their bodies in parallel, so servicing the
 * node from a fast task while the application blocks sit in a slower one puts
 * two threads on one node: one appending to the operation queue while the
 * other walks it. That is a torn read or a reallocation under the other
 * thread's feet, not a stale value.
 *
 * What the guard covers is the API calls themselves. Bound variables, and the
 * text a call such as topicText() points into, are written by loop() without
 * it, so a thread other than the one running loop() reads them at its own
 * risk. The debug output is shared by every node in the process.
 *
 * Recursive, because the guarantee has to survive the callbacks. loop() raises
 * completion and topic handlers, and a handler is expressly allowed to queue
 * more work -- that call re-enters the API on the thread already holding the
 * lock, and a plain mutex would deadlock on the spot. A handler must not call
 * end() or rebind(), though: either frees the node the handler is being called
 * from.
 *
 * Embedded targets compile the guard out: one thread, no cost, no <mutex>.
 */
#if defined(BEEBUS_PLATFORM_HOST)
    #define BEEBUS_GUARD() std::lock_guard<std::recursive_mutex> mbGuard_(_lock)
#else
    #define BEEBUS_GUARD() ((void)0)
#endif

#if defined(BEEBUS_PLATFORM_ARDUINO)
// Declared only for the convenience overload below. The definition is needed
// solely in BeeBusAPI.cpp, so the core stays free of Arduino headers.
class Stream;
#endif

/**
 * BeeBusAPI - one BeeBus node. The application creates an instance per node;
 * several can run side by side, each on its own transport.
 */
class BeeBusAPI {
public:
    BeeBusAPI();
    ~BeeBusAPI();

    // --- Protocol management ---

    // Starts the protocol on any transport. The caller keeps ownership of the
    // transport, which must outlive this instance. The config belongs to this
    // instance, so several nodes in one process can be tuned independently.
    bool begin(BeeBusTransport* transport, uint8_t nodeID = 1,
               const BeeBusConfig& config = BeeBusConfig());

#if defined(BEEBUS_PLATFORM_ARDUINO)
    // Arduino convenience overload, wrapping a Stream in a transport owned by
    // this instance. Pass the same baud rate the sketch gave Serial.begin():
    // it is the one property of the line a node cannot measure for itself, and
    // with it the gap, the response window and a joining node's announcement
    // are all sized from the rate rather than guessed at. Zero keeps the
    // measure-it-yourself behaviour.
    bool begin(Stream* serial, uint8_t nodeID = 1,
               const BeeBusConfig& config = BeeBusConfig(),
               uint32_t baudRate = 0);
#endif

    // CHANGES THE NODE ID OR THE LINE WHILE RUNNING, keeping everything the
    // application bound: registers, topics, subscriptions, fail-safes,
    // handlers. For a settings change - an HMI that moves this node to
    // another ID, a baud rate or a serial port changed from a panel - without
    // a restart and without every block binding its variables again.
    //
    // The node leaves the old ring as abruptly as a cable pulled (call
    // disconnect() and give it a moment first for a graceful departure), and
    // joins on the new transport under the new ID from scratch: connect()
    // afterwards as after begin(). Every operation still owed on the old ring
    // concludes Cancelled before this returns. The transport is the caller's
    // and must outlive this instance, as for begin(); a transport begin()
    // made for itself is closed and freed. False - and nothing changed - for
    // an ID out of range, a transport that will not open, or before begin().
    // `config`, when given, replaces the configuration; otherwise the one in
    // force is kept. Not from inside a handler, for the same reason as end().
    bool rebind(BeeBusTransport* transport, uint8_t nodeID,
                const BeeBusConfig* config = nullptr);
#if defined(BEEBUS_PLATFORM_ARDUINO)
    // The Stream form, as begin(Stream*): the transport over it is this
    // instance's own.
    bool rebind(Stream* serial, uint8_t nodeID, uint32_t baudRate = 0,
                const BeeBusConfig* config = nullptr);
#endif

    // Why the last begin() or rebind() returned false when the reason was
    // memory - "out of memory for the node", "... for the buffers and queues",
    // "... for the link" - so the application can say so rather than retry a
    // board that will never have room. Null otherwise (an ID out of range or a
    // transport that will not open: the transport says why).
    const char* startError() const { return _startError; }

    void loop();
    // Stops the node and frees it. Not from inside a handler: the handler is
    // running on the node this frees.
    void end();
    bool isInitialized();

    // Node management
    uint8_t getNodeID();
    bool isConnected();
    bool isMember(uint8_t nodeID);

    // True while this node is stood down because another node is transmitting
    // under its ID. Without it a commissioning mistake looks exactly like a
    // node that has not finished starting: isConnected() is false and
    // isMember() finds nobody. Reads and writes are refused DuplicateNodeID
    // meanwhile. Clears once the address has been quiet long enough, or on the
    // next connect().
    bool hasDuplicateNodeID();

    void connect();
    void disconnect();

    // True from disconnect() until the departure has gone out and the node is
    // offline. A caller about to rebind() waits for it, so the ring hears the
    // node leave rather than timing it out.
    bool isLeaving();

    // --- Local data map ---

    // Binds a variable to an address. False if the pointer is null, the address
    // is already bound, or memory for the binding cannot be allocated.
    bool addCoil(uint16_t address, bool* variable);
    bool addHreg(uint16_t address, int16_t* variable);
    bool addIsts(uint16_t address, bool* variable);
    bool addIreg(uint16_t address, int16_t* variable);

    // Update local data map values
    bool setCoil(uint16_t address, bool value);
    bool setHreg(uint16_t address, int16_t value);
    bool setIsts(uint16_t address, bool value);
    bool setIreg(uint16_t address, int16_t value);

    // Read local data map values
    bool getCoil(uint16_t address, bool& value);
    bool getHreg(uint16_t address, int16_t& value);
    bool getIsts(uint16_t address, bool& value);
    bool getIreg(uint16_t address, int16_t& value);

    // --- Binding with the type given at runtime ---
    //
    // The same work the templates above do, for a caller that holds the type as
    // a value rather than as T. Everything else is identical. A text topic
    // (BEEBUS_T_STRING) gives the buffer as `variable` and its size as
    // `textCapacity`.

    bool addTopicRaw(uint16_t topicID, uint8_t type, void* variable, bool writable,
                     const char* name, uint16_t textCapacity = 0);
    bool registerTopicRaw(uint8_t nodeID, uint16_t topicID, uint8_t type, void* variable,
                          const char* name, uint16_t textCapacity = 0);
    BeeBusResult writeTopicRaw(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value,
                               BeeBusDelivery delivery, bool wantResult = false);

    // Points an already-bound topic at different storage, for a caller whose
    // variable has moved. The type and the name are unchanged.
    bool rebindTopic(uint16_t topicID, void* variable);
    bool rebindRemoteTopic(uint8_t nodeID, uint16_t topicID, void* variable);

    // Binds one value in a table. A bit table takes a BOOL at one address. A
    // register table takes any fixed-width type at its own width, from
    // `address` on: BOOL, I8, U8, I16 and U16 take one register, I32, U32 and
    // F32 take two, I64, U64 and F64 four, high word first (bb_registersOf).
    // Refuses a type the table cannot hold, and a value any of whose registers
    // is already bound.
    bool addRegister(BeeBusRegisterType table, uint16_t address, void* variable, uint8_t type);
    bool removeRegister(BeeBusRegisterType table, uint16_t address);

    // --- Failsafe ---
    //
    // What an output should do if the node commanding it stops answering.
    // Requires BeeBusConfig::enableFailSafe; without a safe state stated, an
    // address holds whatever the lost node left it at.
    //
    //     beebus.setCoilFailsafe(10, BeeBusFailsafe::ResetToValue, false);
    //     beebus.setHregFailsafe(40, BeeBusFailsafe::ResetToValue, 500);
    //     beebus.setHregFailsafe(41, BeeBusFailsafe::HoldLastValue);

    // Times this address has been driven to its safe state, as a running total.
    uint16_t failsafeCount(BeeBusRegisterType table, uint16_t address);

    // Writes a peer has made across `count` addresses from `address`, as a
    // running total.
    uint16_t registerWrites(BeeBusRegisterType table, uint16_t address,
                            uint16_t count = 1);

    // Which values in a range a peer has written since you last asked. Keep a
    // uint32_t per caller, start it at 0, and pass it every time: the call
    // advances it. Returns how many values were written - a two-register value
    // or a string counts once - and sets `firstAddress` to the lowest one's
    // first register. Polled, like registerWrites, so it needs no handler.
    //
    //     uint32_t seen = 0;
    //     beebus.registerWritesSince(MB_HOLDING_REGISTER, 100, 40, seen);  // prime
    //     ...
    //     uint16_t first;
    //     if (beebus.registerWritesSince(MB_HOLDING_REGISTER, 100, 40, seen, &first)) {
    //         validateSetting(first);
    //     }
    uint16_t registerWritesSince(BeeBusRegisterType table, uint16_t address,
                                 uint16_t count, uint32_t& since,
                                 uint16_t* firstAddress = nullptr);

    void setCoilFailsafe(uint16_t address, BeeBusFailsafe action, bool value = false);
    void setHregFailsafe(uint16_t address, BeeBusFailsafe action, int16_t value = 0);

    // Remove data points
    bool removeCoil(uint16_t address);
    bool removeHreg(uint16_t address);
    bool removeIsts(uint16_t address);
    bool removeIreg(uint16_t address);

    // --- Strings ---
    //
    // Text packed two characters to a register, big-endian and NUL terminated,
    // so it travels as ordinary holding or input registers and a Modbus gateway
    // needs to know nothing about it. A buffer of `capacity` bytes occupies
    // (capacity + 1) / 2 registers and carries capacity - 1 characters, its
    // last byte always being the terminator.
    //
    //     char message[64];
    //     beebus.addString(500, message, sizeof(message));
    //     beebus.setString(500, "node 1 fault code 12");
    //
    // addString binds over holding registers, so peers may write it.
    // addInputString binds over input registers, which peers may only read.
    // False if the pointer is null, the capacity is under two bytes, the range
    // collides with an existing binding, or memory for the binding cannot be
    // allocated.
    bool addString(uint16_t address, char* buffer, uint16_t capacity);
    bool addInputString(uint16_t address, char* buffer, uint16_t capacity);
    bool removeString(uint16_t address);
    bool removeInputString(uint16_t address);

    // Local access. Text too long for the field is truncated.
    bool setString(uint16_t address, const char* text);
    bool setInputString(uint16_t address, const char* text);
    bool getString(uint16_t address, char* out, uint16_t size);
    bool getInputString(uint16_t address, char* out, uint16_t size);

    // Fetches a peer's string into your buffer. `size` is how many bytes to
    // read, so it must fit your buffer and must not exceed the peer's field:
    // reading `size / 2` registers the peer has not bound is refused by the
    // peer, exactly as a block read past the end of a register range is. Over
    // 250 bytes is refused at the call, being past what one read carries.
    //
    // Completion arrives through onReadComplete, because an empty message and
    // one that has not landed yet look the same in the buffer.
    BeeBusResult readString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size);
    BeeBusResult readInputString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size);

    // Sends text to a peer's string. Costs the length of the message rather
    // than the capacity of the field it lands in, so publishing a short fault
    // into a large field is cheap. Refused with InvalidArgument beyond 245
    // characters, which is what one write carries; a split string could not be
    // delivered atomically.
    BeeBusResult writeString(uint8_t nodeID, uint16_t address, const char* text);
    BeeBusResult writeString(uint8_t nodeID, uint16_t address, const char* text,
                             BeeBusDelivery delivery);

    // What the string should say if the node that last wrote it goes away,
    // rather than holding a stale message. False if no string is bound here.
    bool setStringFailsafe(uint16_t address, BeeBusFailsafe action, const char* text = nullptr);

    // --- Arrays, sized from the array type ---

    // AUTO-SIZING Read functions (template-based for fixed arrays)
    template<size_t N>
    BeeBusResult readHreg(uint8_t nodeID, uint16_t offset, int16_t (&values)[N], uint8_t fc = 0) {
        return readHreg_impl(nodeID, offset, values, N, fc);
    }

    template<size_t N>
    BeeBusResult readCoil(uint8_t nodeID, uint16_t offset, bool (&values)[N], uint8_t fc = 0) {
        return readCoil_impl(nodeID, offset, values, N, fc);
    }

    template<size_t N>
    BeeBusResult readIreg(uint8_t nodeID, uint16_t offset, int16_t (&values)[N], uint8_t fc = 0) {
        return readIreg_impl(nodeID, offset, values, N, fc);
    }

    template<size_t N>
    BeeBusResult readIsts(uint8_t nodeID, uint16_t offset, bool (&values)[N], uint8_t fc = 0) {
        return readIsts_impl(nodeID, offset, values, N, fc);
    }

    // AUTO-SIZING Write functions (template-based for fixed arrays)
    template<size_t N>
    BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, const int16_t (&values)[N], uint8_t fc = 0) {
        return writeHreg_impl(nodeID, offset, values, N, fc, BeeBusDelivery::Unconfirmed);
    }

    template<size_t N>
    BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, const int16_t (&values)[N],
                           BeeBusDelivery delivery) {
        return writeHreg_impl(nodeID, offset, values, N, 0, delivery);
    }

    template<size_t N>
    BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, const bool (&values)[N], uint8_t fc = 0) {
        return writeCoil_impl(nodeID, offset, values, N, fc, BeeBusDelivery::Unconfirmed);
    }

    template<size_t N>
    BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, const bool (&values)[N],
                           BeeBusDelivery delivery) {
        return writeCoil_impl(nodeID, offset, values, N, 0, delivery);
    }

    // --- Arrays, sized explicitly ---

    // Manual read functions (pointer-based for dynamic arrays)
    // `stride` is the gap in bytes between the caller's elements. 0 means
    // packed, which is what a plain array is; give it a value when the elements
    // are spaced more widely than their own width.
    BeeBusResult readHregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numregs, uint8_t fc = 0, uint16_t stride = 0);
    BeeBusResult readCoilManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numcoils, uint8_t fc = 0, uint16_t stride = 0);
    BeeBusResult readIregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numiregs, uint8_t fc = 0, uint16_t stride = 0);
    BeeBusResult readIstsManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numists, uint8_t fc = 0, uint16_t stride = 0);

    // Manual write functions (pointer-based for dynamic arrays)
    BeeBusResult writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values, uint16_t numregs, uint8_t fc = 0, uint16_t stride = 0);
    BeeBusResult writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values, uint16_t numcoils, uint8_t fc = 0, uint16_t stride = 0);

    // --- Single values ---

    // Single value read functions (reference-based)
    BeeBusResult readHreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc = 0);
    BeeBusResult readCoil(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc = 0);
    BeeBusResult readIreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc = 0);
    BeeBusResult readIsts(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc = 0);

    // Single value write functions (value-based)
    BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, int16_t value, uint8_t fc = 0);
    BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, bool value, uint8_t fc = 0);

    // --- Confirmed writes ---
    //
    // A confirmed write stays queued until the destination acknowledges it,
    // retries if it does not, and reports failure through the error handler.
    // Use it for setpoints, commands and interlocks. Leave periodic telemetry
    // unconfirmed: the next update is along shortly and an ack per reading is
    // traffic the bus does not need.
    //
    //     beebus.writeHreg(2, 100, setpoint, BeeBusDelivery::Confirmed);

    BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, int16_t value,
                           BeeBusDelivery delivery);
    BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, bool value,
                           BeeBusDelivery delivery);

    BeeBusResult writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values,
                                 uint16_t numregs, BeeBusDelivery delivery, uint16_t stride = 0);
    BeeBusResult writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values,
                                 uint16_t numcoils, BeeBusDelivery delivery, uint16_t stride = 0);

    // Called when a write concludes. A confirmed write reports Immediate when
    // the destination acknowledged it, otherwise a failure result; an
    // unconfirmed one reports Immediate once it is transmitted. Topic writes
    // made without `wantResult` report here too. Applies to writes issued after
    // it is set.
    void onWriteComplete(std::function<void(uint8_t nodeID, uint16_t address, BeeBusResult result)> handler);

    // Called when a read concludes: Immediate once the value is in your
    // variable, Timeout if it never arrived, NodeLost if the peer left, or the
    // reason it was refused. Applies to reads issued after it is set.
    //
    // Without it a read can only be judged by watching its own result variable
    // change, which cannot distinguish a value that has not arrived from one
    // that arrived unchanged.
    void onReadComplete(std::function<void(uint8_t nodeID, uint16_t address, BeeBusResult result)> handler);

    // --- Events ---
    //
    // Raised when a peer writes a new value into this node's data map, once the
    // whole request has been applied and only when the value actually changed.
    // A string raises one event for the message, not one per register. A local
    // set raises nothing: that is the application writing to itself.
    //
    //     beebus.onStringChanged(500, [](uint16_t, const char* text, uint8_t from) {
    //         log(text);
    //     });
    //
    // Handlers run inside loop() while the frame that carried the write is
    // being processed, so they delay this node's token pass. Keep them short.
    void onCoilChanged(uint16_t address, BeeBusCoilChangedHandler handler);
    void onHregChanged(uint16_t address, BeeBusHregChangedHandler handler);
    void onStringChanged(uint16_t address, BeeBusStringChangedHandler handler);

    // Called once per address driven to its safe value after the node
    // commanding it left the network.
    void onFailsafeApplied(BeeBusFailsafeHandler handler);

    // Called when a peer enters or leaves this node's ring.
    void onMemberJoined(BeeBusMemberHandler handler);
    void onMemberLeft(BeeBusMemberHandler handler);

    // =========================================================================
    // TOPICS
    // =========================================================================
    //
    // A topic is a value on the node that owns it, addressed on the wire as
    // (nodeID, topicID). The type comes from the bound variable and travels
    // with every value.
    //
    // A topic is named where it is bound, and named in every call afterwards.
    // The ID is what goes on the wire.
    //
    //     owner.addTopic("analogValue", 1, &analogValue);
    //     watcher.registerTopic(OWNER, "analogValue", 1, &remoteValue);
    //     watcher.subscribe(OWNER, "analogValue", 0.5f, 60000);
    //
    // Bind fixed-width types: bool, int8_t, uint8_t, int16_t, uint16_t,
    // int32_t, uint32_t, int64_t, uint64_t, float, double. Not int, which is 16
    // bit on some targets and 32 bit on others. Text topics bind a character
    // buffer instead (see Text, below).
    //
    // Every named call has an ID-only form beside it, for a build with
    // BEEBUS_TOPIC_NAMES=0 or for a topic that needs no name.
    //
    // Topic IDs run BEEBUS_TOPIC_ID_MIN to BEEBUS_TOPIC_ID_MAX, 1 to 32767.

    // --- Topics this node owns ---
    //
    // addTopic binds one peers may read and write; addInputTopic binds one they
    // may only read. `name` must outlive the binding, so pass a literal.
#if BEEBUS_TOPIC_NAMES
    template <typename T>
    bool addTopic(const char* name, uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return addTopicRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, true, name);
    }

    template <typename T>
    bool addInputTopic(const char* name, uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return addTopicRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, false, name);
    }
#endif

    template <typename T>
    bool addTopic(uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return addTopicRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, true, nullptr);
    }

    template <typename T>
    bool addInputTopic(uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return addTopicRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, false, nullptr);
    }

    // --- Text ---
    //
    // Bind a character buffer and a topic carries text: a fault message, an
    // identifier, a state name. The length travels on the wire, so nothing at
    // the far end needs to know how long the value is, which is the whole
    // reason this is a topic type and not a run of registers.
    //
    //     char status[32] = "";
    //     beebus.addInputTopic("status", 20, status, sizeof(status));
    //     strcpy(status, "calibrating");
    //
    // The buffer is the caller's, bound by its address and capacity like a
    // string register (addString), and must outlive the binding. Text arriving
    // from a peer is cut to capacity - 1 and always terminated; the node
    // allocates nothing for it. An Arduino String reaches any call below that
    // takes text as myString.c_str().
    //
    // Up to BEEBUS_TOPIC_STRING_MAX characters travel; a longer value is
    // carried as far as it goes rather than not arriving at all.
#define BEEBUS_TEXT_TOPICS 1
#if BEEBUS_TOPIC_NAMES
    bool addTopic(const char* name, uint16_t topicID, char* text, uint16_t capacity);
    bool addInputTopic(const char* name, uint16_t topicID, char* text, uint16_t capacity);
    bool registerTopic(uint8_t nodeID, const char* name, uint16_t topicID,
                       char* text, uint16_t capacity);
#endif
    bool addTopic(uint16_t topicID, char* text, uint16_t capacity);
    bool addInputTopic(uint16_t topicID, char* text, uint16_t capacity);
    bool registerTopic(uint8_t nodeID, uint16_t topicID, char* text, uint16_t capacity);

    bool removeTopic(uint16_t topicID);

    // Reserves room for a known number of topics. Each insert keeps the map
    // sorted, so adding thousands one at a time is thousands of moves.
    void reserveTopics(uint16_t localCount, uint16_t remoteCount = 0);

    // --- Topics belonging to a peer ---
    //
    // Binds a peer's topic to a variable of yours. A value arriving for a topic
    // nothing is bound to is dropped: the binding states the expected type.
#if BEEBUS_TOPIC_NAMES
    template <typename T>
    bool registerTopic(uint8_t nodeID, const char* name, uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return registerTopicRaw(nodeID, topicID, BeeBusTypeOf<T>::value,
                                (void*)variable, name);
    }
#endif

    template <typename T>
    bool registerTopic(uint8_t nodeID, uint16_t topicID, T* variable) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return registerTopicRaw(nodeID, topicID, BeeBusTypeOf<T>::value,
                                (void*)variable, nullptr);
    }

    bool unregisterTopic(uint8_t nodeID, uint16_t topicID);

    // --- Publishing ---
    //
    // Two ways out, and they are separate pipelines on purpose.
    //
    // publishAll sends the value ONCE, addressed to the whole ring, and every
    // node holding that topic takes it. One send whatever the number of
    // listeners, which is what a node that wakes, reports and sleeps again
    // wants: one turn on the bus instead of one per listener. Addressed to the
    // ring rather than to a list because both transports are shared — every
    // node's receiver already has the bytes whatever the section says, so a
    // copy per listener buys no delivery and costs a copy.
    //
    // A broadcast carries no stream. A sequence number belongs to one publisher
    // counting to one destination, which is what lets a subscriber on a
    // deadband be sent one sample in six without reading five as losses; a
    // frame addressed to the ring has no destination of its own, so the
    // receiver neither measures it against a stream nor records it into one.
    // That is what keeps this apart from the subscription pipeline: a topic may
    // be broadcast AND published to its subscribers on their own terms, and
    // neither disturbs the other. Nothing is given up, because a broadcast is
    // unacknowledged by definition and had no loss to detect.
    //
    // Nothing acknowledges a broadcast, so Confirmed is refused with
    // InvalidArgument rather than waiting for an acknowledgement that cannot
    // come. A command that has to be confirmed names its peer: publishTo sends
    // to one node, subscribed or not, on that node's own stream, and completes
    // on the acknowledgement — the same path writeTopic uses, so a command can
    // be published rather than written into a peer's topic.
    //
    // On a buffered topic both record into the ring instead.
    BeeBusResult publishAll(uint16_t topicID,
                            BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
    BeeBusResult publishTo(uint8_t nodeID, uint16_t topicID,
                           BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);

    // Asks a peer for a value now. It lands in the registered variable;
    // completion arrives through onReadComplete, where `address` is the topic.
    BeeBusResult readTopic(uint8_t nodeID, uint16_t topicID);

    // Writes into a peer's topic. Refused with TypeMismatch if it is not the
    // type the peer bound, or Refused if the peer made it read-only.
    //
    // Confirmed (Reliable) is acknowledged and retried, and the owner applies
    // it once however often it arrives. `wantResult` asks the owner what it
    // did with it: ACCEPTED, CHANGED or REFUSED and the value it kept, through
    // onTopicResult. A bind does this for you.
    template <typename T>
    BeeBusResult writeTopic(uint8_t nodeID, uint16_t topicID, T value,
                            BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                            bool wantResult = false) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return writeTopicRaw(nodeID, topicID, bb_scalarOf<T>(value), delivery, wantResult);
    }

    // Writes text into a peer's string topic. Refused with InvalidArgument
    // beyond BEEBUS_TOPIC_STRING_MAX characters, which is what one PDU carries.
    BeeBusResult writeTopic(uint8_t nodeID, uint16_t topicID, const char* text,
                            BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                            bool wantResult = false);

    // The text of a bound string topic, ours or a peer's. "" when the topic is
    // not bound, is not a string, or nothing has arrived.
    const char* topicText(uint16_t topicID);
    const char* topicText(uint8_t nodeID, uint16_t topicID);

    // --- Subscriptions ---
    //
    // Asks a peer to publish when the value moves by `deadband` in the value's
    // own units, and in any case every `maxIntervalMs`. A deadband of 0 reports
    // every change; an interval of 0 makes no periodic promise.
    //
    // The terms belong to the subscription, so two subscribers may hold
    // different ones for the same topic.
    //
    // QoS Live (the default) is best effort: newest wins, a deadband applies.
    // Reliable delivers every change: what a lost frame took is noticed in the
    // next rotation and sent again as the current value, so a value is never
    // left blank or old. The first value travels with the answer either way.
    // The deadband does not apply to Reliable.
    //
    // A subscription that finds no room in the queue is held and sent as room
    // frees, so everything can be subscribed at once.
    BeeBusResult subscribe(uint8_t nodeID, uint16_t topicID, float deadband = 0.0f,
                           bb_time_t maxIntervalMs = 0,
                           BeeBusQoS qos = BeeBusQoS::Live);
    BeeBusResult unsubscribe(uint8_t nodeID, uint16_t topicID);

    // --- Commissioning ---
    //
    // fetchTopics walks a peer's whole list, a page per token, reporting each
    // entry through onTopicList. publishTopicList pushes ours the other way.
    BeeBusResult fetchTopics(uint8_t nodeID, bool withNames = true);
    BeeBusResult publishTopicList(uint8_t nodeID, bool withNames = true);
    BeeBusResult registerTopicWith(uint8_t nodeID, uint16_t topicID);

    // --- Values and freshness ---
    bool getTopic(uint16_t topicID, BeeBusScalar& value);
    bool getRemoteTopic(uint8_t nodeID, uint16_t topicID, BeeBusScalar& value);

    // Whether a peer's value can be acted on, judged against the interval its
    // subscription asked for.
    BeeBusQuality topicQuality(uint8_t nodeID, uint16_t topicID);

    // How long ago the value arrived, back-dated by the age a buffered sample
    // carried. BEEBUS_SILENCE_NEVER if none ever has.
    bb_time_t topicAge(uint8_t nodeID, uint16_t topicID);

    // How many publishes this node has taken for a peer's topic, and how many
    // the sequence says never arrived. False when the topic is not bound here.
    bool topicStream(uint8_t nodeID, uint16_t topicID, BeeBusTopicStream& out);

    // Writes a peer has made to a topic this node owns, as a running total.
    uint16_t topicWrites(uint16_t topicID);

    // Queue a writable topic's writes instead of storing them: a peer's write
    // then only reaches onTopicWritten / onTopicStringWritten, and the variable
    // changes when the program takes it. False when the topic is not bound here.
    bool setTopicQueued(uint16_t topicID, bool queued);

    // How long since anything was heard from a node, and whether it is taking
    // part in the ring now.
    bb_time_t silenceFor(uint8_t nodeID);
    bool isAlive(uint8_t nodeID);

    // --- Store and forward ---
    //
    // Holds up to `depth` samples while there is nowhere to send them, oldest
    // overwritten by newest, and releases them in order with their ages
    // attached. Depth 0 turns it off; the cap is BEEBUS_TOPIC_BUFFER_MAX.
    //
    // Every publish of a buffered topic goes through the ring, network up or
    // down, so a live value cannot arrive ahead of the history behind it.
    bool bufferTopic(uint16_t topicID, uint8_t depth);
    uint16_t bufferedCount(uint16_t topicID);

    // --- Enumeration ---
    uint16_t topicCount();
    bool topicAt(uint16_t index, BeeBusTopicEntry& entry);
    uint16_t discoveredCount(uint8_t nodeID);
    bool discoveredAt(uint8_t nodeID, uint16_t index, BeeBusTopicEntry& entry);

#if BEEBUS_TOPIC_NAMES
    // --- By name ---
    //
    // A name resolves against what this node bound by hand first, then against
    // whatever fetchTopics discovered. NotMapped if it resolves to nothing.
    BeeBusResult publishAll(const char* name,
                         BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
    BeeBusResult publishTo(uint8_t nodeID, const char* name,
                           BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
    BeeBusResult readTopic(uint8_t nodeID, const char* name);
    BeeBusResult subscribe(uint8_t nodeID, const char* name, float deadband = 0.0f,
                           bb_time_t maxIntervalMs = 0,
                           BeeBusQoS qos = BeeBusQoS::Live);
    BeeBusResult unsubscribe(uint8_t nodeID, const char* name);
    BeeBusResult registerTopicWith(uint8_t nodeID, const char* name);

    bool removeTopic(const char* name);
    bool unregisterTopic(uint8_t nodeID, const char* name);
    bool getTopic(const char* name, BeeBusScalar& value);
    bool getRemoteTopic(uint8_t nodeID, const char* name, BeeBusScalar& value);
    BeeBusQuality topicQuality(uint8_t nodeID, const char* name);
    bb_time_t topicAge(uint8_t nodeID, const char* name);
    bool bufferTopic(const char* name, uint8_t depth);
    uint16_t bufferedCount(const char* name);

    template <typename T>
    BeeBusResult writeTopic(uint8_t nodeID, const char* name, T value,
                            BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                            bool wantResult = false) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        const uint16_t topicID = remoteTopicIdOf(nodeID, name);
        if (topicID == BEEBUS_TOPIC_NONE) return BeeBusResult::NotMapped;
        return writeTopicRaw(nodeID, topicID, bb_scalarOf<T>(value), delivery, wantResult);
    }

    BeeBusResult writeTopic(uint8_t nodeID, const char* name, const char* text,
                            BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                            bool wantResult = false);

    const char* topicText(const char* name);
    const char* topicText(uint8_t nodeID, const char* name);

    // Subscribes to every topic of a peer whose name matches a glob: `*` for
    // any run of characters, `?` for one. Requires a completed fetchTopics for
    // that node and returns NotMapped without one.
    BeeBusResult subscribeAll(uint8_t nodeID, const char* pattern, float deadband = 0.0f,
                              bb_time_t maxIntervalMs = 0,
                              BeeBusQoS qos = BeeBusQoS::Live);

    // The ID a name resolves to, or BEEBUS_TOPIC_NONE.
    uint16_t topicIdOf(const char* name);
    uint16_t remoteTopicIdOf(uint8_t nodeID, const char* name);
#endif

    // --- Topic events ---
    //
    // onTopicChanged: a peer published a topic registered from it.
    // onTopicWritten: a peer wrote a topic this node owns.
    // onTopicGap: publishes went missing, naming how many.
    // onTopicList: one entry of a peer's list arrived.
    // onTopicStringChanged / onTopicStringWritten: the same two for a string
    // topic, carrying the text. Use these rather than reading the bound
    // variable: a store-and-forward flush raises the handler for every buffered
    // entry but writes the variable only for the newest.
    void onTopicChanged(BeeBusTopicChangedHandler handler);
    void onTopicWritten(BeeBusTopicWrittenHandler handler);
    void onTopicStringChanged(BeeBusTopicStringChangedHandler handler);
    void onTopicStringWritten(BeeBusTopicStringWrittenHandler handler);
    void onTopicGap(BeeBusTopicGapHandler handler);

    // Outcome of every topic operation this node issues — publish, read, write,
    // subscribe. The counterpart of onReadComplete and onWriteComplete for the
    // register operations. A topic write reports through onWriteComplete
    // instead when that handler is set, and through onTopicResult when made
    // with `wantResult`.
    //
    // An unconfirmed operation completes Immediate when it reaches the wire,
    // which is all it ever promised. A confirmed one completes when it is
    // acknowledged, or Timeout when it is not; a read completes when its reply
    // arrives. Set it before connect(): the protocol captures the handler as
    // each operation is accepted, so one installed later misses everything
    // already in flight.
    void onTopicComplete(BeeBusTopicCompleteHandler handler);
    void onTopicList(BeeBusTopicListHandler handler);

    // What an owner decided about a write made with `wantResult`.
    void onTopicResult(BeeBusTopicResultHandler handler);

    // The same, polled: the last decision on a write of ours to that topic,
    // and a count that moves with each decision. `outcome` is
    // BEEBUS_OUTCOME_NONE when none came back (the write failed, or no answer
    // within rotationsResult). False when no such write was made.
    bool topicResult(uint8_t nodeID, uint16_t topicID, uint8_t& outcome, BeeBusScalar& kept,
                     uint16_t& count);

    // A queued topic's write has been taken by the program (setTopicQueued):
    // a write that asked for a result is read back from here, after the next
    // loop() and `settleMs`; a `command` is answered at once, not read back.
    // Call it once per write taken.
    void topicApplied(uint16_t topicID, uint16_t settleMs = 0, bool command = false);

    // This node, as the owner, decided a write that asked: (writer, topicID,
    // outcome). For counting refusals.
    void onTopicDecided(BeeBusTopicDecidedHandler handler);

    // --- Bidirectional binding (layer 3) ---
    //
    // One owner per topic serves it; any node binds it. Change a bound
    // variable and the change goes to the owner, which decides; the owner's
    // value comes back into the variable whatever it decided, and onBindState
    // says how it went. Nothing to pace, retry or time out in the application.
    //
    //     owner.serve(10, &setpoint);                       // the PLC
    //     panel.bind(OWNER, 10, &shownSetpoint);            // a panel
    //     shownSetpoint = 50;   // -> Pending -> InSync (or Refused, kept value)
    //
    // serve: a writable topic. Without a validate the owner reads the variable
    // back after the next loop() and settleMs, answering ACCEPTED if it still
    // holds what was written or CHANGED with what it holds; with min < max a
    // value outside is REFUSED. A COMMAND serve answers once the write is
    // taken.
    template <typename T>
    bool serve(uint16_t topicID, T* variable, const BeeBusServe& terms = BeeBusServe()) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return serveRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, terms, nullptr, 0);
    }
    template <typename T>
    bool serve(const char* name, uint16_t topicID, T* variable,
               const BeeBusServe& terms = BeeBusServe()) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return serveRaw(topicID, BeeBusTypeOf<T>::value, (void*)variable, terms, name, 0);
    }
    bool serve(uint16_t topicID, char* text, uint16_t capacity,
               const BeeBusServe& terms = BeeBusServe());
    bool serveRaw(uint16_t topicID, uint8_t type, void* variable, const BeeBusServe& terms,
                  const char* name, uint16_t textCapacity);
    bool unserve(uint16_t topicID);

    // bind: LIVE by default, like every primitive; ask for RELIABLE in the
    // terms for a setpoint, a setting or configuration, which a lost frame
    // must never leave wrong. The variable holds the owner's value once it has
    // arrived (state InSync); until then it is left as it is.
    template <typename T>
    bool bind(uint8_t ownerID, uint16_t topicID, T* variable,
              const BeeBusBindTerms& terms = BeeBusBindTerms()) {
        static_assert(BeeBusTypeOf<T>::supported, BEEBUS_TOPIC_TYPE_MESSAGE);
        return bindRaw(ownerID, topicID, BeeBusTypeOf<T>::value, (void*)variable, terms,
                       nullptr, 0);
    }
    bool bind(uint8_t ownerID, uint16_t topicID, char* text, uint16_t capacity,
              const BeeBusBindTerms& terms = BeeBusBindTerms());
    bool bindRaw(uint8_t ownerID, uint16_t topicID, uint8_t type, void* variable,
                 const BeeBusBindTerms& terms, const char* name, uint16_t textCapacity);
    bool unbind(uint8_t ownerID, uint16_t topicID);

    // Where a bind stands, and the owner's last decision on a change of it
    // with the value it kept. False when nothing is bound there.
    BeeBusBindState bindState(uint8_t ownerID, uint16_t topicID);
    bool bindResult(uint8_t ownerID, uint16_t topicID, uint8_t& outcome, BeeBusScalar& kept);

    // Sends a change of the bound variable now, rather than when loop() next
    // notices it.
    void bindChanged(uint8_t ownerID, uint16_t topicID);
    void onBindState(BeeBusBindStateHandler handler);

    // This instance's session, which tells peers a restart apart from it.
    uint16_t session();

    // --- Utility and status functions ---

    // Operations status
    uint16_t getPendingOpCount();

    // Where a read or write to this peer and address has got to. Pending while
    // it is outstanding, then Done or Failed until the next request for the
    // same address replaces it. `result` says which failure.
    BeeBusRequestState requestState(uint8_t nodeID, uint16_t address, BeeBusResult& result);
    void clearPendingOps();

    // Link health, as counted by the framer. A rising crcErrors on an otherwise
    // quiet bus is the number to judge a link by; noiseBytesDiscarded growing
    // steadily means a floating bus or a baud rate mismatch rather than an
    // occasional collision.
    BeeBusIOStats getLinkStatistics();
    void resetLinkStatistics();

    // Peers this node currently counts as ring members, itself included, and
    // each of them by position so the ring can be walked. getMemberAt returns
    // 0 past the end, and the order is the order the token takes.
    uint8_t getMemberCount();
    uint8_t getMemberAt(uint8_t index);

    // Measured network timing, in milliseconds. Both return 0 until the node
    // has seen enough clean rotations to be sure of them.
    //
    // getRotationTime() is how long the token takes to come round, and the
    // honest answer to "how quickly can this network respond". getHopTime()
    // is how long one peer takes to accept the token, which is where a slow
    // link on an otherwise fast ring shows up.
    bb_time_t getRotationTime();
    bb_time_t getHopTime(uint8_t nodeID);

    // What the same hop is allowed before the token is presumed lost: the mean
    // plus four times the spread this node has measured on it. getHopTime says
    // how quick a peer is, this says how steady, and the two together are what
    // separates a link that is slow from one that is going bad.
    //
    // Every node times every hop it hears, so these agree across the ring
    // rather than describing one node's corner of it. Walking getMemberAt and
    // asking for each is a latency and health map of the whole network.
    bb_time_t getHopTimeout(uint8_t nodeID);

    // The response window this node would state on its next invitation, and the
    // longest it will hold the bus quiet for somebody else's.
    //
    // A window is silence the ring asks of itself, and the second figure is
    // what keeps that inside the ring's own patience with a peer it has not
    // heard from. Worth watching on a segment that is misbehaving: a window
    // climbing away from the settled figure is corruption being read as nodes
    // contending for a gap, which on RS485 is usually a pair with nothing
    // driving it rather than anybody waiting to join.
    bb_time_t getSolicitWindow();
    bb_time_t getSolicitWindowLimit();

    // Measured bus and host timing, both 0 until something has been observed.
    //
    // getByteTime() is what one byte costs on this bus in microseconds, taken
    // from arriving frames, and is what raises the interframe gap on a slow
    // link. getServiceInterval() is how long this node takes between calls to
    // loop(), in milliseconds; it floors every timeout the node derives, since
    // one shorter than that cannot be enforced whatever the network is doing.
    // A figure here far above the others is application code, not the network.
    uint16_t getByteTime();
    bb_time_t getServiceInterval();

    // Error handling
    void onError(void (*errorHandler)(BeeBusError error, const char* message));

    // Debug output. The debug instance is process-wide, so these handlers apply
    // to every node in the process; each message names the node it came from.
    void onDebug(void (*debugHandler)(const char* category, const char* message));

    // Every frame on the wire, decoded. `direction` is "TX" or "RX". Costs a
    // decode per frame, so register it to diagnose a segment, not in production.
    void onFrameDebug(void (*handler)(const char* direction, const uint8_t* frame,
                                      uint16_t length, const char* decoded));

    // Every Modbus PDU, decoded. `type` is "REQUEST" or "RESPONSE".
    void onModbusDebug(void (*handler)(const char* type, uint8_t nodeID,
                                       const char* decoded));

private:
    // Why a destination cannot be reached. A node stood down for a duplicate ID
    // has no peers at all, so without this every operation reports UnknownNode
    // and blames the destination for a fault that is entirely local.
    BeeBusResult unreachable() const;

    // An address on this node is served in place, so it never reaches the
    // operation queue and nothing there retires it. These owe the same one
    // report as any other accepted operation, delivered on the next loop().
    // The completions of reads and writes: the application's read / write
    // complete handler, as it is when the operation concludes. `ctx` is the node.
    static void readCompleted(void* ctx, uint8_t nodeID, uint16_t address, uint8_t function,
                              BeeBusResult result);
    static void writeCompleted(void* ctx, uint8_t nodeID, uint16_t address, uint8_t function,
                               BeeBusResult result);

    void reportLocalRead(uint8_t nodeID, uint16_t address, BeeBusResult result);
    void reportLocalWrite(uint8_t nodeID, uint16_t address, BeeBusResult result);

    // Direct protocol instance
    BeeBusProtocol* _protocol;

    // Set when begin() created the transport itself and must delete it.
    BeeBusTransport* _ownedTransport;
    const char* _startError = nullptr;

    // Implementation methods for templates
    BeeBusResult readHreg_impl(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numregs, uint8_t fc, uint16_t stride = 0);
    BeeBusResult readCoil_impl(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numcoils, uint8_t fc, uint16_t stride = 0);
    BeeBusResult readIreg_impl(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numiregs, uint8_t fc, uint16_t stride = 0);
    BeeBusResult readIsts_impl(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numists, uint8_t fc, uint16_t stride = 0);
    BeeBusResult writeHreg_impl(uint8_t nodeID, uint16_t offset, const int16_t* values,
                                uint16_t numregs, uint8_t fc, BeeBusDelivery delivery,
                                uint16_t stride = 0);
    BeeBusResult writeCoil_impl(uint8_t nodeID, uint16_t offset, const bool* values,
                                uint16_t numcoils, uint8_t fc, BeeBusDelivery delivery,
                                uint16_t stride = 0);

    // Queues a read to a peer. Shared by every read, whatever its buffer holds.
    BeeBusResult queueRead(uint8_t nodeID, uint16_t offset, uint16_t quantity,
                           uint8_t functionCode, void* result, BeeBusResultKind kind,
                           uint16_t resultCapacity = 0, uint16_t stride = 0);

    BeeBusResult readString_impl(uint8_t nodeID, uint16_t address, char* buffer,
                                 uint16_t size, uint8_t functionCode);

    BeeBusResult captureForBuffer(uint16_t index, uint16_t topicID,
                                  const BeeBusScalar& value);

    // Queues an already-encoded WRITE and wires the completion handler, shared
    // by the scalar and text forms.
    BeeBusResult queueTopicWrite(uint8_t nodeID, uint16_t topicID,
                                 const BeeBusBytes& pdu, BeeBusDelivery delivery,
                                 bool wantResult = false);
    static void resultWriteCompleted(void* ctx, uint8_t nodeID, uint16_t address,
                                     uint8_t function, BeeBusResult result);

    BeeBusResult writeTopicText(uint8_t nodeID, uint16_t topicID, const char* text,
                                size_t length, BeeBusDelivery delivery,
                                bool wantResult = false);

    std::function<void(uint8_t, uint16_t, BeeBusResult)> _writeCompleteHandler;
    std::function<void(uint8_t, uint16_t, BeeBusResult)> _readCompleteHandler;

#if defined(BEEBUS_PLATFORM_HOST)
    // Mutable so the const accessors can take it: reporting state is a read to
    // the caller and has to be serialised the same as everything else.
    mutable std::recursive_mutex _lock;
#endif
};
