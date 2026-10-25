#pragma once
#include "BeeBusGlobal.h"

// Forward declarations
class BeeBusProtocol;

// --- Statistics structure ---
struct BeeBusIOStats {
    uint32_t framesReceived = 0;
    uint32_t framesSent = 0;
    uint32_t crcErrors = 0;
    uint32_t framingErrors = 0;
    uint32_t bufferOverflows = 0;

    // Bytes dropped while hunting for a frame start. Sustained growth means
    // line noise, a floating bus, or a baud rate mismatch.
    uint32_t noiseBytesDiscarded = 0;

    // Times an oversized partial frame was abandoned to resynchronise.
    uint32_t resyncs = 0;
};

/**
 * BeeBusIO - moves frames between the protocol and the transport: assembles
 * and checks arriving frames, dispatches their sections, and builds and sends
 * outgoing ones.
 */
class BeeBusIO {
public:
    // --- Constructor and destructor ---
    BeeBusIO(BeeBusProtocol& protocol);
    ~BeeBusIO();

    // --- Initialization ---
    bool begin(BeeBusTransport* transport);

    // --- Main processing ---
    void processIncoming();

    // --- Frame transmission ---
    // Hands the token to successorID, carrying whatever else the header says.
    bool sendTokenFrame(const BeeBusHeader& header);

    // The same, with any queued operations and responses packed in behind it.
    // Falls back to a header-only frame when nothing fits.
    bool sendDataFrame(const BeeBusHeader& header);

    // Writes one section at pos: the PDU of `req` (an operation's request when
    // `op` is given, else a response), the acknowledgement ID when asked for,
    // and the backfilled length. False, with pos unchanged, when it does not
    // fit, which ends the frame.
    bool appendSection(uint8_t* buffer, uint16_t& pos, uint8_t pduType, uint8_t destNodeID,
                       const BeeBusModbusRequest& req, const BeeBusPendingModbusOp* op,
                       bool wantsAck, uint16_t opId);

    bool sendJoinRequestFrame(uint8_t srcNodeID);
    bool sendDepartureFrame(uint8_t srcNodeID);

    // Says this node is waiting, into the gap a frame has just left behind.
    // Refuses unless the first byte can still land inside that gap, since a
    // gap is only mutual exclusion for as long as everybody is still in it.
    bool announcePresence();

    // --- Measured bus timing ---

    // What one byte costs on this bus, in microseconds, or zero before that is
    // known from either source. The stated line rate where the transport knows
    // it, and the measurement otherwise, which is the same order lineByteTimeUs
    // applies: two accessors answering the same question differently is how a
    // joining node ends up sizing its answer slot against one figure while the
    // ring sizes the window it answers in against another.
    uint16_t byteTimeUs() const {
        const bb_time_t line = lineByteTimeUs();
        return line > BEEBUS_BYTE_TIME_MAX_US ? (uint16_t)BEEBUS_BYTE_TIME_MAX_US
                                              : (uint16_t)line;
    }

    // The measurement alone, ignoring anything the transport states. For
    // reporting what the bus was observed to do, not for deriving timing from.
    uint16_t measuredByteTimeUs() const { return _byteTimeUs; }

    // Silence this node leaves before transmitting. The configured gap is a
    // minimum the bus may raise, never lower: one byte must fit inside it, or
    // a node transmits over a frame whose first byte has not arrived yet.
    bb_time_t effectiveGapUs() const;
    bb_time_t lineByteTimeUs() const;

    // Microseconds since the last byte arrived.
    bb_time_t silenceUs() const;

    // --- Statistics ---
    void resetStatistics();
    BeeBusIOStats getStatistics();
    void incrementFrameReceived();
    void incrementFrameSent();
    void incrementCrcError();
    void incrementFramingError();
    void incrementBufferOverflow();

    // Whether anything arrived during the last processIncoming(), which is how
    // a node about to start a ring of its own tells that somebody else is
    // already talking.
    bool _rxAvailable;

private:
    BeeBusProtocol& _protocol;
    BeeBusTransport* _transport;

    // Last bus activity in microseconds. Scaling the
    // millisecond stamp instead would overflow 32 bits after ~72 minutes.
    bb_time_t _lastBusActivityUs;

    // CRC and length of the last frame this node transmitted, so a transport
    // that loops its own output back is not mistaken for another node using
    // our address. Half-duplex RS485 usually does not echo; a socket, a
    // hardware loopback or a repeater does.
    uint16_t _lastTxCrc;
    uint16_t _lastTxLen;
    bb_time_t _lastTxAt;

    // Set when a frame has been received and acted on, cleared by whoever acts
    // on the gap it left behind.
    bool _frameCompleted;

    // Byte timing, measured from the spacing of arrivals rather than from any
    // frame. A line clocks its bytes out evenly whatever they turn out to
    // mean, so a run of even arrivals is the rate however it is framed: a
    // collision wreck is timed as readily as a good frame, which is the only
    // evidence available to a node hearing nothing but wrecks.
    //
    // Frame boundaries are deliberately not used. Bytes are read a bounded
    // number at a time, so a frame drained out of a backlog spans this node's
    // own read pacing and not the bus at all.
    bb_time_t _runLastUs;
    bb_time_t _runSpanUs;
    bb_time_t _runMinDeltaUs;
    bb_time_t _runMaxDeltaUs;
    uint16_t _runBytes;

    // The lowest mean across the last BEEBUS_BYTE_TIME_WINDOW closed runs, and
    // the runs it is taken from. Recomputed when a run closes, so a line that
    // slows down carries the figure up with it once the fast runs have aged
    // out of the window.
    uint16_t _byteTimeUs;
    uint16_t _byteTimeRuns[BEEBUS_BYTE_TIME_WINDOW];
    uint8_t _byteTimeNext;
    uint8_t _byteTimeCount;

    // Folds one closed run's mean in and takes the window's minimum.
    void noteByteTime(uint16_t meanUs);

    // Adds one arrival to the run in progress, and closes a run off when the
    // bus falls silent or the run stops looking like one transmission.
    void noteArrival(bb_time_t arrivedUs);
    void closeRun();

    // Whether an incoming frame is our own transmission coming back.
    bool isOwnEcho(const uint8_t* buffer, uint16_t length) const;

    // --- Receive buffers ---

    // Primary receive buffer (continuously filled)
    uint8_t _primaryRxBuffer[BEEBUS_MAX_RX_BUFFER];
    uint16_t _primaryRxPos;

    // Frame assembly state. Bytes only accumulate between two FLAGs, and an
    // ESC applies to the byte that follows it.
    bool _inFrame;
    bool _escapePending;

    // Escaped frame staged for transmission. Worst case every body byte needs
    // escaping, plus the two flags.
    uint8_t _txWireBuffer[BEEBUS_MAX_WIRE_FRAME];

    // The frame being assembled, before escaping. A member rather than a heap
    // allocation per frame: a node passes the token thousands of times an hour
    // for years, and an allocate/free of half a kilobyte on that path is how a
    // long-running embedded node fragments its heap. Same bytes either way.
    uint8_t _txFrameBuffer[BEEBUS_MAX_TX_BUFFER];

    // Processing buffer (contains complete frames for processing)
    uint8_t _processingBuffer[BEEBUS_MAX_RX_BUFFER];
    uint16_t _processingBufferLen;

    // Frame extraction queue
    struct CompleteFrame {
        uint8_t data[BEEBUS_MAX_RX_BUFFER];
        uint16_t length;
    };

    // The topic PDU being handled, decoded here rather than on the stack.
    BeeBusTopicPdu _topicPdu;

    // A ring of frames waiting to be processed, oldest at _frameHead.
    static constexpr uint8_t MAX_FRAME_QUEUE = 5;
    CompleteFrame _frames[MAX_FRAME_QUEUE];
    uint8_t _frameHead = 0;
    uint8_t _frameCount = 0;

    // --- Statistics ---
    BeeBusIOStats _stats;

    // --- Double buffer methods ---
    void queueAssembledFrame();
    void processQueuedFrames();

    // --- Frame processing ---
    void processCompleteFrame();
    void handleControlFrame(const BeeBusHeader& header);
    void processModbusData(const BeeBusHeader& header);
    void processModbusSection(const uint8_t* buffer, const BeeBusSection& section, uint8_t srcNodeID);
    void processTopicSection(const uint8_t* buffer, const BeeBusSection& section, uint8_t srcNodeID);
    void handleModbusRequest(const BeeBusModbusRequest& request, uint8_t srcNodeID,
                             bool ackRequested, uint16_t ackOpId);
    void queueAcknowledgement(uint8_t destNodeID, uint16_t opId, uint8_t status);
    void handleAcknowledgement(const uint8_t* pdu, uint16_t length, uint8_t srcNodeID);
    void handleModbusResponse(const BeeBusModbusRequest& response, uint8_t srcNodeID);

    // --- Transmission utilities ---
    bool sendFrame(const uint8_t* buffer, uint16_t length);

public:
    // Whether the bus has been quiet long enough to transmit into. Public so a
    // caller can ask before it commits state to a frame that may not go out.
    bool isTransmissionReady();

    // Whether a frame completed during the last processIncoming(), clearing
    // the flag. Transmitting is not done from inside the receive path: on a
    // blocking transport a write holds the caller for the whole airtime, and
    // anything that had already read the clock would be working from a reading
    // the write itself invalidated.
    bool takeFrameCompleted() {
        const bool completed = _frameCompleted;
        _frameCompleted = false;
        return completed;
    }
};
