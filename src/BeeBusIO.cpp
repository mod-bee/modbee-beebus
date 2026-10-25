#include "BeeBusGlobal.h"
#include "BeeBusTopicHandler.h"
#include "BeeBusNoExcept.h"

// --- Construction ---
BeeBusIO::BeeBusIO(BeeBusProtocol& protocol)
    // Ordered to match declaration order in the header.
    : _rxAvailable(false),
      _protocol(protocol),
      _transport(nullptr),
      _lastBusActivityUs(0),
      _lastTxCrc(0),
      _lastTxLen(0),
      _lastTxAt(0),
      _frameCompleted(false),
      _runLastUs(0),
      _runSpanUs(0),
      _runMinDeltaUs(0),
      _runMaxDeltaUs(0),
      _runBytes(0),
      _byteTimeUs(0),
      _byteTimeRuns{},
      _byteTimeNext(0),
      _byteTimeCount(0),
      _primaryRxPos(0),
      _inFrame(false),
      _escapePending(false),
      _processingBufferLen(0) {

    // Initialize frame queue
    _frameHead = 0;
    _frameCount = 0;

    // Initialize statistics
    resetStatistics();
}

BeeBusIO::~BeeBusIO() = default;

// --- Initialisation ---
bool BeeBusIO::begin(BeeBusTransport* transport) {
    if (!transport) {
        return false;
    }

    _transport = transport;
    _primaryRxPos = 0;
    _inFrame = false;
    _escapePending = false;
    _processingBufferLen = 0;
    _lastBusActivityUs = 0;
    _rxAvailable = false;
    _frameCompleted = false;
    _runLastUs = 0;
    _runSpanUs = 0;
    _runMinDeltaUs = 0;
    _runMaxDeltaUs = 0;
    _runBytes = 0;
    _byteTimeUs = 0;
    _byteTimeNext = 0;
    _byteTimeCount = 0;
    for (uint8_t i = 0; i < BEEBUS_BYTE_TIME_WINDOW; i++) _byteTimeRuns[i] = 0;

    // Clear frame queue
    _frameHead = 0;
    _frameCount = 0;

    // Reset statistics
    resetStatistics();

    return true;
}

// --- Receive path ---
void BeeBusIO::processIncoming() {
    if (!_transport) {
        return;
    }

    // Step 1: Assemble frames between FLAGs, un-escaping as we go.
    //
    // A FLAG can never appear inside a frame body, so it unambiguously ends the
    // frame in progress and starts the next. Anything arriving outside a frame
    // is line noise and is discarded immediately, which is what stops a
    // floating bus, reading as sustained 0xFF or 0x00, from filling the buffer.
    bool dataReceived = false;
    uint16_t bytesThisCall = 0;
    uint16_t noiseThisCall = 0;

    // Two bounds, not one. The byte cap bounds the time this call may take. The
    // queue check bounds it by frames, and is what stops a backlog being read
    // off the transport faster than it can be processed and thrown away for
    // want of a slot: the byte cap admits far more control frames than the
    // queue holds, so on its own it would let a stalled application lose
    // frames it had already taken off the wire. Stopping instead leaves them
    // in the transport, where the next call takes them.
    while (_transport->available() && bytesThisCall < BEEBUS_MAX_RX_BYTES_PER_LOOP &&
           _frameCount < MAX_FRAME_QUEUE) {
        int incomingByte = _transport->read();
        if (incomingByte == -1) break;

        const uint8_t byte = (uint8_t)incomingByte;
        bytesThisCall++;
        const bb_time_t arrivedUs = bb_micros();
        _lastBusActivityUs = arrivedUs;
        dataReceived = true;

        noteArrival(arrivedUs);

        if (byte == BEEBUS_FLAG) {
            if (_inFrame && _primaryRxPos >= BEEBUS_MIN_FRAME_LEN) {
                queueAssembledFrame();
            } else if (_inFrame && _primaryRxPos == 2 &&
                       _primaryRxBuffer[1] == BEEBUS_ANNOUNCE_BYTE) {
                // Somebody spoke in the gap. One byte is short of any frame, so
                // the two flags around it belong to the frames either side and
                // the announcement costs nothing but itself.
                _protocol.handleJoinAnnounce();
            } else if (_inFrame && _primaryRxPos > 1) {
                incrementFramingError();   // Runt between two flags
            }
            // Restart assembly. Index 0 carries the frame marker, which is the
            // layout the parsing layer expects.
            _primaryRxBuffer[0] = BEEBUS_FLAG;
            _primaryRxPos = 1;
            _escapePending = false;
            _inFrame = true;
            continue;
        }

        if (!_inFrame) {
            _stats.noiseBytesDiscarded++;   // Hunting for a frame start
            noiseThisCall++;

            continue;
        }

        uint8_t value = byte;
        if (_escapePending) {
            value = byte ^ BEEBUS_ESC_XOR;
            _escapePending = false;
        } else if (byte == BEEBUS_ESC) {
            _escapePending = true;
            continue;
        }

        if (_primaryRxPos >= BEEBUS_MAX_RX_BUFFER) {
            // Longer than any valid frame, so abandon it and hunt again.
            _stats.resyncs++;
            _primaryRxPos = 0;
            _escapePending = false;
            _inFrame = false;
            continue;
        }

        _primaryRxBuffer[_primaryRxPos++] = value;
    }

    // Update availability flag
    _rxAvailable = dataReceived;

    // Reported once for the call rather than per byte: a collision that leaves
    // no delimiter behind is heard as a run of noise, and that is still
    // somebody having answered. How much of it there was goes with it, since
    // an unbiased pair produces the same bytes without ever stopping and the
    // length is what separates the two.
    if (noiseThisCall != 0) {
        _protocol.noteCorruptFrame(BeeBusProtocol::BeeBusCorruption::Noise,
                                   noiseThisCall);
    }

    // Step 2: Process any queued complete frames
    processQueuedFrames();
}

// --- Frame assembly ---
void BeeBusIO::queueAssembledFrame() {
    // Full, so something has to go. It is the oldest, not the newest: the queue
    // only fills when a backlog delivered more frames in one drain than the
    // queue holds, and in that backlog the newest frame is the one carrying the
    // live token. Dropping it instead would have the node act on stale frames,
    // miss its turn, and stall the ring in exactly the conditions that produced
    // the backlog.
    if (_frameCount >= MAX_FRAME_QUEUE) {
        incrementBufferOverflow();
        _frameHead = (uint8_t)((_frameHead + 1) % MAX_FRAME_QUEUE);
        _frameCount--;
    }

    CompleteFrame& frame = _frames[(_frameHead + _frameCount) % MAX_FRAME_QUEUE];
    frame.length = _primaryRxPos;
    memcpy(frame.data, _primaryRxBuffer, frame.length);
    _frameCount++;
}

// --- Queued frames ---
void BeeBusIO::processQueuedFrames() {
    while (_frameCount > 0) {
        const CompleteFrame& frame = _frames[_frameHead];
        memcpy(_processingBuffer, frame.data, frame.length);
        _processingBufferLen = frame.length;
        _frameHead = (uint8_t)((_frameHead + 1) % MAX_FRAME_QUEUE);
        _frameCount--;

        processCompleteFrame();
    }
}

// --- Frame processing ---
void BeeBusIO::processCompleteFrame() {
    if (_processingBufferLen < BEEBUS_MIN_FRAME_LEN) {
        return;
    }

    if (!BeeBusFrame::verifyCRC(_processingBuffer, _processingBufferLen)) {
        incrementCrcError();
        _protocol.reportError(BeeBusError::CrcError, "CRC verification failed");
        return;
    }

    incrementFrameReceived();

    BeeBusHeader header;
    if (!BeeBusFrame::parseHeader(_processingBuffer, _processingBufferLen, header)) {
        incrementFramingError();
        _protocol.reportError(BeeBusError::FrameError, "Header parsing failed");
        return;
    }

    // Every node transmits under an address in the usable range, so a sender
    // outside it is a malformed header however good the CRC: 0 means "none"
    // and 255 is the broadcast destination. Nothing from such a frame can be
    // answered, timed or admitted, so none of it is read. Only the sender is
    // checked here; a section addressed to 255 is a broadcast.
    if (header.srcNodeID < BEEBUS_NODE_ID_MIN || header.srcNodeID > BEEBUS_NODE_ID_MAX) {
        incrementFramingError();
        _protocol.reportError(BeeBusError::FrameError, "Sender outside the node ID range");
        return;
    }

    // A frame carrying our own address that we did not send means another node
    // is using it. Nothing downstream can make sense of the bus until that is
    // resolved, so it is checked before anything is learnt from the frame.
    if (header.srcNodeID == _protocol.getNodeID()) {
        if (!isOwnEcho(_processingBuffer, _processingBufferLen)) {
            _protocol.handleDuplicateNodeID(header.srcNodeID);
        }
        return;
    }

    // Everything the frame says about who is taking part in which ring.
    _protocol.observeFrame(header);

    // Sections first: they are the time critical part, because several nodes
    // acting on one frame is what keeps their outputs in step.
    if (header.has(BEEBUS_HDR_HAS_SECTIONS)) {
        processModbusData(header);
    }

    handleControlFrame(header);

    // A frame has ended, so the bus is in the gap that follows it. Recorded
    // rather than acted on: whoever wants that gap is told once this call has
    // returned and the receive path is off the stack.
    _frameCompleted = true;
}

// --- Control frames ---
void BeeBusIO::handleControlFrame(const BeeBusHeader& header) {
    // An exclusive decode: every bit means one thing, so no two handlers fire
    // for the same bytes.
    //
    // Almost everything here is conditional on the sender taking part in our
    // ring. A node with a failed receiver transmits for ever as its own ring of
    // one, and unguarded those frames confirm token passes that never happened,
    // evict members, and stall the ring on every solicitation.
    const bool fromOurRing = _protocol.isMember(header.srcNodeID);

    // The token first: it is what decides whether this node may transmit.
    if (header.successorID == _protocol.getNodeID() && fromOurRing) {
        // A second copy of a turn already taken says only that the sender did
        // not hear us take it. Acting on it would hand out a token already
        // elsewhere in the ring and time a rotation as the retry interval.
        const bool repeated = _protocol.isRepeatedTurn(header);
        if (!repeated || _protocol.noteRepeatedTurn(header.srcNodeID)) {
            _protocol.handleTokenReceived(header.srcNodeID, true);
            _protocol.setTokenReceivedForUs();
        }
    } else if (header.successorID != 0 && header.successorID != header.srcNodeID &&
               fromOurRing) {
        // A node handing the token to itself is running a ring of one, whatever
        // our membership still says. It confirms nothing, and counting it as
        // the token going round would have a node that dropped out keep every
        // survivor believing a stopped ring is running.
        _protocol.handleTokenReceived(header.srcNodeID, false);
        _protocol.notePassNotForUs();
    }

    // Departures before arrivals, so a frame that removes one node and admits
    // another leaves membership in the state the sender described.
    if (header.has(BEEBUS_HDR_MEMBER_LEFT) && fromOurRing) {
        _protocol.handleMemberLeft(header.leftNodeID, header.srcNodeID);
    }

    if (header.has(BEEBUS_HDR_MEMBER_JOINED)) {
        if (header.joinedNodeID == header.srcNodeID) {
            // First person: the node is asking to join, having been solicited.
            // This is the one case that cannot require ring membership, since
            // it is what confers it.
            _protocol.handleJoinRequest(header.joinedNodeID, header.srcNodeID);
        } else if (fromOurRing) {
            // Third person: a member announcing somebody else's arrival.
            _protocol.handleMemberJoined(header.joinedNodeID, header.srcNodeID);
        }
    }

    if (header.has(BEEBUS_HDR_SOLICIT)) {
        _protocol.handleSolicitation(header, fromOurRing);
    }
}

// --- Modbus sections ---
void BeeBusIO::processModbusData(const BeeBusHeader& header) {
    // Taken one at a time, straight from the frame: each section is read once,
    // in order, so nothing needs holding.
    uint16_t pos = 0;
    BeeBusSection section;
    while (BeeBusFrame::nextSection(_processingBuffer, _processingBufferLen, header,
                                    pos, section)) {
        switch (section.pduType) {
            case BEEBUS_PDU_MODBUS:
                processModbusSection(_processingBuffer, section, header.srcNodeID);
                break;

            case BEEBUS_PDU_TOPIC:
                processTopicSection(_processingBuffer, section, header.srcNodeID);
                break;

            default:
                // A PDU type this build does not carry. The section is
                // length-prefixed, so skipping it costs nothing and the rest of
                // the frame is still read: a node meeting a newer peer loses
                // only what it could not have understood anyway.
                BEEBUS_DEBUG_IO("SECTION: Unknown PDU type %d from Node:%d, skipped",
                    section.pduType, header.srcNodeID);
                break;
        }
    }
}

void BeeBusIO::processModbusSection(const uint8_t* buffer, const BeeBusSection& section, uint8_t srcNodeID) {
    const uint16_t start = section.start;
    const uint16_t end = section.end;

    if (start >= end || end > _processingBufferLen) {
        return;
    }

    uint16_t pos = start;

    // The destination node ID prefixes the section and is not part of the PDU.
    const uint8_t destNodeID = buffer[pos++];

    if (destNodeID != _protocol.getNodeID()) {
        return;   // Addressed to somebody else on this bus
    }

    uint16_t modbusLen = end - pos;

    // A flagged section carries the sender's operation ID in its last two
    // bytes. They are not PDU content and must come off before parsing.
    uint16_t ackOpId = 0;
    if (section.ackRequested) {
        if (modbusLen < BEEBUS_ACK_ID_BYTES + 2) {
            BEEBUS_DEBUG_IO("MODBUS: Section flagged for ack but too short (%d bytes)", modbusLen);
            return;
        }
        modbusLen = (uint16_t)(modbusLen - BEEBUS_ACK_ID_BYTES);
        ackOpId = bb_unpackBE16(&buffer[pos + modbusLen]);
    }

    if (modbusLen < 2) {
        BEEBUS_DEBUG_IO("MODBUS: Frame too short (%d bytes)", modbusLen);
        return;
    }

    // Which direction this section runs in.
    bool isResponse = false;
    uint8_t functionCode = buffer[pos];

    // A batch of acknowledgements for confirmed writes.
    if (functionCode == MB_FC_ACK) {
        handleAcknowledgement(&buffer[pos], modbusLen, srcNodeID);
        return;
    }

    // An exception response: the function code with 0x80 set.
    if (functionCode & 0x80) {
        isResponse = true;
        BEEBUS_DEBUG_IO("MODBUS: Detected ERROR RESPONSE FC:%02X", functionCode);
    }
    // Said on the wire by any peer at this version. Nothing to work out.
    else if (section.declaredResponse) {
        isResponse = true;
    }
    // A peer that leaves the response bit clear. Fall back on the shape of the
    // PDU, which is enough except for one case.
    else if (functionCode >= 0x01 && functionCode <= 0x04 && modbusLen >= 5) {
        const uint8_t byteCount = buffer[pos + 3];   // After FC + Addr(2)

        if (modbusLen == (uint16_t)(4 + byteCount)) {
            // Structurally a response: FC + Addr(2) + ByteCount + Data. But a
            // read request is FC + Addr(2) + Quantity(2), always five bytes,
            // and its quantity's high byte sits where the byte count does. So
            // the two shapes coincide at exactly five bytes with a byte count
            // of one, which is a quantity of 256..511.
            const bool ambiguous = (modbusLen == 5 && byteCount == 1);

            if (!ambiguous) {
                isResponse = true;
            } else if (functionCode == MB_FC_READ_COILS ||
                       functionCode == MB_FC_READ_DISCRETE_INPUTS) {
                // A single data byte is one to eight bits, so only a bit read
                // can produce it. Settle it by asking whether this node has
                // such a read outstanding to that peer: only a node that asked
                // can be answered.
                isResponse = _protocol.getOperations().hasOutstandingRead(
                    srcNodeID, functionCode, bb_unpackBE16(&buffer[pos + 1]));
            }
            // A register read answers two bytes per register, so a byte count
            // of one cannot be a response to 0x03 or 0x04 at all: it is a
            // request for 256..511 registers, which the handler will refuse
            // as an illegal quantity rather than silently ignore.
        }
    }

    BeeBusModbusRequest modbusFrame;
    bool parseSuccess = false;

    if (isResponse) {
        parseSuccess = BeeBusModbusFrame::parseModbusResponse(&buffer[pos], modbusLen, modbusFrame);
        if (parseSuccess) {
            handleModbusResponse(modbusFrame, srcNodeID);
        } else {
            BEEBUS_DEBUG_IO("MODBUS: Failed to parse response");
        }
    } else {
        parseSuccess = BeeBusModbusFrame::parseModbusRequest(&buffer[pos], modbusLen, modbusFrame);
        if (parseSuccess) {
            handleModbusRequest(modbusFrame, srcNodeID, section.ackRequested, ackOpId);
        } else {
            BEEBUS_DEBUG_IO("MODBUS: Failed to parse request");
        }
    }

#ifndef DISABLE_BEEBUS_DEBUG
    // The first 32 bytes of what would not parse, for the debug log only.
    if (!parseSuccess) {
        char hexDump[32 * 3 + 1] = "";
        char* p = hexDump;
        for (uint16_t i = pos; i < end && i < pos + 32; i++) {
            snprintf(p, 4, "%02X ", buffer[i]);
            p += 3;
        }
        BEEBUS_DEBUG_IO("MODBUS: Failed to parse section, raw data: %s", hexDump);
    }
#endif
}

void BeeBusIO::processTopicSection(const uint8_t* buffer, const BeeBusSection& section,
                                   uint8_t srcNodeID) {
    const uint16_t start = section.start;
    const uint16_t end = section.end;

    if (start >= end || end > _processingBufferLen) {
        return;
    }

    uint16_t pos = start;

    // The destination node ID prefixes the section as it does a Modbus one.
    // A broadcast is addressed to everybody and read by whoever has the topic
    // bound; the rest drop it in handleValue as they would any topic they do
    // not hold.
    const uint8_t destNodeID = buffer[pos++];
    if (destNodeID != _protocol.getNodeID() && destNodeID != BEEBUS_NODE_BROADCAST) {
        return;
    }

    uint16_t pduLen = (uint16_t)(end - pos);

    uint16_t ackOpId = 0;
    if (section.ackRequested) {
        if (pduLen < BEEBUS_ACK_ID_BYTES + 1) {
            BEEBUS_DEBUG_IO("TOPIC: Section flagged for ack but too short (%d bytes)", pduLen);
            return;
        }
        pduLen = (uint16_t)(pduLen - BEEBUS_ACK_ID_BYTES);
        ackOpId = bb_unpackBE16(&buffer[pos + pduLen]);
    }

    BeeBusTopicHandler handler(_protocol);
    const uint8_t reason = handler.process(&buffer[pos], pduLen, srcNodeID,
                                           destNodeID == BEEBUS_NODE_BROADCAST, _topicPdu);

    // The acknowledgement only confirms. A refusal has already gone back as an
    // ERROR or a SUBACK, carrying a reason an exception code cannot express.
    if (section.ackRequested && reason == BEEBUS_TOP_OK) {
        queueAcknowledgement(srcNodeID, ackOpId, MB_ACK_OK);
    }
}

// --- Modbus requests and responses ---
void BeeBusIO::handleModbusRequest(const BeeBusModbusRequest& request, uint8_t srcNodeID,
                                   bool ackRequested, uint16_t ackOpId) {
    BEEBUS_DEBUG_MODBUS(BEEBUS_MODBUS_REQUEST, srcNodeID, request);

    // Process the request using BeeBusModbusHandler
    BeeBusModbusHandler handler(_protocol.getDataMap());
    BeeBusModbusRequest response;

    // Pass the sourceNodeID to the handler so it can be recorded for failsafe purposes
    if (handler.processRequest(request, response, srcNodeID)) {
        // Acknowledge if asked to, whatever the outcome. The sender is waiting
        // on this ID and will keep retrying until it hears something, so a
        // refusal has to travel back just as a success does.
        if (ackRequested) {
            queueAcknowledgement(srcNodeID, ackOpId,
                                 BeeBusModbusFrame::isErrorResponse(response.function)
                                     ? (response.data.empty() ? MB_EX_SLAVE_DEVICE_FAILURE
                                                              : response.data[0])
                                     : MB_ACK_OK);
        }
        // Only queue response if it's a read operation
        if (BeeBusModbusFrame::isReadFunction(request.function)) {
            BeeBusPendingResponse pendingResponse;
            pendingResponse.response = response;
            pendingResponse.destNodeID = srcNodeID;
            pendingResponse.sourceNodeID = _protocol.getNodeID();
            pendingResponse.timestamp = bb_millis();

            _protocol.getOperations().addPendingResponse(pendingResponse);
        }
    } else {
        // Error occurred - only send error response for read operations
        if (BeeBusModbusFrame::isReadFunction(request.function)) {
            BeeBusPendingResponse errorResponse;
            errorResponse.response = response; // Contains error
            errorResponse.destNodeID = srcNodeID;
            errorResponse.sourceNodeID = _protocol.getNodeID();
            errorResponse.timestamp = bb_millis();

            _protocol.getOperations().addPendingResponse(errorResponse);
            BEEBUS_DEBUG_IO("REQUEST: Queued error response to Node:%d", srcNodeID);
        } else {
            BEEBUS_DEBUG_IO("REQUEST: Write operation failed, no error response");
        }
    }
}

namespace {

// One acknowledgement: the operation ID being acknowledged, then its status.
void appendAckEntry(BeeBusBytes& data, uint16_t opId, uint8_t status) {
    uint8_t entry[BEEBUS_ACK_ENTRY_BYTES];
    bb_packBE16(entry, opId);
    entry[2] = status;
    data.insert(data.end(), entry, entry + sizeof(entry));
}

} // namespace

void BeeBusIO::queueAcknowledgement(uint8_t destNodeID, uint16_t opId, uint8_t status) {
    // One token can carry several confirmed writes from the same node.
    // Appending to the section already queued for it costs three bytes rather
    // than a section of its own.
    for (auto& queued : _protocol.getOperations().getPendingResponses()) {
        if (queued.destNodeID != destNodeID) continue;
        if (queued.response.function != MB_FC_ACK) continue;
        if (queued.response.data.size() >=
            (size_t)(BEEBUS_MAX_ACKS_PER_SECTION * BEEBUS_ACK_ENTRY_BYTES)) continue;

        appendAckEntry(queued.response.data, opId, status);
        return;
    }

    BeeBusPendingResponse ack;
    ack.response.function = MB_FC_ACK;
    ack.response.startAddr = opId;      // First ID in the batch, for matching
    ack.response.quantity = 0;
    ack.response.isResponse = true;
    appendAckEntry(ack.response.data, opId, status);
    ack.destNodeID = destNodeID;
    ack.sourceNodeID = _protocol.getNodeID();
    ack.timestamp = bb_millis();

    _protocol.getOperations().addPendingResponse(ack);
}

void BeeBusIO::handleAcknowledgement(const uint8_t* pdu, uint16_t length, uint8_t srcNodeID) {
    // [FC]([opIdHi][opIdLo][status])... The count is the section length rather
    // than a byte on the wire, so a malformed length has to be rejected here.
    const uint16_t entryBytes = (uint16_t)(length - 1);
    if (length < 1 + BEEBUS_ACK_ENTRY_BYTES ||
        entryBytes % BEEBUS_ACK_ENTRY_BYTES != 0) {
        BEEBUS_DEBUG_IO("ACK: Malformed acknowledgement from Node:%d (%d bytes)",
                      srcNodeID, length);
        return;
    }

    for (uint16_t pos = 1; pos < length; pos += BEEBUS_ACK_ENTRY_BYTES) {
        const uint16_t opId = bb_unpackBE16(&pdu[pos]);
        const uint8_t status = pdu[pos + 2];
        _protocol.getOperations().completeAcknowledgedWrite(srcNodeID, opId, status, _protocol);
    }
}

void BeeBusIO::handleModbusResponse(const BeeBusModbusRequest& response, uint8_t srcNodeID) {
    // An exception carries no data. Fulfilling it would retire the request as
    // though it had succeeded and leave the caller reading a stale value with
    // no way to tell.
    if (BeeBusModbusFrame::isErrorResponse(response.function)) {
        const uint8_t code = response.data.empty() ? MB_EX_SLAVE_DEVICE_FAILURE
                                                   : response.data[0];
        if (!_protocol.getOperations().failPendingRequest(response, srcNodeID)) {
            BEEBUS_DEBUG_IO("EXCEPTION: No matching request for Node:%d FC:%02X Addr:%d",
                srcNodeID, response.function, response.startAddr);
        }
        _protocol.reportError(bb_exceptionToError(code), bb_exceptionName(code));
        return;
    }

    // Match it to the pending request and fulfil that.
    if (!_protocol.getOperations().matchAndFulfillResponse(response, srcNodeID)) {
        BEEBUS_DEBUG_IO("RESPONSE: No matching request found for Node:%d FC:%02X Addr:%d",
            srcNodeID, response.function, response.startAddr);
    }
}

// --- Frame transmission ---
bool BeeBusIO::sendTokenFrame(const BeeBusHeader& header) {
    if (!isTransmissionReady()) {
        return false;
    }

    uint8_t buffer[BEEBUS_MAX_TX_BUFFER];
    const uint16_t frameLen = BeeBusFrame::buildControlFrame(buffer, sizeof(buffer), header);
    if (frameLen == 0) {
        BEEBUS_DEBUG_IO("TOKEN: Failed to build frame");
        return false;
    }

    return sendFrame(buffer, frameLen);
}

bool BeeBusIO::sendJoinRequestFrame(uint8_t srcNodeID) {
    // No gap check, and deliberately. This is sent in a slot of a contention
    // window, and the slot is the arbitration: a node that waits for quiet
    // instead misses the slot it was given and answers in somebody else's,
    // which is the collision the slotting exists to prevent. Carrier sense on
    // top of a schedule fights the schedule.
    BeeBusHeader header;
    header.srcNodeID = srcNodeID;
    header.joined(srcNodeID);        // First person: we are the one joining

    uint8_t buffer[BEEBUS_MAX_TX_BUFFER];
    const uint16_t frameLen = BeeBusFrame::buildControlFrame(buffer, sizeof(buffer), header);
    if (frameLen == 0) {
        return false;
    }

    return sendFrame(buffer, frameLen);
}

bool BeeBusIO::announcePresence() {
    if (_transport == nullptr) {
        return false;
    }

    // Nothing is known about the line yet, so there is no telling whether the
    // byte would land in time. Wait to be invited instead.
    const bb_time_t byteUs = lineByteTimeUs();
    if (byteUs == 0) {
        return false;
    }

    // It has to be over before the gap is, or the ring transmits across it.
    // The gap is at least two byte times wide, so one byte started now always
    // fits; started late it does not.
    if (silenceUs() + byteUs >= effectiveGapUs()) {
        return false;
    }

    const uint8_t byte = BEEBUS_ANNOUNCE_BYTE;
    return _transport->write(&byte, 1) == 1;
}

bool BeeBusIO::sendDepartureFrame(uint8_t srcNodeID) {
    if (!isTransmissionReady()) {
        return false;
    }

    BeeBusHeader header;
    header.srcNodeID = srcNodeID;
    header.left(srcNodeID);          // First person: we are the one leaving

    uint8_t buffer[BEEBUS_MAX_TX_BUFFER];
    const uint16_t frameLen = BeeBusFrame::buildControlFrame(buffer, sizeof(buffer), header);
    if (frameLen == 0) {
        return false;
    }

    return sendFrame(buffer, frameLen);
}

bool BeeBusIO::sendDataFrame(const BeeBusHeader& tokenHeader) {
    if (!isTransmissionReady()) {
        return false;
    }

    uint8_t* buffer = _txFrameBuffer;
    BeeBusOperations& operations = _protocol.getOperations();

    // What this frame may carry: the operations ready now, in queue order, and
    // the responses queued now. Kept as keys rather than copies; nothing in the
    // queue is copied or allocated to send it.
    typedef BeeBusOperations::FrameOp OpKey;
    BeeBusList<OpKey>& ready = operations.frameReady;
    BeeBusList<OpKey>& unbuildable = operations.frameUnbuildable;
    ready.clear();
    unbuildable.clear();
    {
        const BeeBusList<BeeBusPendingModbusOp>& pending = operations.getPendingOps();
        for (size_t i = 0; i < pending.size(); i++) {
            const BeeBusPendingModbusOp& op = pending[i];
            // A read, or a write awaiting acknowledgement, already went out. It
            // stays queued until its answer or its timeout, and is not sent twice.
            if (op.awaitingResponse) {
                continue;
            }
            // A function code the frame builder cannot encode would be emitted
            // as a section the receiver discards, and the caller would never
            // hear. It is refused here, once, with the reason.
            if (op.pduType == BEEBUS_PDU_MODBUS && !op.req.isResponse &&
                !BeeBusModbusFrame::isValidFunctionCode(op.req.function)) {
                unbuildable.push_back(OpKey{op.destNodeID, op.opId, false});
                continue;
            }
            // A retry waits out its backoff before it is sent again.
            if (!operations.isOperationReady(op, _protocol)) {
                continue;
            }
            // A confirmed topic write waits behind any earlier one to the same
            // topic still queued - awaiting its acknowledgement, or waiting out
            // a retry's backoff - so the owner applies them in the order they
            // were made even when the first is retried.
            if (op.pduType == BEEBUS_PDU_TOPIC && op.req.function == BEEBUS_TOP_WRITE &&
                op.wantsAck) {
                bool behind = false;
                for (size_t j = 0; j < i && !behind; j++) {
                    const BeeBusPendingModbusOp& earlier = pending[j];
                    behind = earlier.pduType == BEEBUS_PDU_TOPIC &&
                             earlier.destNodeID == op.destNodeID &&
                             earlier.req.function == BEEBUS_TOP_WRITE &&
                             earlier.req.startAddr == op.req.startAddr;
                }
                if (behind) continue;
            }
            ready.push_back(OpKey{op.destNodeID, op.opId,
                                  bb_opExpectsResponse(op) || op.wantsAck});
        }
    }
    const size_t responseCount = operations.getPendingResponses().size();

    for (size_t i = 0; i < unbuildable.size(); i++) {
        BEEBUS_DEBUG_IO("FRAME BUILD: Dropping unencodable operation for Node:%d",
            unbuildable[i].node);
        operations.removePendingOperation(unbuildable[i].node, unbuildable[i].opId,
                                          BeeBusResult::InvalidArgument);
        _protocol.reportError(BeeBusError::InvalidFunction,
                              "Unsupported function code, operation dropped");
    }

    BeeBusHeader header = tokenHeader;
    if (ready.size() + responseCount > 0) {
        header.flags |= BEEBUS_HDR_HAS_SECTIONS;
    }

    uint16_t pos = BeeBusFrame::buildHeader(buffer, BEEBUS_MAX_TX_BUFFER, header);
    if (pos == 0) {
        return false;
    }
    const uint16_t flagsPos = BEEBUS_HDR_FLAGS;

    // Each section in turn: the ready operations, in queue order, then the
    // responses. The build stops at the first that will not fit; whatever was
    // left behind stays queued for the next token.
    BeeBusList<OpKey>& sentOps = operations.frameSent;
    sentOps.clear();
    size_t responsesSent = 0;
    bool full = false;

    const BeeBusList<BeeBusPendingModbusOp>& pending = operations.getPendingOps();
    for (size_t k = 0; k < ready.size() && !full; k++) {
        const BeeBusPendingModbusOp* found = nullptr;
        for (size_t i = 0; i < pending.size(); i++) {
            if (pending[i].destNodeID == ready[k].node && pending[i].opId == ready[k].opId) {
                found = &pending[i];
                break;
            }
        }
        if (!found) {
            continue;   // Concluded meanwhile by a handler
        }
        const uint16_t sectionAt = pos;
        if (!appendSection(buffer, pos, found->pduType, found->destNodeID, found->req,
                           found, found->wantsAck && !found->req.isResponse, found->opId)) {
            full = true;
            break;
        }
        // Numbered as it goes into the frame, in the order it goes: stream
        // numbers and request IDs are only ever spent on what is sent.
        if (found->pduType == BEEBUS_PDU_TOPIC) {
            BeeBusPendingModbusOp* stored = operations.findByOpId(found->destNodeID, found->opId);
            _protocol.stampTopicSection(found->destNodeID, &buffer[sectionAt + 3],
                                        (uint16_t)found->req.data.size(),
                                        stored ? &stored->req : nullptr);
        }
        sentOps.push_back(ready[k]);
    }

    const BeeBusList<BeeBusPendingResponse>& responses = operations.getPendingResponses();
    for (size_t i = 0; i < responseCount && i < responses.size() && !full; i++) {
        const BeeBusPendingResponse& resp = responses[i];
        const uint16_t sectionAt = pos;
        if (!appendSection(buffer, pos, resp.pduType, resp.destNodeID, resp.response,
                           nullptr, false, 0)) {
            full = true;
            break;
        }
        if (resp.pduType == BEEBUS_PDU_TOPIC) {
            _protocol.stampTopicSection(resp.destNodeID, &buffer[sectionAt + 3],
                                        (uint16_t)resp.response.data.size(), nullptr);
        }
        responsesSent++;
    }

    // Nothing fitted after all, so the frame does not carry sections and must
    // not claim to. The bit pulls no byte in, so clearing it in place is safe.
    if (sentOps.size() + responsesSent == 0) {
        buffer[flagsPos] &= (uint8_t)~BEEBUS_HDR_HAS_SECTIONS;
    }

    const uint16_t frameLen = BeeBusFrame::finish(buffer, BEEBUS_MAX_TX_BUFFER, pos);
    if (frameLen == 0) {
        return false;
    }

    const bool sent = sendFrame(buffer, frameLen);

    if (sent) {
        // Retire only what went into the frame. The build loop stops as soon as
        // a section will not fit, and an operation left behind stays queued:
        // the caller was told it was accepted, so dropping it loses a write.
        // The responses sent are the first responsesSent of the queue, taken
        // before any operation retires (a retirement never adds a response).
        BeeBusList<BeeBusOperations::FrameResponse>& sentResponses = operations.frameResponses;
        sentResponses.clear();
        for (size_t i = 0; i < responsesSent; i++) {
            sentResponses.push_back(BeeBusOperations::FrameResponse{
                responses[i].pduType, responses[i].destNodeID,
                responses[i].response.function, responses[i].response.startAddr});
        }
        for (size_t i = 0; i < sentOps.size(); i++) {
            if (sentOps[i].keep) {
                // A read is kept so its response can be matched back to the
                // caller's variable; a confirmed write is kept so it can be
                // retried if no acknowledgement arrives. An unconfirmed write
                // is finished the moment it is transmitted.
                operations.markAwaitingResponse(sentOps[i].node, sentOps[i].opId);
            } else {
                // Nothing will answer it, and nothing needs to: it is on the
                // wire, which is all an unconfirmed write promises.
                operations.removePendingOperation(sentOps[i].node, sentOps[i].opId,
                                                  BeeBusResult::Immediate);
            }
        }
        for (size_t i = 0; i < sentResponses.size(); i++) {
            const BeeBusOperations::FrameResponse& r = sentResponses[i];
            operations.removePendingResponse(r.type, r.node, r.function, r.address);
        }
    }

    return sent;
}

bool BeeBusIO::appendSection(uint8_t* buffer, uint16_t& pos, uint8_t pduType,
                             uint8_t destNodeID, const BeeBusModbusRequest& req,
                             const BeeBusPendingModbusOp* op, bool wantsAck,
                             uint16_t opId) {
    // Two bytes of section header, one destination byte, and enough PDU to
    // be worth starting; plus the CRC that has to follow.
    if (pos + 8 > BEEBUS_MAX_TX_BUFFER - 2) {
        BEEBUS_DEBUG_IO("FRAME BUILD: Stopping - insufficient space at pos %d", pos);
        return false;
    }

    const uint16_t posBeforeSection = pos;

    // Sections are length-prefixed, so no payload byte needs a delimiter.
    // The length covers the destination ID plus the PDU, and is backfilled
    // once the PDU size is known.
    const uint16_t lengthPos = pos;
    pos += 2;
    buffer[pos++] = destNodeID;

    // Room left for the PDU, keeping two bytes back for the CRC. The PDU
    // builders cannot see the frame they are writing into, so the bound has
    // to come from here.
    const uint16_t available = (uint16_t)(BEEBUS_MAX_TX_BUFFER - pos - 2);
    uint16_t modbusLen = 0;
    if (pduType == BEEBUS_PDU_TOPIC) {
        // Encoded when the operation was queued: bounds check and copy.
        if (req.data.size() <= available) {
            memcpy(&buffer[pos], req.data.data(), req.data.size());
            modbusLen = (uint16_t)req.data.size();
        }
    } else if (req.isResponse || !op) {
        modbusLen = BeeBusModbusFrame::buildModbusResponse(&buffer[pos], available, req);
    } else {
        modbusLen = BeeBusModbusFrame::buildModbusRequest(&buffer[pos], available, op);
    }

    if (modbusLen == 0 || pos + modbusLen >= BEEBUS_MAX_TX_BUFFER - 2) {
        pos = posBeforeSection;
        return false;
    }

    pos += modbusLen;

    uint16_t sectionLength = (uint16_t)(1 + modbusLen);
    uint16_t sectionFlags = (uint16_t)((uint16_t)pduType << BEEBUS_SECTION_PDU_SHIFT);

    // Say which direction this is, rather than leaving the receiver to work
    // it out from the shape of a PDU that does not always give it away.
    if (req.isResponse) {
        sectionFlags |= BEEBUS_SECTION_RESPONSE;
    }

    // An operation asking to be acknowledged carries its ID after the PDU,
    // and flags the section so the receiver knows those bytes are not PDU
    // content. Nothing is added to operations that did not ask.
    if (wantsAck) {
        if (pos + BEEBUS_ACK_ID_BYTES >= BEEBUS_MAX_TX_BUFFER - 2) {
            pos = posBeforeSection;
            return false;
        }
        bb_packBE16(&buffer[pos], opId);
        pos += 2;
        sectionLength = (uint16_t)(sectionLength + BEEBUS_ACK_ID_BYTES);
        sectionFlags |= BEEBUS_SECTION_ACK_REQUESTED;
    }

    // The length shares its 16 bits with the flags, so a section too long
    // to express would overwrite them and be read back as a different
    // PDU type or a spurious acknowledgement request. Nothing Modbus builds
    // comes close, but the bound belongs here rather than in the callers.
    if (sectionLength > BEEBUS_SECTION_LEN_MASK) {
        pos = posBeforeSection;
        return false;
    }

    bb_packBE16(&buffer[lengthPos], (uint16_t)(sectionLength | sectionFlags));
    return true;
}

bool BeeBusIO::sendFrame(const uint8_t* buffer, uint16_t length) {
    if (!_transport || length == 0) {
        return false;
    }

    if (length > BEEBUS_MAX_TX_BUFFER) {
        return false;
    }

    // Escape the body and wrap it in flags. buffer[0] is the frame marker and
    // becomes the opening flag, so it is written raw and not escaped.
    uint16_t wireLen = 0;
    _txWireBuffer[wireLen++] = BEEBUS_FLAG;

    for (uint16_t i = 1; i < length; i++) {
        const uint8_t value = buffer[i];
        if (value == BEEBUS_FLAG || value == BEEBUS_ESC) {
            _txWireBuffer[wireLen++] = BEEBUS_ESC;
            _txWireBuffer[wireLen++] = value ^ BEEBUS_ESC_XOR;
        } else {
            _txWireBuffer[wireLen++] = value;
        }
    }

    _txWireBuffer[wireLen++] = BEEBUS_FLAG;

    // Remember what went out, so a transport that echoes is not mistaken for
    // another node using our address.
    _lastTxCrc = bb_unpackBE16(&buffer[length - 2]);
    _lastTxLen = length;
    _lastTxAt = bb_millis();

    size_t bytesWritten = _transport->write(_txWireBuffer, wireLen);

    if (bytesWritten == wireLen) {
        incrementFrameSent();
        BEEBUS_DEBUG_FRAME(BEEBUS_FRAME_TX, buffer, length);
        return true;
    } else {
        BEEBUS_DEBUG_IO("TX: Failed to send frame, only %u of %u bytes written",
            (unsigned)bytesWritten, (unsigned)wireLen);
        return false;
    }
}

bool BeeBusIO::isOwnEcho(const uint8_t* buffer, uint16_t length) const {
    if (_lastTxLen == 0 || length != _lastTxLen) {
        return false;
    }

    // An echo arrives while the frame is still going out, so anything older
    // than one interframe gap is somebody else transmitting the same bytes,
    // which a duplicate address is exactly what would produce.
    const bb_time_t age = bb_elapsed(_lastTxAt, bb_millis());
    if (age > _protocol.interframeGapMs() + 1) {
        return false;
    }

    const uint16_t crc = bb_unpackBE16(&buffer[length - 2]);
    return crc == _lastTxCrc;
}

// --- Measured bus timing ---
void BeeBusIO::noteArrival(bb_time_t arrivedUs) {
    if (_runBytes == 0) {
        _runBytes = 1;
        _runLastUs = arrivedUs;
        _runSpanUs = 0;
        return;
    }

    const bb_time_t delta = bb_elapsed(_runLastUs, arrivedUs);
    _runLastUs = arrivedUs;

    // Silence long enough to have been a gap between transmissions ends the
    // run. What follows was carried separately, and a run spanning the gap
    // reports the line as slower by however long the bus was idle.
    if (delta > BEEBUS_BYTE_RUN_GAP_US) {
        closeRun();
        _runBytes = 1;
        return;
    }

    if (_runBytes == 1 || delta < _runMinDeltaUs) _runMinDeltaUs = delta;
    if (_runBytes == 1 || delta > _runMaxDeltaUs) _runMaxDeltaUs = delta;
    _runSpanUs += delta;

    // Longer than any frame could be, so nothing is carrying it: a floating
    // line or a driver stuck on produces bytes without having carried one.
    if (_runBytes > BEEBUS_BYTE_RUN_MAX_BYTES) {
        closeRun();
        return;
    }
    _runBytes++;
}

void BeeBusIO::closeRun() {
    // A line clocks its bytes out evenly. Bytes read a bounded number at a
    // time out of a backlog do not: several arrive together and then a whole
    // pass through loop() goes by before the next. Judged on the spread, so
    // what is measured is the bus rather than this node's own read pacing, and
    // so that a collision wreck counts as readily as a frame: it was carried
    // at the same rate, and a node hearing only wrecks has nothing else.
    const bool even = _runBytes > BEEBUS_BYTE_TIME_MIN_BYTES &&
                      _runMinDeltaUs != 0 &&
                      _runMinDeltaUs <= BEEBUS_BYTE_TIME_MAX_US &&
                      _runMaxDeltaUs <= _runMinDeltaUs * BEEBUS_BYTE_RUN_SPREAD;

    // Averaged across the run rather than taken from its narrowest gap. Where
    // a byte is read a little late the next reads early, so the narrowest gap
    // is shorter than the line can actually carry one, and a gap sized from it
    // would be too short. Evenness has already ruled out any idle being
    // averaged in, which is the only thing the mean would be wrong about.
    const bb_time_t mean = even ? _runSpanUs / (bb_time_t)(_runBytes - 1) : 0;

    // Lowest in the window wins. A run can be stretched by this node being slow
    // to read it and never compressed below what the line allows, so every
    // error in a run that passed the evenness test is upward.
    if (mean != 0 && mean <= BEEBUS_BYTE_TIME_MAX_US) {
        noteByteTime((uint16_t)mean);
    }

    _runBytes = 0;
    _runSpanUs = 0;
    _runMinDeltaUs = 0;
    _runMaxDeltaUs = 0;
}

void BeeBusIO::noteByteTime(uint16_t meanUs) {
    _byteTimeRuns[_byteTimeNext] = meanUs;
    _byteTimeNext = (uint8_t)((_byteTimeNext + 1) % BEEBUS_BYTE_TIME_WINDOW);
    if (_byteTimeCount < BEEBUS_BYTE_TIME_WINDOW) _byteTimeCount++;

    uint16_t lowest = 0;
    for (uint8_t i = 0; i < _byteTimeCount; i++) {
        const uint16_t sample = _byteTimeRuns[i];
        if (sample != 0 && (lowest == 0 || sample < lowest)) lowest = sample;
    }
    _byteTimeUs = lowest;
}

bb_time_t BeeBusIO::lineByteTimeUs() const {
    // From the stated line rate where the transport knows it, and from the
    // spacing of arrivals otherwise. Spacing reads as nothing on a line fast
    // enough to deliver a whole frame between two looks at the bus.
    if (_transport != nullptr) {
        const uint32_t bps = _transport->bitsPerSecond();
        if (bps != 0) return (bb_time_t)((10ull * 1000000ull) / bps);   // 8N1
    }
    return _byteTimeUs;
}

bb_time_t BeeBusIO::effectiveGapUs() const {
    const bb_time_t configured = _protocol.config().interframeGapUs;
    const bb_time_t byteUs = lineByteTimeUs();

    if (byteUs == 0) {
        // Nothing known yet, from either source.
        return configured != BEEBUS_AUTO ? configured : BEEBUS_GAP_COLD_US;
    }

    // Two byte times, so a node answering inside the gap still has room for a
    // whole byte to land before the gap it is answering in runs out.
    bb_time_t gap = byteUs * 2;

    // Never shorter than this node looks at the bus: silence never observed
    // cannot be acted on.
    const bb_time_t serviceUs = _protocol.serviceIntervalUs();
    if (serviceUs > gap) gap = serviceUs;

    // A configured gap is a floor only where it was asked for; the default is
    // what to assume before the line is known, not a bound on it.
    if (configured != BEEBUS_AUTO && gap < configured) {
        gap = configured;
    }
    return gap;
}

bb_time_t BeeBusIO::silenceUs() const {
    return bb_elapsed(_lastBusActivityUs, bb_micros());
}

// --- Transmission utilities ---
bool BeeBusIO::isTransmissionReady() {
    if (!_transport) {
        return false;
    }

    return silenceUs() >= effectiveGapUs();
}

// --- Statistics ---
void BeeBusIO::resetStatistics() {
    // A fresh struct rather than field-by-field clearing, so a counter added
    // later cannot be left out and survive its own reset.
    _stats = BeeBusIOStats();
}

BeeBusIOStats BeeBusIO::getStatistics() {
    return _stats;
}

void BeeBusIO::incrementFrameReceived() {
    _stats.framesReceived++;
}

void BeeBusIO::incrementFrameSent() {
    _stats.framesSent++;
}

void BeeBusIO::incrementCrcError() {
    _stats.crcErrors++;

    // A corrupt frame on an otherwise quiet bus is how two nodes answering the
    // same solicitation announce themselves. Nothing else can see it.
    _protocol.noteCorruptFrame();
}

void BeeBusIO::incrementFramingError() {
    _stats.framingErrors++;

    // Two transmissions overlapping on a shared line rarely leave anything
    // frame shaped behind. A runt between two flags is the same evidence of
    // contention a bad CRC is, and reading only the CRC leaves the response
    // window looking empty when it was in fact fought over.
    _protocol.noteCorruptFrame();
}

void BeeBusIO::incrementBufferOverflow() {
    _stats.bufferOverflows++;
}
