#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

#ifdef DISABLE_BEEBUS_DEBUG
    // --- Debug disabled - minimal implementation ---

    // Just the global instance
    BeeBusDebug g_beebusDebug;

#else
    // --- Debug enabled - full implementation ---

    // Global debug instance
    BeeBusDebug g_beebusDebug;
    bool BeeBusDebug::_globalDebugEnabled = true;

    // --- Constructor and destructor ---

    BeeBusDebug::BeeBusDebug() :
        _debugLevel(BEEBUS_DEBUG_INFO),
        _debugCategories(0xFF), // All categories enabled by default
        _currentNodeID(0),
        _debugHandler(nullptr),
        _frameDebugHandler(nullptr),
        _modbusDebugHandler(nullptr),
        _framesSent(0),
        _framesReceived(0),
        _modbusRequestsSent(0),
        _modbusResponsesReceived(0)
    {
    }

    BeeBusDebug::~BeeBusDebug() {
    }

    // --- Debug handler registration ---

    void BeeBusDebug::setCurrentNode(uint8_t nodeID) {
        _currentNodeID = nodeID;
    }

    void BeeBusDebug::onDebug(BeeBusDebugHandler handler) {
        _debugHandler = handler;
    }

    void BeeBusDebug::onFrameDebug(BeeBusFrameDebugHandler handler) {
        _frameDebugHandler = handler;
    }

    void BeeBusDebug::onModbusDebug(BeeBusModbusDebugHandler handler) {
        _modbusDebugHandler = handler;
    }

    // --- Core debug output methods ---

    void BeeBusDebug::debug(BeeBusDebugLevel level, BeeBusDebugCategory category, const char* format, ...) {
        if (!shouldDebug(level, category) || !_debugHandler) {
            return;
        }

        char buffer[512];
        int prefix = 0;
        if (_currentNodeID != 0) {
            prefix = snprintf(buffer, sizeof(buffer), "[node %u] ", _currentNodeID);
            if (prefix < 0 || (size_t)prefix >= sizeof(buffer)) prefix = 0;
        }

        va_list args;
        va_start(args, format);
        vsnprintf(buffer + prefix, sizeof(buffer) - (size_t)prefix, format, args);
        va_end(args);

        _debugHandler(level, category, buffer);
    }

    void BeeBusDebug::debugFrame(BeeBusFrameDirection direction, const uint8_t* frame, uint16_t length) {
        if (!shouldDebug(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_FRAMES) || !_frameDebugHandler) {
            return;
        }

        // Update statistics
        if (direction == BEEBUS_FRAME_TX) {
            _framesSent++;
        } else {
            _framesReceived++;
        }

        char hexOutput[1024];
        char* p = hexOutput;
        size_t remaining = sizeof(hexOutput) - 1;

        const char* dirStr = (direction == BEEBUS_FRAME_TX) ? "TX" : "RX";
        int written = snprintf(p, remaining, "[BeeBus Debug FRAMES] %s: ", dirStr);
        if (written > 0 && (size_t)written < remaining) {
            p += written;
            remaining -= written;
        }

        // Add hex data
        for (uint16_t i = 0; i < length && remaining > 3; i++) {
            written = snprintf(p, remaining, "%02X ", frame[i]);
            if (written > 0 && (size_t)written < remaining) {
                p += written;
                remaining -= written;
            }
        }

        // Terminating at p would write over the first character of what was
        // just appended, so the count never appeared. snprintf terminates for
        // itself; advancing past it is all that is needed.
        written = snprintf(p, remaining, "(%d bytes)", length);
        if (written > 0 && (size_t)written < remaining) {
            p += written;
            remaining -= written;
        }

        // Output via debug handler
        debug(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_FRAMES, "%s", hexOutput);

        // Analyze frame
        char analysis[1024];
        analyzeFrame(frame, length, analysis, sizeof(analysis));

        if (_frameDebugHandler) {
            _frameDebugHandler(direction, frame, length, analysis);
        }
    }

    void BeeBusDebug::debugModbus(BeeBusModbusType type, uint8_t nodeID, const BeeBusModbusRequest& request) {
        if (!shouldDebug(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_MODBUS) || !_modbusDebugHandler) {
            return;
        }

        // Update statistics
        if (type == BEEBUS_MODBUS_REQUEST) {
            _modbusRequestsSent++;
        } else {
            _modbusResponsesReceived++;
        }

        // Analyze Modbus request
        char analysis[512];
        analyzeModbusRequest(request, analysis, sizeof(analysis));

        _modbusDebugHandler(type, nodeID, request, analysis);
    }

    // --- Frame and modbus analysis methods ---

    void BeeBusDebug::analyzeFrame(const uint8_t* frame, uint16_t length, char* output, size_t outputSize) {
        if (!frame || length == 0 || !output || outputSize == 0) {
            return;
        }

        char* p = output;
        size_t remaining = outputSize - 1; // Reserve space for null terminator

        // Frame type and basic info
        const char* frameType = getFrameTypeString(frame, length);
        int written = snprintf(p, remaining, "Type: %s, Length: %d\n", frameType, length);
        if (written < 0 || (size_t)written >= remaining) return;
        p += written;
        remaining -= written;

        // Hex dump
        written = snprintf(p, remaining, "Raw: ");
        if (written < 0 || (size_t)written >= remaining) return;
        p += written;
        remaining -= written;

        formatHexDump(frame, length, p, remaining);
        size_t hexLen = strlen(p);
        p += hexLen;
        remaining -= hexLen;

        written = snprintf(p, remaining, "\n");
        if (written < 0 || (size_t)written >= remaining) return;
        p += written;
        remaining -= written;

        // Decode header
        decodeFrameHeader(frame, length, p, remaining);
        size_t headerLen = strlen(p);
        p += headerLen;
        remaining -= headerLen;

        // Decode Modbus sections if present
        decodeModbusSections(frame, length, p, remaining);

        *p = '\0'; // Ensure null termination
    }

    void BeeBusDebug::analyzeModbusRequest(const BeeBusModbusRequest& request, char* output, size_t outputSize) {
        if (!output || outputSize == 0) {
            return;
        }

        const char* functionName = getModbusFunctionString(request.function);
        const char* type = request.isResponse ? "RESP" : "REQ";

        snprintf(output, outputSize,
            "%s | FC:%02X (%s) | Addr:%d | Qty:%d | DataLen:%d",
            type, request.function, functionName,
            request.startAddr, request.quantity, (int)request.data.size());
    }

    // --- Private helper methods ---

    bool BeeBusDebug::shouldDebug(BeeBusDebugLevel level, BeeBusDebugCategory category) const {
        return _globalDebugEnabled &&
               level <= _debugLevel &&
               (_debugCategories & category) != 0;
    }

    void BeeBusDebug::formatHexDump(const uint8_t* data, uint16_t length, char* output, size_t outputSize) {
        if (!data || length == 0 || !output || outputSize == 0) {
            return;
        }

        char* p = output;
        size_t remaining = outputSize - 1;

        for (uint16_t i = 0; i < length && remaining > 3; i++) {
            int written = snprintf(p, remaining, "%02X ", data[i]);
            if (written < 0 || (size_t)written >= remaining) break;
            p += written;
            remaining -= written;
        }

        *p = '\0';
    }

    void BeeBusDebug::decodeFrameHeader(const uint8_t* frame, uint16_t length, char* output, size_t outputSize) {
        if (!frame || length < BEEBUS_MIN_FRAME_LEN || !output || outputSize == 0) {
            return;
        }

        BeeBusHeader header;
        if (!BeeBusFrame::parseHeader(frame, length, header)) {
            snprintf(output, outputSize, "Header: INVALID\n");
            return;
        }

        snprintf(output, outputSize,
            "Header: SRC:%d, Next:%d, Flags:%02X, Round:%d, Window:%d, Joined:%d, Left:%d\n",
            header.srcNodeID, header.successorID, header.flags,
            header.solicitRound, header.solicitWindow,
            header.joinedNodeID, header.leftNodeID);
    }

    void BeeBusDebug::decodeModbusSections(const uint8_t* frame, uint16_t length, char* output, size_t outputSize) {
        if (!frame || length == 0 || !output || outputSize == 0) {
            return;
        }

        BeeBusHeader header;
        if (!BeeBusFrame::parseHeader(frame, length, header)) {
            return;
        }

        // Walked twice: once for the count the text starts with, then for each.
        int sectionCount = 0;
        uint16_t pos = 0;
        BeeBusSection section;
        while (BeeBusFrame::nextSection(frame, length, header, pos, section)) {
            sectionCount++;
        }

        char* p = output;
        size_t remaining = outputSize - 1;

        int written = snprintf(p, remaining, "Modbus Sections: %d\n", sectionCount);
        if (written < 0 || (size_t)written >= remaining) return;
        p += written;
        remaining -= written;

        pos = 0;
        for (int i = 0; BeeBusFrame::nextSection(frame, length, header, pos, section); i++) {
            uint16_t start = section.start;
            uint16_t end = section.end;

            if (start < length) {
                uint8_t targetNodeID = frame[start];
                written = snprintf(p, remaining, "  Section %d: Target:%d, Bytes:%d\n",
                    i, targetNodeID, end - start - 1);
                if (written < 0 || (size_t)written >= remaining) break;
                p += written;
                remaining -= written;
            }
        }

        *p = '\0';
    }

    // --- String conversion utility methods ---

    const char* BeeBusDebug::getFrameTypeString(const uint8_t* frame, uint16_t length) {
        if (!frame || length < BEEBUS_MIN_FRAME_LEN) {
            return "INVALID";
        }

        BeeBusHeader header;
        if (!BeeBusFrame::parseHeader(frame, length, header)) {
            return "INVALID";
        }

        if (header.has(BEEBUS_HDR_HAS_SECTIONS))  return "DATA";
        if (header.has(BEEBUS_HDR_SOLICIT))       return "SOLICIT";
        if (header.has(BEEBUS_HDR_MEMBER_LEFT))   return "MEMBER_LEFT";
        if (header.has(BEEBUS_HDR_MEMBER_JOINED)) return "MEMBER_JOINED";
        if (header.successorID != 0)              return "TOKEN";
        return "CONTROL";
    }

    const char* BeeBusDebug::getModbusFunctionString(uint8_t functionCode) {
        switch (functionCode & 0x7F) {
            case MB_FC_READ_COILS: return "READ_COILS";
            case MB_FC_READ_DISCRETE_INPUTS: return "READ_DISCRETE_INPUTS";
            case MB_FC_READ_HOLDING_REGISTERS: return "READ_HOLDING_REGISTERS";
            case MB_FC_READ_INPUT_REGISTERS: return "READ_INPUT_REGISTERS";
            case MB_FC_WRITE_SINGLE_COIL: return "WRITE_SINGLE_COIL";
            case MB_FC_WRITE_SINGLE_REGISTER: return "WRITE_SINGLE_REGISTER";
            case MB_FC_WRITE_MULTIPLE_COILS: return "WRITE_MULTIPLE_COILS";
            case MB_FC_WRITE_MULTIPLE_REGISTERS: return "WRITE_MULTIPLE_REGISTERS";
            default: return "UNKNOWN";
        }
    }

    // --- Global debug control ---

    bool BeeBusDebug::isGlobalDebugEnabled() {
        return _globalDebugEnabled;
    }

    // --- Statistics methods ---

#endif
