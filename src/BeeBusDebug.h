#pragma once
#include "BeeBusGlobal.h"

// Boards leave the debug system out unless BEEBUS_ENABLE_DEBUG is defined: the
// message formatting and frame decoding cost about 10 KB of flash, and a board
// in the field has nobody reading the console. A host build keeps it unless
// DISABLE_BEEBUS_DEBUG is defined. onDebug() and the other handlers stay
// declared either way, so callers compile in every build; without the debug
// system they do nothing.
#if defined(BEEBUS_PLATFORM_ARDUINO) && !defined(BEEBUS_ENABLE_DEBUG) && !defined(DISABLE_BEEBUS_DEBUG)
    #define DISABLE_BEEBUS_DEBUG
#endif

#ifdef DISABLE_BEEBUS_DEBUG
    // Replace all debug macros with empty ones
    #define BEEBUS_DEBUG(level, category, ...)
    #define BEEBUS_DEBUG_FRAME(direction, frame, length)
    #define BEEBUS_DEBUG_MODBUS(type, nodeID, request)
    #define BEEBUS_DEBUG_PROTOCOL(...)
    #define BEEBUS_DEBUG_FRAMES(...)
    #define BEEBUS_DEBUG_MODBUS_INFO(...)
    #define BEEBUS_DEBUG_OPERATIONS(...)
    #define BEEBUS_DEBUG_IO(...)
    #define BEEBUS_DEBUG_ERROR(...)
    #define BEEBUS_DEBUG_WARN(...)
    #define BEEBUS_DEBUG_VERBOSE(...)
    #define BEEBUS_DEBUG_NODE(nodeID)

    // Empty debug class. BeeBusAPI::onDebug compiles out entirely in this
    // build, so no handler registration is declared here.
    class BeeBusDebug {
    public:
        BeeBusDebug() {}
        static bool isGlobalDebugEnabled() { return false; }
    };
    extern BeeBusDebug g_beebusDebug;

#else
    // --- Debug system when enabled ---

    // Debug levels
    enum BeeBusDebugLevel {
        BEEBUS_DEBUG_NONE = 0,
        BEEBUS_DEBUG_ERROR = 1,
        BEEBUS_DEBUG_WARN = 2,
        BEEBUS_DEBUG_INFO = 3,
        BEEBUS_DEBUG_VERBOSE = 4,
        BEEBUS_DEBUG_ALL = 5
    };

    // Debug categories
    enum BeeBusDebugCategory {
        BEEBUS_DEBUG_PROTOCOL = 0x01,
        BEEBUS_DEBUG_FRAMES = 0x02,
        BEEBUS_DEBUG_MODBUS = 0x04,
        BEEBUS_DEBUG_OPERATIONS = 0x08,
        BEEBUS_DEBUG_IO = 0x20
    };

    // Frame direction
    enum BeeBusFrameDirection {
        BEEBUS_FRAME_TX,
        BEEBUS_FRAME_RX
    };

    // Modbus operation type
    enum BeeBusModbusType {
        BEEBUS_MODBUS_REQUEST,
        BEEBUS_MODBUS_RESPONSE
    };

    // Callback function types
    typedef void (*BeeBusDebugHandler)(BeeBusDebugLevel level, BeeBusDebugCategory category, const char* message);
    typedef void (*BeeBusFrameDebugHandler)(BeeBusFrameDirection direction, const uint8_t* frame, uint16_t length, const char* decoded);
    typedef void (*BeeBusModbusDebugHandler)(BeeBusModbusType type, uint8_t nodeID, const BeeBusModbusRequest& request, const char* decoded);

    /**
     * BeeBusDebug - Comprehensive debugging system for BeeBus protocol
     */
    class BeeBusDebug {
    public:
        BeeBusDebug();
        ~BeeBusDebug();

        // Debug configuration

        // Which node the messages that follow belong to. The debug instance is
        // process-wide, so several nodes in one process share it; this is what
        // keeps their output apart.
        void setCurrentNode(uint8_t nodeID);

        // Debug handlers
        void onDebug(BeeBusDebugHandler handler);
        void onFrameDebug(BeeBusFrameDebugHandler handler);
        void onModbusDebug(BeeBusModbusDebugHandler handler);

        // Debug output methods
        void debug(BeeBusDebugLevel level, BeeBusDebugCategory category, const char* format, ...);
        void debugFrame(BeeBusFrameDirection direction, const uint8_t* frame, uint16_t length);
        void debugModbus(BeeBusModbusType type, uint8_t nodeID, const BeeBusModbusRequest& request);

        // Frame analysis
        void analyzeFrame(const uint8_t* frame, uint16_t length, char* output, size_t outputSize);
        void analyzeModbusRequest(const BeeBusModbusRequest& request, char* output, size_t outputSize);

        // Protocol state debugging

        // Statistics

        // Utility methods
        const char* getFrameTypeString(const uint8_t* frame, uint16_t length);
        const char* getModbusFunctionString(uint8_t functionCode);

        // Enable/disable debug globally
        static bool isGlobalDebugEnabled();

    private:
        BeeBusDebugLevel _debugLevel;
        uint8_t _debugCategories;
        uint8_t _currentNodeID;

        BeeBusDebugHandler _debugHandler;
        BeeBusFrameDebugHandler _frameDebugHandler;
        BeeBusModbusDebugHandler _modbusDebugHandler;

        // Statistics
        uint32_t _framesSent;
        uint32_t _framesReceived;
        uint32_t _modbusRequestsSent;
        uint32_t _modbusResponsesReceived;

        // Helper methods
        bool shouldDebug(BeeBusDebugLevel level, BeeBusDebugCategory category) const;
        void formatHexDump(const uint8_t* data, uint16_t length, char* output, size_t outputSize);
        void decodeFrameHeader(const uint8_t* frame, uint16_t length, char* output, size_t outputSize);
        void decodeModbusSections(const uint8_t* frame, uint16_t length, char* output, size_t outputSize);

        static bool _globalDebugEnabled;
    };

    // Global debug instance
    extern BeeBusDebug g_beebusDebug;

    // --- Debug macros ---

    // Convenience macros for debug output
    #define BEEBUS_DEBUG(level, category, ...) \
        do { \
            if (BeeBusDebug::isGlobalDebugEnabled()) { \
                g_beebusDebug.debug(level, category, __VA_ARGS__); \
            } \
        } while(0)

    #define BEEBUS_DEBUG_FRAME(direction, frame, length) \
        do { \
            if (BeeBusDebug::isGlobalDebugEnabled()) { \
                g_beebusDebug.debugFrame(direction, frame, length); \
            } \
        } while(0)

    #define BEEBUS_DEBUG_MODBUS(type, nodeID, request) \
        do { \
            if (BeeBusDebug::isGlobalDebugEnabled()) { \
                g_beebusDebug.debugModbus(type, nodeID, request); \
            } \
        } while(0)

    // Quick debug macros
    #define BEEBUS_DEBUG_PROTOCOL(...) BEEBUS_DEBUG(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_PROTOCOL, __VA_ARGS__)
    #define BEEBUS_DEBUG_FRAMES(...) BEEBUS_DEBUG(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_FRAMES, __VA_ARGS__)
    #define BEEBUS_DEBUG_MODBUS_INFO(...) BEEBUS_DEBUG(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_MODBUS, __VA_ARGS__)
    #define BEEBUS_DEBUG_OPERATIONS(...) BEEBUS_DEBUG(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_OPERATIONS, __VA_ARGS__)
    #define BEEBUS_DEBUG_IO(...) BEEBUS_DEBUG(BEEBUS_DEBUG_INFO, BEEBUS_DEBUG_IO, __VA_ARGS__)

    #define BEEBUS_DEBUG_ERROR(...) BEEBUS_DEBUG(BEEBUS_DEBUG_ERROR, BEEBUS_DEBUG_PROTOCOL, __VA_ARGS__)
    #define BEEBUS_DEBUG_WARN(...) BEEBUS_DEBUG(BEEBUS_DEBUG_WARN, BEEBUS_DEBUG_PROTOCOL, __VA_ARGS__)
    #define BEEBUS_DEBUG_VERBOSE(...) BEEBUS_DEBUG(BEEBUS_DEBUG_VERBOSE, BEEBUS_DEBUG_PROTOCOL, __VA_ARGS__)

    // Names the node whose messages follow.
    #define BEEBUS_DEBUG_NODE(nodeID) \
        do { \
            if (BeeBusDebug::isGlobalDebugEnabled()) { \
                g_beebusDebug.setCurrentNode(nodeID); \
            } \
        } while(0)
#endif
