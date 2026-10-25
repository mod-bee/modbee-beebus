#pragma once
#include "BeeBusGlobal.h"

/**
 * BeeBusModbusFrame - builds and parses Modbus PDUs: the function code and
 * its payload, with no address or CRC. BeeBus carries them as sections of its
 * own frames, which supply both.
 */
class BeeBusModbusFrame {
public:
    // --- Core request/response building ---
    static uint16_t buildModbusRequest(uint8_t* buffer, uint16_t capacity, const BeeBusPendingModbusOp* op);
    static uint16_t buildModbusResponse(uint8_t* buffer, uint16_t capacity, const BeeBusModbusRequest& response);

    // --- Parsing functions ---
    static bool parseModbusRequest(const uint8_t* buffer, uint16_t len, BeeBusModbusRequest& req);
    static bool parseModbusResponse(const uint8_t* buffer, uint16_t len, BeeBusModbusRequest& resp);

    // --- Function codes ---
    static bool isValidFunctionCode(uint8_t functionCode);
    static bool isReadFunction(uint8_t functionCode);
    static bool isErrorResponse(uint8_t functionCode);
    static uint8_t makeErrorResponse(uint8_t functionCode);

    // --- Request/response validation ---
    // False when startAddr + quantity would run past the end of the 16 bit
    // address space and wrap around to address 0.
    static bool isValidAddressRange(uint16_t startAddr, uint16_t quantity);

    static bool validateRequest(const BeeBusModbusRequest& request);

    // --- Bit packing ---
    static void packBits(const bool* bits, uint8_t* packed, uint16_t quantity);
    static void unpackBits(const uint8_t* packed, bool* bits, uint16_t quantity);
    static uint8_t getBitPackedBytes(uint16_t quantity);
};
