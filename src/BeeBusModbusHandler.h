#pragma once
#include "BeeBusGlobal.h"

// Forward declarations
class BeeBusProtocol;

/**
 * BeeBusModbusHandler - Processes Modbus requests and builds responses
 *
 * This class handles the business logic of processing incoming Modbus requests
 * against the local data map and building appropriate responses. It also manages
 * request/response matching and operation grouping for efficient transmission.
 */
class BeeBusModbusHandler {
public:
    // --- Constructor and destructor ---
    BeeBusModbusHandler(BeeBusModbusDataMap& dataMap);
    ~BeeBusModbusHandler();

    // --- Main request processing ---
    bool processRequest(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID);

    // --- Individual modbus function requests ---
    bool processReadCoils(const BeeBusModbusRequest& request, BeeBusModbusRequest& response);
    bool processReadDiscreteInputs(const BeeBusModbusRequest& request, BeeBusModbusRequest& response);
    bool processReadHoldingRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response);
    bool processReadInputRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response);
    bool processWriteSingleCoil(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID);
    bool processWriteSingleRegister(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID);
    bool processWriteMultipleCoils(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID);
    bool processWriteMultipleRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID);

    // Responses are matched and applied by BeeBusOperations, reached through
    // BeeBusIO::handleModbusResponse.

private:
    BeeBusModbusDataMap& _dataMap;

    // --- Helper methods ---
    bool buildErrorResponse(const BeeBusModbusRequest& request, uint8_t exceptionCode, BeeBusModbusRequest& response);
};
