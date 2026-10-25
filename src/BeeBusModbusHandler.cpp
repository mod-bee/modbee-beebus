#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

BeeBusModbusHandler::BeeBusModbusHandler(BeeBusModbusDataMap& dataMap) : _dataMap(dataMap) {}

BeeBusModbusHandler::~BeeBusModbusHandler() = default;

// --- Main request processing ---

bool BeeBusModbusHandler::processRequest(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID) {
    // An unsupported function code and a bad quantity are different faults and
    // must be reported as such, or the requester is sent looking at its data
    // when the problem is the function it asked for.
    if (!BeeBusModbusFrame::isValidFunctionCode(request.function) || (request.function & 0x80)) {
        BEEBUS_DEBUG_MODBUS_INFO("UNSUPPORTED FUNCTION: FC:%02X", request.function);
        return buildErrorResponse(request, MB_EX_ILLEGAL_FUNCTION, response);
    }

    if (!BeeBusModbusFrame::validateRequest(request)) {
        BEEBUS_DEBUG_MODBUS_INFO("REQUEST VALIDATION FAILED: FC:%02X", request.function);
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // The addresses past the end of the space are not addresses this node has,
    // whatever the wrap-around would land on.
    if (!BeeBusModbusFrame::isValidAddressRange(request.startAddr, request.quantity)) {
        BEEBUS_DEBUG_MODBUS_INFO("ADDRESS RANGE WRAPS: Addr:%d Qty:%d",
            request.startAddr, request.quantity);
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
    }

    // Initialize response
    response.function = request.function;
    response.startAddr = request.startAddr;
    response.quantity = request.quantity;
    response.isResponse = true;
    response.data.clear();

    // Process based on function code
    bool result = false;
    switch (request.function) {
        case MB_FC_READ_COILS:
            result = processReadCoils(request, response);
            break;

        case MB_FC_READ_DISCRETE_INPUTS:
            result = processReadDiscreteInputs(request, response);
            break;

        case MB_FC_READ_HOLDING_REGISTERS:
            result = processReadHoldingRegisters(request, response);
            break;

        case MB_FC_READ_INPUT_REGISTERS:
            result = processReadInputRegisters(request, response);
            break;

        case MB_FC_WRITE_SINGLE_COIL:
            result = processWriteSingleCoil(request, response, sourceNodeID);
            break;

        case MB_FC_WRITE_SINGLE_REGISTER:
            result = processWriteSingleRegister(request, response, sourceNodeID);
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
            result = processWriteMultipleCoils(request, response, sourceNodeID);
            break;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            result = processWriteMultipleRegisters(request, response, sourceNodeID);
            break;

        default:
            BEEBUS_DEBUG_MODBUS_INFO("UNSUPPORTED FUNCTION: FC:%02X", request.function);
            result = buildErrorResponse(request, MB_EX_ILLEGAL_FUNCTION, response);
            break;
    }

    // Whatever the request changed is reported now that all of it has been
    // applied, so a handler never sees a half-written block.
    _dataMap.dispatchChangeEvents();

    return result;
}

// --- Read function implementations ---

bool BeeBusModbusHandler::processReadCoils(const BeeBusModbusRequest& request, BeeBusModbusRequest& response) {
    // Check if all requested coils exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasCoil(request.startAddr + i)) {
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // Calculate byte count
    uint8_t byteCount = BeeBusModbusFrame::getBitPackedBytes(request.quantity);

    // Build response data
    response.data.resize(1 + byteCount);
    response.data[0] = byteCount;

    // Packed straight out of the data map, allocating nothing: a busy node
    // takes this path on every token.
    for (uint16_t i = 0; i < byteCount; i++) {
        response.data[1 + i] = 0;
    }
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (_dataMap.getCoil(request.startAddr + i)) {
            response.data[1 + (i / 8)] |= (uint8_t)(1 << (i % 8));
        }
    }

    return true;
}

bool BeeBusModbusHandler::processReadDiscreteInputs(const BeeBusModbusRequest& request, BeeBusModbusRequest& response) {
    BEEBUS_DEBUG_MODBUS_INFO("READ DISCRETE INPUTS: Addr:%d Qty:%d", request.startAddr, request.quantity);

    // Check if all requested inputs exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasIsts(request.startAddr + i)) {
            BEEBUS_DEBUG_MODBUS_INFO("DISCRETE INPUT NOT FOUND: Addr:%d", request.startAddr + i);
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // Set response start address from request
    response.startAddr = request.startAddr;
    response.quantity = request.quantity;

    // Calculate byte count
    uint8_t byteCount = BeeBusModbusFrame::getBitPackedBytes(request.quantity);

    // Response data: byte count, then the packed bits. No address.
    response.data.resize(1 + byteCount);
    response.data[0] = byteCount;

    // Packed straight out of the data map, allocating nothing.
    for (uint16_t i = 0; i < byteCount; i++) {
        response.data[1 + i] = 0;
    }
    for (uint16_t i = 0; i < request.quantity; i++) {
        const bool value = _dataMap.getIsts(request.startAddr + i);
        BEEBUS_DEBUG_MODBUS_INFO("ISTS[%d] = %d", request.startAddr + i, value);
        if (value) {
            response.data[1 + (i / 8)] |= (uint8_t)(1 << (i % 8));
        }
    }

    BEEBUS_DEBUG_MODBUS_INFO("READ DISCRETE INPUTS: Built response addr:%d with %u data bytes (byteCount:%d)",
        response.startAddr, (unsigned)response.data.size(), byteCount);
    return true;
}

bool BeeBusModbusHandler::processReadHoldingRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response) {
    BEEBUS_DEBUG_MODBUS_INFO("READ HOLDING REGS: Addr:%d Qty:%d", request.startAddr, request.quantity);

    // Check if all requested registers exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasHreg(request.startAddr + i)) {
            BEEBUS_DEBUG_MODBUS_INFO("MISSING REGISTER: Addr:%d", request.startAddr + i);
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // A byte count must fit one byte, which caps the read at 127 registers.
    // BeeBusModbusFrame::validateRequest already caps it at 125, but the bound belongs
    // here too rather than being inherited from a caller.
    if (request.quantity > 127) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }
    const uint8_t byteCount = (uint8_t)(request.quantity * 2);

    response.data.resize(1 + byteCount);
    response.data[0] = byteCount;

    // Read register values
    uint16_t dataIndex = 1;
    for (uint16_t i = 0; i < request.quantity; i++) {
        int16_t regValue = _dataMap.getHreg(request.startAddr + i);
        bb_packBE16(&response.data[dataIndex], (uint16_t)regValue);
        dataIndex += 2;
        BEEBUS_DEBUG_MODBUS_INFO("READ REG[%d] = %d", request.startAddr + i, regValue);
    }

    return true;
}

bool BeeBusModbusHandler::processReadInputRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response) {
    BEEBUS_DEBUG_MODBUS_INFO("READ INPUT REGS: Addr:%d Qty:%d", request.startAddr, request.quantity);

    // Check if all requested registers exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasIreg(request.startAddr + i)) {
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // A byte count must fit one byte, which caps the read at 127 registers.
    // BeeBusModbusFrame::validateRequest already caps it at 125, but the bound belongs
    // here too rather than being inherited from a caller.
    if (request.quantity > 127) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }
    const uint8_t byteCount = (uint8_t)(request.quantity * 2);

    response.data.resize(1 + byteCount);
    response.data[0] = byteCount;

    // Read register values
    uint16_t dataIndex = 1;
    for (uint16_t i = 0; i < request.quantity; i++) {
        int16_t value = _dataMap.getIreg(request.startAddr + i);
        bb_packBE16(&response.data[dataIndex], (uint16_t)value);
        dataIndex += 2;
    }

    return true;
}

// --- Write function implementations ---

bool BeeBusModbusHandler::processWriteSingleCoil(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID) {
    if (request.data.size() < 2) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // Check if coil exists
    if (!_dataMap.hasCoil(request.startAddr)) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
    }

    // 0xFF00 is on and 0x0000 off; the specification refuses anything else.
    const uint16_t raw = bb_unpackBE16(&request.data[0]);
    if (raw != 0xFF00 && raw != 0x0000) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }
    const bool coilValue = (raw == 0xFF00);

    // Write coil
    if (!_dataMap.setCoil(request.startAddr, coilValue, sourceNodeID)) {
        return buildErrorResponse(request, MB_EX_SLAVE_DEVICE_FAILURE, response);
    }

    // A write is acknowledged, not echoed: the response carries no data.
    response.data.clear();

    return true;
}

bool BeeBusModbusHandler::processWriteSingleRegister(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID) {
    if (request.data.size() < 2) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // Check if register exists
    if (!_dataMap.hasHreg(request.startAddr)) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
    }

    // Extract value
    int16_t value = (int16_t)bb_unpackBE16(&request.data[0]);

    // Write register
    if (!_dataMap.setHreg(request.startAddr, value, sourceNodeID)) {
        return buildErrorResponse(request, MB_EX_SLAVE_DEVICE_FAILURE, response);
    }

    // A write is acknowledged, not echoed: the response carries no data.
    response.data.clear();

    return true;
}

bool BeeBusModbusHandler::processWriteMultipleCoils(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID) {
    if (request.data.size() < 1) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    uint8_t byteCount = request.data[0];
    if (request.data.size() < (size_t)(1 + byteCount)) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // The byte count and the quantity are two statements about the same
    // payload. When they disagree the request is malformed, and unpacking as
    // far as the quantity claims would read past what the sender supplied.
    if (byteCount != BeeBusModbusFrame::getBitPackedBytes(request.quantity)) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // Check if all coils exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasCoil(request.startAddr + i)) {
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // Read the block out first, so a failure partway can put back what was
    // there. A block write is one command: applying half of it and reporting
    // the whole thing refused leaves the plant running a combination nobody
    // asked for, while the requester believes nothing changed.
    // Bit-packed, like the request: the byte count bounds the quantity, so the
    // old values always fit a PDU's worth of bytes.
    uint8_t previous[BEEBUS_MAX_PDU_BYTES];
    memset(previous, 0, sizeof(previous));
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (_dataMap.getCoil(request.startAddr + i)) {
            previous[i / 8] = (uint8_t)(previous[i / 8] | (1 << (i % 8)));
        }
    }

    // Unpack and write coils, straight from the request.
    for (uint16_t i = 0; i < request.quantity; i++) {
        const bool value = (request.data[(uint16_t)(1 + (i / 8))] & (1 << (i % 8))) != 0;
        if (_dataMap.setCoil(request.startAddr + i, value, sourceNodeID)) {
            continue;
        }

        // Roll back everything already applied, so the refusal is truthful.
        for (uint16_t back = 0; back < i; back++) {
            _dataMap.setCoil(request.startAddr + back,
                             (previous[back / 8] & (1 << (back % 8))) != 0, sourceNodeID);
        }
        return buildErrorResponse(request, MB_EX_SLAVE_DEVICE_FAILURE, response);
    }

    // A write is acknowledged, not echoed: the response carries no data.
    response.data.clear();

    return true;
}

bool BeeBusModbusHandler::processWriteMultipleRegisters(const BeeBusModbusRequest& request, BeeBusModbusRequest& response, uint8_t sourceNodeID) {
    if (request.data.size() < 1) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    uint8_t byteCount = request.data[0];
    if (request.data.size() < (size_t)(1 + byteCount) ||
        byteCount != (uint16_t)(request.quantity * 2)) {
        return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_VALUE, response);
    }

    // Check if all registers exist
    for (uint16_t i = 0; i < request.quantity; i++) {
        if (!_dataMap.hasHreg(request.startAddr + i)) {
            return buildErrorResponse(request, MB_EX_ILLEGAL_DATA_ADDRESS, response);
        }
    }

    // Read the block out first, for the same reason as the coil path: a
    // half-applied setpoint block is worse than either outcome, and the
    // requester cannot tell it happened.
    // The byte count bounds the quantity, so a PDU's worth of words holds it.
    int16_t previous[BEEBUS_MAX_PDU_BYTES / 2];
    for (uint16_t i = 0; i < request.quantity; i++) {
        previous[i] = _dataMap.getHreg(request.startAddr + i);
    }

    // Write registers
    uint16_t dataIndex = 1;
    for (uint16_t i = 0; i < request.quantity; i++) {
        int16_t value = (int16_t)bb_unpackBE16(&request.data[dataIndex]);
        dataIndex += 2;

        if (!_dataMap.setHreg(request.startAddr + i, value, sourceNodeID)) {
            for (uint16_t back = 0; back < i; back++) {
                _dataMap.setHreg(request.startAddr + back, previous[back], sourceNodeID);
            }
            return buildErrorResponse(request, MB_EX_SLAVE_DEVICE_FAILURE, response);
        }
    }

    // A write is acknowledged, not echoed: the response carries no data.
    response.data.clear();

    return true;
}

// --- Helper methods ---

bool BeeBusModbusHandler::buildErrorResponse(const BeeBusModbusRequest& request, uint8_t exceptionCode, BeeBusModbusRequest& response) {
    response.function = BeeBusModbusFrame::makeErrorResponse(request.function);
    response.isResponse = true;
    response.startAddr = request.startAddr;
    response.quantity = 0;
    response.data.clear();
    response.data.push_back(exceptionCode);

    return true;
}
