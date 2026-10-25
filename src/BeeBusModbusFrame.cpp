#include "BeeBusGlobal.h"
#include "BeeBusNoExcept.h"

// --- Modbus request building ---
uint16_t BeeBusModbusFrame::buildModbusRequest(uint8_t* buffer, uint16_t capacity,
                                         const BeeBusPendingModbusOp* op) {
    if (!buffer || !op) {
        return 0;
    }

    const BeeBusModbusRequest& request = op->req;

    // Every request begins with the function code and the address. A write
    // carries its payload in request.data, copied there when the operation was
    // queued, so nothing here reads through a caller-owned pointer.
    uint16_t needed = 3;
    switch (request.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            needed += 2;
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            needed = (uint16_t)(needed + 2 + request.data.size());
            break;

        default:
            return 0;   // Unknown function code
    }

    // The caller decides where the PDU goes and how much room is left. Without
    // the capacity this function cannot tell, and a large batched write runs
    // straight off the end of the frame buffer.
    if (needed > capacity) {
        return 0;
    }

    uint16_t pos = 0;
    buffer[pos++] = request.function;
    bb_packBE16(&buffer[pos], request.startAddr);
    pos += 2;

    switch (request.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            bb_packBE16(&buffer[pos], request.quantity);
            pos += 2;
            break;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            // Two payload bytes, written when the operation was queued. A
            // request that reaches here with fewer has been built wrong, and
            // sending zeroes would fabricate a value nobody asked for.
            if (request.data.size() < 2) {
                return 0;
            }
            buffer[pos++] = request.data[0];
            buffer[pos++] = request.data[1];
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            bb_packBE16(&buffer[pos], request.quantity);
            pos += 2;
            // data[0] is the byte count, followed by the packed payload.
            for (size_t i = 0; i < request.data.size(); i++) {
                buffer[pos++] = request.data[i];
            }
            break;

        default:
            return 0;
    }

    return pos;
}

// --- Modbus response building ---
uint16_t BeeBusModbusFrame::buildModbusResponse(uint8_t* buffer, uint16_t capacity,
                                          const BeeBusModbusRequest& response) {
    if (!buffer) {
        return 0;
    }

    // Size the PDU before writing a byte of it, for the same reason as
    // buildModbusRequest: this function is handed a pointer into a frame that
    // is already partly full, and only the caller knows what is left.
    uint16_t needed = 0;
    switch (response.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            if (response.data.empty()) return 0;
            // FC, address, byte count, payload.
            needed = (uint16_t)(4 + response.data[0]);
            if (response.data.size() < (size_t)(1 + response.data[0])) return 0;
            break;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            // A write is not echoed back. When the sender asked to be told the
            // outcome it gets an MB_FC_ACK section instead, which carries the
            // operation ID rather than repeating the payload.
            return 0;

        case MB_FC_ACK:
            // One or more entries, each an operation ID and a status. The
            // receiver derives the count from the section length, so no count
            // byte is carried.
            if (response.data.empty()) return 0;
            if (response.data.size() % BEEBUS_ACK_ENTRY_BYTES != 0) return 0;
            needed = (uint16_t)(1 + response.data.size());
            break;

        default:
            needed = 4;   // Exception: FC, address, exception code
            break;
    }

    if (needed > capacity) {
        return 0;
    }

    uint16_t pos = 0;
    buffer[pos++] = response.function;

    switch (response.function) {
        case MB_FC_ACK: {
            // [FC]([opIdHi][opIdLo][status])... Each status is MB_ACK_OK or the
            // exception code that refused the write.
            for (size_t i = 0; i < response.data.size(); i++) {
                buffer[pos++] = response.data[i];
            }
            break;
        }

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS: {
            // The address is carried so the requester can match the response to
            // one of several requests it may have outstanding to this node.
            bb_packBE16(&buffer[pos], response.startAddr);
            pos += 2;

            const uint8_t byteCount = response.data[0];
            buffer[pos++] = byteCount;
            for (uint8_t i = 1; i <= byteCount; i++) {
                buffer[pos++] = response.data[i];
            }
            break;
        }

        default: {
            // Exception response: FC | 0x80, the address of the request that
            // failed, then one exception code.
            //
            // The function code was already written above, so writing it again
            // here would displace the exception code and the requester would
            // read the function code as the reason for the failure.
            //
            // The address is carried for the same reason read responses carry
            // one: BeeBus pipelines several requests to a node in one token, so
            // an exception with no address cannot be attributed to the request
            // that caused it.
            buffer[0] = makeErrorResponse(response.function);
            bb_packBE16(&buffer[pos], response.startAddr);
            pos += 2;
            buffer[pos++] = response.data.size() > 0 ? response.data[0]
                                                     : MB_EX_SLAVE_DEVICE_FAILURE;
            break;
        }
    }

    return pos;
}

// --- Modbus request parsing ---
bool BeeBusModbusFrame::parseModbusRequest(const uint8_t* buffer, uint16_t length, BeeBusModbusRequest& request) {
    if (!buffer || length < 2) {
        return false;
    }

    uint16_t pos = 0;

    // Function code
    request.function = buffer[pos++];
    request.isResponse = false;
    request.data.clear();

    switch (request.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            if (length < 5) return false;

            // Starting address
            request.startAddr = bb_unpackBE16(&buffer[pos]);
            pos += 2;

            // Quantity
            request.quantity = bb_unpackBE16(&buffer[pos]);
            pos += 2;
            break;
        }
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER: {
            if (length < 5) return false;

            // Starting address
            request.startAddr = bb_unpackBE16(&buffer[pos]);
            pos += 2;

            // Value
            request.quantity = 1;
            request.data.resize(2);
            request.data[0] = buffer[pos++];
            request.data[1] = buffer[pos++];
            break;
        }
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            if (length < 6) return false;

            // Starting address
            request.startAddr = bb_unpackBE16(&buffer[pos]);
            pos += 2;

            // Quantity
            request.quantity = bb_unpackBE16(&buffer[pos]);
            pos += 2;

            // Byte count
            if (pos >= length) return false;
            uint8_t byteCount = buffer[pos++];

            // Data
            if (pos + byteCount > length) return false;
            request.data.resize(byteCount + 1);
            request.data[0] = byteCount;
            for (uint8_t i = 0; i < byteCount; i++) {
                request.data[i + 1] = buffer[pos++];
            }
            break;
        }
        default:
            return false;
    }

    return true;
}

// --- Modbus response parsing ---
bool BeeBusModbusFrame::parseModbusResponse(const uint8_t* buffer, uint16_t length, BeeBusModbusRequest& response) {
    if (!buffer || length < 2) {
        return false;
    }

    uint16_t pos = 0;

    // Function code
    response.function = buffer[pos++];
    response.isResponse = true;
    response.data.clear();

    // Handle error responses: FC | 0x80, address, exception code.
    if (response.function & 0x80) {
        if (length < 4) return false;
        response.startAddr = bb_unpackBE16(&buffer[pos]);
        pos += 2;
        response.quantity = 0;
        response.data.push_back(buffer[pos++]);
        return true;
    }

    switch (response.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            // Parse custom response format with address
            if (length < 5) return false; // FC + Addr(2) + ByteCount + Data(min 1)

            // Extract address (custom BeeBus format)
            response.startAddr = bb_unpackBE16(&buffer[pos]);
            pos += 2;

            // Byte count
            uint8_t byteCount = buffer[pos++];

            if (length < (4 + byteCount)) return false; // FC + Addr(2) + ByteCount + Data

            // Store response data properly
            response.data.resize(1 + byteCount);
            response.data[0] = byteCount;

            for (uint8_t i = 0; i < byteCount; i++) {
                response.data[1 + i] = buffer[pos++];
            }

            // Calculate quantity from byte count
            if (response.function == MB_FC_READ_HOLDING_REGISTERS ||
                response.function == MB_FC_READ_INPUT_REGISTERS) {
                response.quantity = byteCount / 2; // Register responses: 2 bytes per register
            } else {
                response.quantity = byteCount * 8; // Bit responses: an upper bound, padded to whole bytes
            }
            break;
        }

        // No write response parsing - Writes don't generate responses in token ring

        default:
            return false;
    }

    return true;
}

// --- Bit packing utilities ---
void BeeBusModbusFrame::packBits(const bool* bits, uint8_t* buffer, uint16_t numBits) {
    if (!bits || !buffer || numBits == 0) {
        return;
    }

    uint16_t numBytes = (numBits + 7) / 8;

    // Clear buffer
    for (uint16_t i = 0; i < numBytes; i++) {
        buffer[i] = 0;
    }

    // Pack bits
    for (uint16_t i = 0; i < numBits; i++) {
        if (bits[i]) {
            uint16_t byteIndex = i / 8;
            uint8_t bitIndex = i % 8;
            buffer[byteIndex] |= (1 << bitIndex);
        }
    }
}

void BeeBusModbusFrame::unpackBits(const uint8_t* buffer, bool* bits, uint16_t numBits) {
    if (!buffer || !bits || numBits == 0) {
        return;
    }

    for (uint16_t i = 0; i < numBits; i++) {
        uint16_t byteIndex = i / 8;
        uint8_t bitIndex = i % 8;
        bits[i] = (buffer[byteIndex] & (1 << bitIndex)) != 0;
    }
}

uint8_t BeeBusModbusFrame::getBitPackedBytes(uint16_t numBits) {
    return (numBits + 7) / 8;
}

// --- Function code utilities ---
bool BeeBusModbusFrame::isValidFunctionCode(uint8_t functionCode) {
    uint8_t fc = functionCode & 0x7F; // Remove error bit

    switch (fc) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return true;
        default:
            return false;
    }
}

bool BeeBusModbusFrame::isReadFunction(uint8_t functionCode) {
    switch (functionCode) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            return true;
        default:
            return false;
    }
}

bool BeeBusModbusFrame::isErrorResponse(uint8_t functionCode) {
    return (functionCode & 0x80) != 0;
}

uint8_t BeeBusModbusFrame::makeErrorResponse(uint8_t functionCode) {
    return functionCode | 0x80;
}

// --- Validation ---
bool BeeBusModbusFrame::isValidAddressRange(uint16_t startAddr, uint16_t quantity) {
    // A block is walked with 16 bit arithmetic, so one that runs past 65535
    // reappears at address 0 rather than ending. The addresses it would then
    // reach are real, mapped and completely unrelated, so the caller would be
    // handed someone else's registers with no way to tell.
    return (uint32_t)startAddr + (uint32_t)quantity <= 0x10000u;
}

bool BeeBusModbusFrame::validateRequest(const BeeBusModbusRequest& request) {
    switch (request.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
            return request.quantity > 0 && request.quantity <= 2000;

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            return request.quantity > 0 && request.quantity <= 125;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            return request.data.size() >= 2;

        case MB_FC_WRITE_MULTIPLE_COILS:
            return request.quantity > 0 && request.quantity <= 1968 && request.data.size() > 0;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return request.quantity > 0 && request.quantity <= 123 && request.data.size() > 0;

        default:
            return false;
    }
}
