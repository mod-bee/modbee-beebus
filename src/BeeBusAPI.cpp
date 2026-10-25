#include "BeeBusGlobal.h"

#if defined(BEEBUS_PLATFORM_ARDUINO)
#include "transport/BeeBusStreamTransport.h"
#endif
#include "BeeBusNoExcept.h"

BeeBusAPI::BeeBusAPI()
    : _protocol(nullptr), _ownedTransport(nullptr) {
    // Constructor - protocol will be created in begin()
}

BeeBusAPI::~BeeBusAPI() {
    end();
}

// --- Protocol management functions ---

bool BeeBusAPI::begin(BeeBusTransport* transport, uint8_t nodeID, const BeeBusConfig& config) {
    BEEBUS_GUARD();
    _startError = nullptr;
    if (_protocol) {
        return false; // Already initialized
    }

    if (nodeID < BEEBUS_NODE_ID_MIN || nodeID > BEEBUS_NODE_ID_MAX) {
        return false; // 0 means "none" on the wire, 255 is the join token
    }

    if (!transport || !transport->begin()) {
        return false; // Transport unusable
    }

    // Create protocol instance. Without exceptions a plain new that fails is
    // a hard fault on a board, so the failure is a refusal that says why.
    _protocol = new (std::nothrow) BeeBusProtocol();
    if (!_protocol) {
        _startError = "out of memory for the node";
        return false;
    }

    // Initialize protocol. A node whose queues could not be taken would refuse
    // every operation for ever, so it does not start.
    if (!_protocol->prepare(config) || !_protocol->begin(nodeID, transport, config)) {
        delete _protocol;
        _protocol = nullptr;
        _startError = "out of memory for the buffers and queues";
        return false;
    }
    return true;
}

#if defined(BEEBUS_PLATFORM_ARDUINO)
bool BeeBusAPI::begin(Stream* serial, uint8_t nodeID, const BeeBusConfig& config,
                      uint32_t baudRate) {
    BEEBUS_GUARD();
    _startError = nullptr;
    if (_protocol || !serial) {
        return false;
    }

    BeeBusTransport* transport = new (std::nothrow) BeeBusStreamTransport(serial, baudRate);
    if (!transport) {
        _startError = "out of memory for the link";
        return false;
    }
    if (!begin(transport, nodeID, config)) {
        delete transport;
        return false;
    }

    _ownedTransport = transport;
    return true;
}
#endif

bool BeeBusAPI::rebind(BeeBusTransport* transport, uint8_t nodeID,
                       const BeeBusConfig* config) {
    BEEBUS_GUARD();
    _startError = nullptr;
    if (!_protocol) return false;
    if (nodeID < BEEBUS_NODE_ID_MIN || nodeID > BEEBUS_NODE_ID_MAX) return false;
    if (!transport || !transport->begin()) return false;

    // Everything the new protocol needs is taken BEFORE the application moves
    // over to it, so running out of memory changes nothing: the node stays on
    // its old ring, as for any other refusal.
    BeeBusProtocol* fresh = new (std::nothrow) BeeBusProtocol();
    if (!fresh) {
        _startError = "out of memory for the node";
        return false;
    }
    if (!fresh->prepare(config != nullptr ? *config : _protocol->config())) {
        delete fresh;
        _startError = "out of memory for the buffers and queues";
        return false;
    }

    // Everything owed on the old ring is concluded now, while the handlers
    // still have something to report to.
    _protocol->getOperations().clearPendingOperations();
    _protocol->getOperations().dispatchLocalCompletions();

    fresh->adoptApplication(*_protocol);
    fresh->begin(nodeID, transport, config != nullptr ? *config : fresh->config());

    BeeBusTransport* oldOwned = _ownedTransport;
    _ownedTransport = nullptr;
    delete _protocol;
    _protocol = fresh;
    if (oldOwned && oldOwned != transport) {
        oldOwned->end();
        delete oldOwned;
    }
    return true;
}

#if defined(BEEBUS_PLATFORM_ARDUINO)
bool BeeBusAPI::rebind(Stream* serial, uint8_t nodeID, uint32_t baudRate,
                       const BeeBusConfig* config) {
    BEEBUS_GUARD();
    _startError = nullptr;
    if (!_protocol || !serial) return false;
    BeeBusTransport* transport = new (std::nothrow) BeeBusStreamTransport(serial, baudRate);
    if (!transport) {
        _startError = "out of memory for the link";
        return false;
    }
    if (!rebind(transport, nodeID, config)) {
        delete transport;
        return false;
    }
    _ownedTransport = transport;
    return true;
}
#endif

void BeeBusAPI::loop() {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->loop();
    }
}

void BeeBusAPI::end() {
    BEEBUS_GUARD();
    if (_protocol) {
        delete _protocol;
        _protocol = nullptr;
    }
    if (_ownedTransport) {
        _ownedTransport->end();
        delete _ownedTransport;
        _ownedTransport = nullptr;
    }
}

bool BeeBusAPI::isInitialized() {
    BEEBUS_GUARD();
    return _protocol != nullptr;
}

uint8_t BeeBusAPI::getNodeID() {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getNodeID();
    }
    return 0;
}

bool BeeBusAPI::isConnected() {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->isConnected();
    }
    return false;
}

void BeeBusAPI::connect() {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->nodeConnect();
    }
}

bool BeeBusAPI::isLeaving() {
    BEEBUS_GUARD();
    return _protocol != nullptr && _protocol->state() == BeeBusState::Leaving;
}

void BeeBusAPI::disconnect() {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->nodeDisconnect();
    }
}

bool BeeBusAPI::isMember(uint8_t nodeID) {
    BEEBUS_GUARD();
    return _protocol && _protocol->isRingParticipant() && _protocol->isMember(nodeID);
}

bool BeeBusAPI::hasDuplicateNodeID() {
    BEEBUS_GUARD();
    return _protocol && _protocol->hasDuplicateNodeID();
}

BeeBusResult BeeBusAPI::unreachable() const {
    BEEBUS_GUARD();
    return (_protocol && _protocol->hasDuplicateNodeID()) ? BeeBusResult::DuplicateNodeID
                                                          : BeeBusResult::UnknownNode;
}

void BeeBusAPI::readCompleted(void* ctx, uint8_t nodeID, uint16_t address, uint8_t,
                              BeeBusResult result) {
    BeeBusAPI* api = static_cast<BeeBusAPI*>(ctx);
    if (api->_readCompleteHandler) api->_readCompleteHandler(nodeID, address, result);
}

void BeeBusAPI::writeCompleted(void* ctx, uint8_t nodeID, uint16_t address, uint8_t,
                               BeeBusResult result) {
    BeeBusAPI* api = static_cast<BeeBusAPI*>(ctx);
    if (api->_writeCompleteHandler) api->_writeCompleteHandler(nodeID, address, result);
}

void BeeBusAPI::reportLocalRead(uint8_t nodeID, uint16_t address, BeeBusResult result) {
    BEEBUS_GUARD();
    if (!_protocol) return;
    // Recorded whether or not anyone registered a handler: a caller that
    // polls has nothing to register.
    _protocol->getOperations().noteCompleted(nodeID, address, result);
    if (!_readCompleteHandler) return;
    _protocol->getOperations().reportLocal(bb_completion(&BeeBusAPI::readCompleted, this),
                                           nodeID, address, 0, result);
}

void BeeBusAPI::reportLocalWrite(uint8_t nodeID, uint16_t address, BeeBusResult result) {
    BEEBUS_GUARD();
    if (!_protocol) return;
    // Recorded whether or not anyone registered a handler: a caller that
    // polls has nothing to register.
    _protocol->getOperations().noteCompleted(nodeID, address, result);
    if (!_writeCompleteHandler) return;
    _protocol->getOperations().reportLocal(bb_completion(&BeeBusAPI::writeCompleted, this),
                                           nodeID, address, 0, result);
}

// --- Data map management functions ---

BeeBusRequestState BeeBusAPI::requestState(uint8_t nodeID, uint16_t address,
                                           BeeBusResult& result) {
    BEEBUS_GUARD();
    if (!_protocol) {
        result = BeeBusResult::NotInitialised;
        return BeeBusRequestState::Idle;
    }
    return _protocol->getOperations().requestState(nodeID, address, result);
}

uint16_t BeeBusAPI::registerWrites(BeeBusRegisterType table, uint16_t address,
                                   uint16_t count) {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return _protocol->getDataMap().writeCount(table, address, count);
}

uint16_t BeeBusAPI::registerWritesSince(BeeBusRegisterType table, uint16_t address,
                                        uint16_t count, uint32_t& since,
                                        uint16_t* firstAddress) {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return _protocol->getDataMap().writtenSince(table, address, count, since, firstAddress);
}

uint16_t BeeBusAPI::failsafeCount(BeeBusRegisterType table, uint16_t address) {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return _protocol->getDataMap().failsafeCount(table, address);
}

bool BeeBusAPI::addRegister(BeeBusRegisterType table, uint16_t address, void* variable,
                            uint8_t type) {
    BEEBUS_GUARD();
    if (!_protocol || !variable) return false;
    if (!bb_tableAccepts(table, type)) return false;

    // A register table binds the value at its own width: a SINT is not two
    // bytes and a REAL is not one register, so neither may be bound as an INT.
    BeeBusModbusDataMap& map = _protocol->getDataMap();
    switch (table) {
        case MB_OUTPUT_COIL:      return map.addCoil(address, (bool*)variable);
        case MB_INPUT_STATUS:     return map.addIsts(address, (bool*)variable);
        case MB_HOLDING_REGISTER: return map.addHregValue(address, variable, type);
        case MB_INPUT_REGISTER:   return map.addIregValue(address, variable, type);
    }
    return false;
}

bool BeeBusAPI::removeRegister(BeeBusRegisterType table, uint16_t address) {
    BEEBUS_GUARD();
    if (!_protocol) return false;

    BeeBusModbusDataMap& map = _protocol->getDataMap();
    switch (table) {
        case MB_OUTPUT_COIL:      map.removeCoil(address); return true;
        case MB_INPUT_STATUS:     map.removeIsts(address); return true;
        case MB_HOLDING_REGISTER: map.removeHreg(address); return true;
        case MB_INPUT_REGISTER:   map.removeIreg(address); return true;
    }
    return false;
}

bool BeeBusAPI::addCoil(uint16_t address, bool* variable) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addCoil(address, variable);
}

bool BeeBusAPI::addHreg(uint16_t address, int16_t* variable) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addHreg(address, variable);
}

void BeeBusAPI::setCoilFailsafe(uint16_t address, BeeBusFailsafe action, bool value) {
    BEEBUS_GUARD();
    if (!_protocol) return;
    _protocol->getDataMap().setCoilFailsafe(address, action, value);
}

void BeeBusAPI::setHregFailsafe(uint16_t address, BeeBusFailsafe action, int16_t value) {
    BEEBUS_GUARD();
    if (!_protocol) return;
    _protocol->getDataMap().setHregFailsafe(address, action, value);
}

bool BeeBusAPI::addIsts(uint16_t address, bool* variable) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addIsts(address, variable);
}

bool BeeBusAPI::addIreg(uint16_t address, int16_t* variable) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addIreg(address, variable);
}

bool BeeBusAPI::setCoil(uint16_t address, bool value) {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getDataMap().setCoil(address, value);
    }
    return false;
}

bool BeeBusAPI::setHreg(uint16_t address, int16_t value) {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getDataMap().setHreg(address, value);
    }
    return false;
}

bool BeeBusAPI::setIsts(uint16_t address, bool value) {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getDataMap().setIsts(address, value);
    }
    return false;
}

bool BeeBusAPI::setIreg(uint16_t address, int16_t value) {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getDataMap().setIreg(address, value);
    }
    return false;
}

bool BeeBusAPI::getCoil(uint16_t address, bool& value) {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getDataMap().hasCoil(address)) {
        value = _protocol->getDataMap().getCoil(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::getHreg(uint16_t address, int16_t& value) {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getDataMap().hasHreg(address)) {
        value = _protocol->getDataMap().getHreg(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::getIsts(uint16_t address, bool& value) {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getDataMap().hasIsts(address)) {
        value = _protocol->getDataMap().getIsts(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::getIreg(uint16_t address, int16_t& value) {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getDataMap().hasIreg(address)) {
        value = _protocol->getDataMap().getIreg(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::removeCoil(uint16_t address) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().removeCoil(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::removeHreg(uint16_t address) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().removeHreg(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::removeIsts(uint16_t address) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().removeIsts(address);
        return true;
    }
    return false;
}

bool BeeBusAPI::removeIreg(uint16_t address) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().removeIreg(address);
        return true;
    }
    return false;
}

// --- Local string functions ---

bool BeeBusAPI::addString(uint16_t address, char* buffer, uint16_t capacity) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addString(address, buffer, capacity);
}

bool BeeBusAPI::addInputString(uint16_t address, char* buffer, uint16_t capacity) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().addInputString(address, buffer, capacity);
}

bool BeeBusAPI::removeString(uint16_t address) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    _protocol->getDataMap().removeString(address);
    return true;
}

bool BeeBusAPI::removeInputString(uint16_t address) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    _protocol->getDataMap().removeInputString(address);
    return true;
}

bool BeeBusAPI::setString(uint16_t address, const char* text) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().setString(address, text);
}

bool BeeBusAPI::setInputString(uint16_t address, const char* text) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().setInputString(address, text);
}

bool BeeBusAPI::getString(uint16_t address, char* out, uint16_t size) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().getString(address, out, size);
}

bool BeeBusAPI::getInputString(uint16_t address, char* out, uint16_t size) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().getInputString(address, out, size);
}

bool BeeBusAPI::setStringFailsafe(uint16_t address, BeeBusFailsafe action, const char* text) {
    BEEBUS_GUARD();
    if (!_protocol) return false;
    return _protocol->getDataMap().setStringFailsafe(address, action, text);
}

// --- Single value functions ---

BeeBusResult BeeBusAPI::readHreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc) {
    BEEBUS_GUARD();
    return readHreg_impl(nodeID, offset, &value, 1, fc);
}

BeeBusResult BeeBusAPI::readCoil(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc) {
    BEEBUS_GUARD();
    return readCoil_impl(nodeID, offset, &value, 1, fc);
}

BeeBusResult BeeBusAPI::readIreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc) {
    BEEBUS_GUARD();
    return readIreg_impl(nodeID, offset, &value, 1, fc);
}

BeeBusResult BeeBusAPI::readIsts(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc) {
    BEEBUS_GUARD();
    return readIsts_impl(nodeID, offset, &value, 1, fc);
}

BeeBusResult BeeBusAPI::writeHreg(uint8_t nodeID, uint16_t offset, int16_t value, uint8_t fc) {
    BEEBUS_GUARD();
    return writeHreg_impl(nodeID, offset, &value, 1, fc, BeeBusDelivery::Unconfirmed);
}

BeeBusResult BeeBusAPI::writeCoil(uint8_t nodeID, uint16_t offset, bool value, uint8_t fc) {
    BEEBUS_GUARD();
    return writeCoil_impl(nodeID, offset, &value, 1, fc, BeeBusDelivery::Unconfirmed);
}

BeeBusResult BeeBusAPI::writeHreg(uint8_t nodeID, uint16_t offset, int16_t value,
                                  BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    return writeHreg_impl(nodeID, offset, &value, 1, 0, delivery);
}

BeeBusResult BeeBusAPI::writeCoil(uint8_t nodeID, uint16_t offset, bool value,
                                  BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    return writeCoil_impl(nodeID, offset, &value, 1, 0, delivery);
}

void BeeBusAPI::onWriteComplete(
        std::function<void(uint8_t, uint16_t, BeeBusResult)> handler) {
    BEEBUS_GUARD();
    _writeCompleteHandler = handler;
}

// --- Manual sizing, for dynamic arrays ---

BeeBusResult BeeBusAPI::readHregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numregs, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return readHreg_impl(nodeID, offset, values, numregs, fc, stride);
}

BeeBusResult BeeBusAPI::readCoilManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numcoils, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return readCoil_impl(nodeID, offset, values, numcoils, fc, stride);
}

BeeBusResult BeeBusAPI::readIregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numiregs, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return readIreg_impl(nodeID, offset, values, numiregs, fc, stride);
}

BeeBusResult BeeBusAPI::readIstsManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numists, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return readIsts_impl(nodeID, offset, values, numists, fc, stride);
}

BeeBusResult BeeBusAPI::writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values, uint16_t numregs, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return writeHreg_impl(nodeID, offset, values, numregs, fc, BeeBusDelivery::Unconfirmed, stride);
}

BeeBusResult BeeBusAPI::writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values, uint16_t numcoils, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    return writeCoil_impl(nodeID, offset, values, numcoils, fc, BeeBusDelivery::Unconfirmed, stride);
}

BeeBusResult BeeBusAPI::writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values,
                                        uint16_t numregs, BeeBusDelivery delivery, uint16_t stride) {
    BEEBUS_GUARD();
    return writeHreg_impl(nodeID, offset, values, numregs, 0, delivery, stride);
}

BeeBusResult BeeBusAPI::writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values,
                                        uint16_t numcoils, BeeBusDelivery delivery, uint16_t stride) {
    BEEBUS_GUARD();
    return writeCoil_impl(nodeID, offset, values, numcoils, 0, delivery, stride);
}

// --- Implementation, called by the templates and the manual overloads ---

namespace {

// Whether a quantity and address range are ones Modbus can express. Checked
// here so a request that cannot possibly succeed is refused at the call, with a
// result the caller can act on, rather than being queued, transmitted, and
// answered with an exception a token rotation later.
//
// The limits are what fits in one PDU: 125 registers or 2000 coils per read,
// 123 registers or 1968 coils per write.
// Whether a caller-supplied function code override is one the library can
// encode. A section that cannot be built stops the build loop, so an
// unencodable code accepted here would jam the frame builder permanently.
bool isSendableFunction(uint8_t fc) {
    return fc == 0 || BeeBusModbusFrame::isValidFunctionCode(fc);
}

bool isSendableQuantity(uint16_t offset, uint16_t quantity, bool isRead, bool isBit) {
    if (quantity == 0) return false;

    const uint16_t limit = isBit ? (isRead ? 2000 : 1968)
                                 : (isRead ? 125 : 123);
    if (quantity > limit) return false;

    return BeeBusModbusFrame::isValidAddressRange(offset, quantity);
}

} // namespace

BeeBusResult BeeBusAPI::queueRead(uint8_t nodeID, uint16_t offset, uint16_t quantity,
                                  uint8_t functionCode, void* result,
                                  BeeBusResultKind kind, uint16_t resultCapacity,
                                  uint16_t stride) {
    BEEBUS_GUARD();
    // The same read already outstanding would be answered twice into the same
    // buffer, so a second copy is refused rather than queued behind it.
    for (const auto& pending : _protocol->getOperations().getPendingOps()) {
        if (pending.destNodeID == nodeID && pending.req.function == functionCode &&
            pending.req.startAddr == offset && pending.req.quantity == quantity) {
            return BeeBusResult::Duplicate;
        }
    }

    BeeBusPendingModbusOp op;
    op.destNodeID = nodeID;
    op.sourceNodeID = _protocol->getNodeID();
    op.req.function = functionCode;
    op.req.startAddr = offset;
    op.req.quantity = quantity;
    op.req.isResponse = false;
    op.timestamp = bb_millis();
    op.retryCount = 0;
    op.resultPtr = result;
    op.resultKind = kind;
    op.isArray = (quantity > 1);
    op.arraySize = quantity;
    op.resultCapacity = resultCapacity;
    op.resultStride = stride;

    if (_readCompleteHandler) {
        op.onComplete = bb_completion(&BeeBusAPI::readCompleted, this);
    }

    const BeeBusResult queued = _protocol->getOperations().addPendingOperation(op, *_protocol);
    if (queued == BeeBusResult::Queued) {
        BEEBUS_DEBUG_IO("ADDED: Read - Node:%d FC:%02X Addr:%d Qty:%d",
            nodeID, functionCode, offset, quantity);
    }
    return queued;
}

BeeBusResult BeeBusAPI::readHreg_impl(uint8_t nodeID, uint16_t offset, int16_t* values,
                                      uint16_t numregs, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numregs, true, false)) {
        return BeeBusResult::InvalidArgument;
    }
    if (!isMember(nodeID)) return unreachable();

    if (nodeID == _protocol->getNodeID()) {
        for (uint16_t i = 0; i < numregs; i++) {
            if (!_protocol->getDataMap().hasHreg(offset + i)) {
                return BeeBusResult::NotMapped;
            }
            *bb_elemAt<int16_t>(values, i, stride) = _protocol->getDataMap().getHreg(offset + i);
        }
        reportLocalRead(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    return queueRead(nodeID, offset, numregs,
                     (fc != 0) ? fc : MB_FC_READ_HOLDING_REGISTERS,
                     values, BeeBusResultKind::Native, 0, stride);
}

BeeBusResult BeeBusAPI::readCoil_impl(uint8_t nodeID, uint16_t offset, bool* values,
                                      uint16_t numcoils, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numcoils, true, true)) {
        return BeeBusResult::InvalidArgument;
    }
    if (!isMember(nodeID)) return unreachable();

    if (nodeID == _protocol->getNodeID()) {
        for (uint16_t i = 0; i < numcoils; i++) {
            if (!_protocol->getDataMap().hasCoil(offset + i)) {
                return BeeBusResult::NotMapped;
            }
            *bb_elemAt<bool>(values, i, stride) = _protocol->getDataMap().getCoil(offset + i);
        }
        reportLocalRead(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    return queueRead(nodeID, offset, numcoils,
                     (fc != 0) ? fc : MB_FC_READ_COILS,
                     values, BeeBusResultKind::Native, 0, stride);
}

BeeBusResult BeeBusAPI::readIreg_impl(uint8_t nodeID, uint16_t offset, int16_t* values,
                                      uint16_t numiregs, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numiregs, true, false)) {
        return BeeBusResult::InvalidArgument;
    }
    if (!isMember(nodeID)) return unreachable();

    if (nodeID == _protocol->getNodeID()) {
        for (uint16_t i = 0; i < numiregs; i++) {
            if (!_protocol->getDataMap().hasIreg(offset + i)) {
                return BeeBusResult::NotMapped;
            }
            *bb_elemAt<int16_t>(values, i, stride) = _protocol->getDataMap().getIreg(offset + i);
        }
        reportLocalRead(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    return queueRead(nodeID, offset, numiregs,
                     (fc != 0) ? fc : MB_FC_READ_INPUT_REGISTERS,
                     values, BeeBusResultKind::Native, 0, stride);
}

BeeBusResult BeeBusAPI::readIsts_impl(uint8_t nodeID, uint16_t offset, bool* values,
                                      uint16_t numists, uint8_t fc, uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numists, true, true)) {
        return BeeBusResult::InvalidArgument;
    }
    if (!isMember(nodeID)) return unreachable();

    if (nodeID == _protocol->getNodeID()) {
        for (uint16_t i = 0; i < numists; i++) {
            if (!_protocol->getDataMap().hasIsts(offset + i)) {
                return BeeBusResult::NotMapped;
            }
            *bb_elemAt<bool>(values, i, stride) = _protocol->getDataMap().getIsts(offset + i);
        }
        reportLocalRead(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    return queueRead(nodeID, offset, numists, (fc != 0) ? fc : MB_FC_READ_DISCRETE_INPUTS,
                     values, BeeBusResultKind::Native, 0, stride);
}

BeeBusResult BeeBusAPI::writeHreg_impl(uint8_t nodeID, uint16_t offset, const int16_t* values,
                                    uint16_t numregs, uint8_t fc, BeeBusDelivery delivery,
                                       uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numregs, false, false)) {
        return BeeBusResult::InvalidArgument;
    }

    // Check if target node exists
    if (!isMember(nodeID)) {
        return unreachable();
    }

    if (nodeID == _protocol->getNodeID()) {
        // Local write
        for (uint16_t i = 0; i < numregs; i++) {
            if (!_protocol->getDataMap().setHreg(offset + i,
                                                *bb_elemAt<int16_t>((void*)values, i, stride))) {
                return BeeBusResult::NotMapped;
            }
        }
        reportLocalWrite(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    // Determine function code based on quantity
    uint8_t functionCode;
    if (numregs == 1) {
        functionCode = (fc != 0) ? fc : MB_FC_WRITE_SINGLE_REGISTER;
    } else {
        functionCode = (fc != 0) ? fc : MB_FC_WRITE_MULTIPLE_REGISTERS;
    }

    BeeBusModbusRequest req;
    req.function = functionCode;
    req.startAddr = offset;
    req.quantity = numregs;
    req.isResponse = false;

    // Copy the payload now. Holding a pointer and reading it when the token
    // arrives is unsafe: the caller's storage may already be gone, and even a
    // live buffer may have changed since the operation was queued.
    if (numregs == 1) {
        req.data.resize(2);
        bb_packBE16(&req.data[0], (uint16_t)*bb_elemAt<int16_t>((void*)values, 0, stride));
    } else {
        const uint16_t byteCount = (uint16_t)(numregs * 2);
        req.data.resize(1 + byteCount);
        req.data[0] = (uint8_t)byteCount;
        for (uint16_t i = 0; i < numregs; i++) {
            bb_packBE16(&req.data[1 + (i * 2)],
                        (uint16_t)*bb_elemAt<int16_t>((void*)values, i, stride));
        }
    }

    BeeBusPendingModbusOp op;
    op.destNodeID = nodeID;
    op.sourceNodeID = _protocol->getNodeID();
    op.req = req;
    op.timestamp = bb_millis();
    op.retryCount = 0;

    op.resultPtr = nullptr;   // Writes carry their own copy
    op.isArray = (numregs > 1);
    op.arraySize = numregs;
    op.wantsAck = (delivery == BeeBusDelivery::Confirmed);

    // Attached whatever the delivery. An unconfirmed write completes Immediate
    // the moment it is transmitted and a confirmed one when it is acknowledged,
    // so every accepted write reports exactly once and a caller holding a slot
    // per write gets it back either way. Topic writes behave the same way.
    if (_writeCompleteHandler) {
        op.onComplete = bb_completion(&BeeBusAPI::writeCompleted, this);
    }

    return _protocol->getOperations().addPendingOperation(op, *_protocol);
}

BeeBusResult BeeBusAPI::writeCoil_impl(uint8_t nodeID, uint16_t offset, const bool* values,
                                    uint16_t numcoils, uint8_t fc, BeeBusDelivery delivery,
                                       uint16_t stride) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!values) return BeeBusResult::InvalidArgument;
    if (!isSendableFunction(fc)) return BeeBusResult::InvalidArgument;
    if (!isSendableQuantity(offset, numcoils, false, true)) {
        return BeeBusResult::InvalidArgument;
    }

    // Check if target node exists
    if (!isMember(nodeID)) {
        return unreachable();
    }

    if (nodeID == _protocol->getNodeID()) {
        // Local write
        for (uint16_t i = 0; i < numcoils; i++) {
            if (!_protocol->getDataMap().setCoil(offset + i,
                                                *bb_elemAt<bool>((void*)values, i, stride))) {
                return BeeBusResult::NotMapped;
            }
        }
        reportLocalWrite(nodeID, offset, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    // Determine function code based on quantity
    uint8_t functionCode;
    if (numcoils == 1) {
        functionCode = (fc != 0) ? fc : MB_FC_WRITE_SINGLE_COIL;
    } else {
        functionCode = (fc != 0) ? fc : MB_FC_WRITE_MULTIPLE_COILS;
    }

    BeeBusModbusRequest req;
    req.function = functionCode;
    req.startAddr = offset;
    req.quantity = numcoils;
    req.isResponse = false;

    // Copy the payload now, for the same reason as writeHreg_impl.
    if (numcoils == 1) {
        req.data.resize(2);
        req.data[0] = *bb_elemAt<bool>((void*)values, 0, stride) ? 0xFF : 0x00;
        req.data[1] = 0x00;
    } else {
        const uint16_t byteCount = (uint16_t)((numcoils + 7) / 8);
        req.data.assign(1 + byteCount, 0);
        req.data[0] = (uint8_t)byteCount;
        for (uint16_t i = 0; i < numcoils; i++) {
            if (*bb_elemAt<bool>((void*)values, i, stride)) {
                req.data[1 + (i / 8)] |= (uint8_t)(1 << (i % 8));
            }
        }
    }

    BeeBusPendingModbusOp op;
    op.destNodeID = nodeID;
    op.sourceNodeID = _protocol->getNodeID();
    op.req = req;
    op.timestamp = bb_millis();
    op.retryCount = 0;

    op.resultPtr = nullptr;   // Writes carry their own copy
    op.isArray = (numcoils > 1);
    op.arraySize = numcoils;
    op.wantsAck = (delivery == BeeBusDelivery::Confirmed);

    if (_writeCompleteHandler) {
        op.onComplete = bb_completion(&BeeBusAPI::writeCompleted, this);
    }

    return _protocol->getOperations().addPendingOperation(op, *_protocol);
}

// --- Remote string functions ---

namespace {

// One read carries 125 registers and one write 123, so a string is bounded by
// the same PDU limits as any other register operation. The last byte of the
// field is the terminator, which is why each is one short of the byte count.
const uint16_t BEEBUS_STRING_READ_REGS  = 125;
const uint16_t BEEBUS_STRING_WRITE_REGS = 123;
const uint16_t BEEBUS_STRING_WRITE_MAX  = (BEEBUS_STRING_WRITE_REGS * 2) - 1;

} // namespace

BeeBusResult BeeBusAPI::readString_impl(uint8_t nodeID, uint16_t address, char* buffer,
                                        uint16_t size, uint8_t functionCode) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!buffer || size < 2) return BeeBusResult::InvalidArgument;
    if (!isMember(nodeID)) return unreachable();

    // `size` is how much to read, not how much room there happens to be, so
    // asking for more than one operation carries is refused rather than quietly
    // answered in part. Silently reading less than was asked for would present
    // a truncated message as a whole one.
    const uint16_t regs = (uint16_t)(size / 2);
    if (regs > BEEBUS_STRING_READ_REGS ||
        !BeeBusModbusFrame::isValidAddressRange(address, regs)) {
        return BeeBusResult::InvalidArgument;
    }

    if (nodeID == _protocol->getNodeID()) {
        const bool local = (functionCode == MB_FC_READ_INPUT_REGISTERS)
                           ? _protocol->getDataMap().getInputString(address, buffer, size)
                           : _protocol->getDataMap().getString(address, buffer, size);
        if (!local) return BeeBusResult::NotMapped;
        reportLocalRead(nodeID, address, BeeBusResult::Immediate);
        return BeeBusResult::Immediate;
    }

    // The caller's whole buffer, not regs * 2: the response's byte count is
    // what gets copied, and bounding it by anything derived from the register
    // count is what lets an odd count run off the end.
    return queueRead(nodeID, address, regs, functionCode, buffer, BeeBusResultKind::Text,
                     size);
}

BeeBusResult BeeBusAPI::readString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size) {
    BEEBUS_GUARD();
    return readString_impl(nodeID, address, buffer, size, MB_FC_READ_HOLDING_REGISTERS);
}

BeeBusResult BeeBusAPI::readInputString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size) {
    BEEBUS_GUARD();
    return readString_impl(nodeID, address, buffer, size, MB_FC_READ_INPUT_REGISTERS);
}

BeeBusResult BeeBusAPI::writeString(uint8_t nodeID, uint16_t address, const char* text) {
    BEEBUS_GUARD();
    return writeString(nodeID, address, text, BeeBusDelivery::Unconfirmed);
}

BeeBusResult BeeBusAPI::writeString(uint8_t nodeID, uint16_t address, const char* text,
                                    BeeBusDelivery delivery) {
    BEEBUS_GUARD();
    if (!_protocol) return BeeBusResult::NotInitialised;
    if (!text) return BeeBusResult::InvalidArgument;

    const size_t length = strlen(text);
    if (length > BEEBUS_STRING_WRITE_MAX) {
        return BeeBusResult::InvalidArgument;
    }

    // The terminator travels with the text and a register holds two characters,
    // so an even-length message pays for one register it does not fill. Only
    // what the message needs goes out, whatever the field at the far end holds.
    const uint16_t regs = (uint16_t)((length + 2) / 2);

    int16_t packed[BEEBUS_STRING_WRITE_REGS];
    for (uint16_t i = 0; i < regs; i++) {
        const size_t high = (size_t)i * 2;
        const size_t low = high + 1;
        packed[i] = (int16_t)(((uint16_t)(high <= length ? (uint8_t)text[high] : 0) << 8) |
                              (low <= length ? (uint8_t)text[low] : 0));
    }

    return writeHreg_impl(nodeID, address, packed, regs, 0, delivery);
}

// --- Utility functions ---

bb_time_t BeeBusAPI::getRotationTime() {
    BEEBUS_GUARD();
    return _protocol ? _protocol->getRotationTime() : 0;
}

bb_time_t BeeBusAPI::getHopTime(uint8_t nodeID) {
    BEEBUS_GUARD();
    return _protocol ? _protocol->getHopTime(nodeID) : 0;
}

bb_time_t BeeBusAPI::getSolicitWindow() {
    BEEBUS_GUARD();
    return _protocol ? _protocol->solicitWindowMs() : 0;
}

bb_time_t BeeBusAPI::getSolicitWindowLimit() {
    BEEBUS_GUARD();
    // What this node will hold for somebody else's invitation. Zero where
    // there is no ring to protect, which is a node still on its own.
    return _protocol ? _protocol->solicitWindowBudget(false) : 0;
}

bb_time_t BeeBusAPI::getHopTimeout(uint8_t nodeID) {
    BEEBUS_GUARD();
    return _protocol ? _protocol->getHopTimeout(nodeID) : 0;
}

uint16_t BeeBusAPI::getByteTime() {
    BEEBUS_GUARD();
    return _protocol ? _protocol->byteTimeUs() : 0;
}

bb_time_t BeeBusAPI::getServiceInterval() {
    BEEBUS_GUARD();
    return _protocol ? _protocol->getServiceInterval() : 0;
}

uint16_t BeeBusAPI::getPendingOpCount() {
    BEEBUS_GUARD();
    if (_protocol) {
        return _protocol->getOperations().getPendingOpCount();
    }
    return 0;
}

void BeeBusAPI::clearPendingOps() {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getOperations().clearPendingOps();
    }
}

BeeBusIOStats BeeBusAPI::getLinkStatistics() {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getIO()) {
        return _protocol->getIO()->getStatistics();
    }
    return BeeBusIOStats();
}

void BeeBusAPI::resetLinkStatistics() {
    BEEBUS_GUARD();
    if (_protocol && _protocol->getIO()) {
        _protocol->getIO()->resetStatistics();
    }
}

uint8_t BeeBusAPI::getMemberCount() {
    BEEBUS_GUARD();
    if (!_protocol) return 0;
    return _protocol->isRingParticipant() ? _protocol->getMemberCount() : 0;
}

uint8_t BeeBusAPI::getMemberAt(uint8_t index) {
    BEEBUS_GUARD();
    if (!_protocol || !_protocol->isRingParticipant()) return 0;
    return _protocol->getMemberAt(index);
}

// --- Callback registration functions ---

void BeeBusAPI::onError(void (*errorHandler)(BeeBusError error, const char* message)) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->onError(errorHandler);
    }
}

void BeeBusAPI::onReadComplete(
        std::function<void(uint8_t, uint16_t, BeeBusResult)> handler) {
    BEEBUS_GUARD();
    _readCompleteHandler = handler;
}

void BeeBusAPI::onCoilChanged(uint16_t address, BeeBusCoilChangedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().onCoilChanged(address, handler);
    }
}

void BeeBusAPI::onHregChanged(uint16_t address, BeeBusHregChangedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().onHregChanged(address, handler);
    }
}

void BeeBusAPI::onStringChanged(uint16_t address, BeeBusStringChangedHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().onStringChanged(address, handler);
    }
}

void BeeBusAPI::onFailsafeApplied(BeeBusFailsafeHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->getDataMap().onFailsafeApplied(handler);
    }
}

void BeeBusAPI::onMemberJoined(BeeBusMemberHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->onMemberJoined(handler);
    }
}

void BeeBusAPI::onMemberLeft(BeeBusMemberHandler handler) {
    BEEBUS_GUARD();
    if (_protocol) {
        _protocol->onMemberLeft(handler);
    }
}

#ifndef DISABLE_BEEBUS_DEBUG
namespace {

// The debug instance is process-wide, so this handler is too. Messages carry
// the node they came from, stamped by BeeBusProtocol::loop().
void (*g_userDebugHandler)(const char* category, const char* message) = nullptr;

void debugBridge(BeeBusDebugLevel, BeeBusDebugCategory category, const char* message) {
    if (!g_userDebugHandler) return;

    const char* categoryStr = "DEBUG";
    switch (category) {
        case BEEBUS_DEBUG_PROTOCOL:   categoryStr = "PROTOCOL"; break;
        case BEEBUS_DEBUG_FRAMES:     categoryStr = "FRAMES"; break;
        case BEEBUS_DEBUG_MODBUS:     categoryStr = "MODBUS"; break;
        case BEEBUS_DEBUG_OPERATIONS: categoryStr = "OPERATIONS"; break;
        case BEEBUS_DEBUG_IO:         categoryStr = "IO"; break;
        default: break;
    }
    g_userDebugHandler(categoryStr, message);
}

void (*g_userFrameHandler)(const char*, const uint8_t*, uint16_t, const char*) = nullptr;

void frameBridge(BeeBusFrameDirection direction, const uint8_t* frame,
                 uint16_t length, const char* decoded) {
    if (g_userFrameHandler) {
        g_userFrameHandler(direction == BEEBUS_FRAME_TX ? "TX" : "RX", frame, length, decoded);
    }
}

void (*g_userModbusHandler)(const char*, uint8_t, const char*) = nullptr;

void modbusBridge(BeeBusModbusType type, uint8_t nodeID, const BeeBusModbusRequest&,
                  const char* decoded) {
    if (g_userModbusHandler) {
        g_userModbusHandler(type == BEEBUS_MODBUS_REQUEST ? "REQUEST" : "RESPONSE",
                            nodeID, decoded);
    }
}

} // namespace
#endif

void BeeBusAPI::onDebug(void (*debugHandler)(const char* category, const char* message)) {
    BEEBUS_GUARD();
#ifdef DISABLE_BEEBUS_DEBUG
    (void)debugHandler;
#else
    g_userDebugHandler = debugHandler;
    g_beebusDebug.onDebug(debugHandler ? debugBridge : nullptr);
#endif
}

void BeeBusAPI::onFrameDebug(void (*handler)(const char* direction, const uint8_t* frame,
                                             uint16_t length, const char* decoded)) {
    BEEBUS_GUARD();
#ifdef DISABLE_BEEBUS_DEBUG
    (void)handler;
#else
    g_userFrameHandler = handler;
    g_beebusDebug.onFrameDebug(handler ? frameBridge : nullptr);
#endif
}

void BeeBusAPI::onModbusDebug(void (*handler)(const char* type, uint8_t nodeID,
                                              const char* decoded)) {
    BEEBUS_GUARD();
#ifdef DISABLE_BEEBUS_DEBUG
    (void)handler;
#else
    g_userModbusHandler = handler;
    g_beebusDebug.onModbusDebug(handler ? modbusBridge : nullptr);
#endif
}
