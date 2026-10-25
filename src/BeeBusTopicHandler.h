#pragma once
#include "BeeBusGlobal.h"

// Processes one inbound topic PDU addressed to this node, as BeeBusModbusHandler does
// one Modbus request. Holds no state: constructed for one section, discarded.
class BeeBusTopicHandler {
public:
    explicit BeeBusTopicHandler(BeeBusProtocol& protocol) : _protocol(protocol) {}

    // Returns BEEBUS_TOP_OK or the reason the PDU was refused. A refusal has
    // already been answered with an ERROR or a SUBACK; the return value is for
    // the caller's acknowledgement, which only confirms success.
    // `broadcast` says the section was addressed to the ring rather than to
    // this node. It reaches the value path because a broadcast carries no
    // sequence: see handleValue.
    // `parsed` is the caller's: a decoded PDU holds a whole page of entries and
    // a text, too much for every call to put on the stack.
    uint8_t process(const uint8_t* pdu, uint16_t length, uint8_t srcNodeID,
                    bool broadcast, BeeBusTopicPdu& parsed);

private:
    BeeBusProtocol& _protocol;

    uint8_t handleValue(const BeeBusTopicPdu& pdu, uint8_t srcNodeID, bool broadcast);
    uint8_t handleWrite(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleRead(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleSubscribe(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleSuback(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleUnsubscribe(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleListRequest(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleListReply(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleRegister(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleError(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleStreamAck(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleResult(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);
    uint8_t handleSession(const BeeBusTopicPdu& pdu, uint8_t srcNodeID);

    // Answers a refused request with the reason, rather than letting the far
    // end time out.
    void refuse(uint8_t destNodeID, uint16_t topicID, uint8_t reason);
};
