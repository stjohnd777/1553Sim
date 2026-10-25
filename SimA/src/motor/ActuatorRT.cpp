//
// Created by Daniel St. John St. John on 10/21/26.
//

#include "motor/ActuatorRT.hpp"

ActuatorRT::ActuatorRT(uint8_t address) : RT(address) {}

StatusWord ActuatorRT::handleReceive(uint8_t subAddress, const DataBlock1553& inData) {
    StatusWord status{address_};

    if (subAddress == 1 && inData.count == 1) { // Subaddress 1: Position Command (exact 1 word)
        int16_t rawCmd = static_cast<int16_t>(inData.words[0]);
        commandedDeflectionDeg = scaleFromWord(rawCmd, 45.0); // ±45 deg range
        actualDeflectionDeg = commandedDeflectionDeg;
        return status;
    }

    status.messageError = true;
    return status;
}

StatusWord ActuatorRT::handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) {
    StatusWord status{address_};

    if (subAddress == 1 && wordCount == 1) { // Subaddress 1: Position Telemetry (exact 1 word)
        outData.count = 1;
        outData.words[0] = static_cast<uint16_t>(scaleToWord(actualDeflectionDeg, 45.0));
        return status;
    }

    status.messageError = true;
    return status;
}
