//
// Created by Daniel St. John St. John on 10/25/26.
//

#include "sensor/PressureSensorRT.hpp"
#include <algorithm>

PressureSensorRT::PressureSensorRT(uint8_t address) : RT(address) {}

void PressureSensorRT::stepSimulation(double dtSeconds) {
    pressureBar += pressureRate * dtSeconds;
}

StatusWord PressureSensorRT::handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) {
    StatusWord status{address_};

    if (subAddress == 1) { // Subaddress 1: Pressure Telemetry (1 word)
        if (wordCount != 1) {
            status.messageError = true;
            return status;
        }
        outData.count = 1;
        outData.words[0] = static_cast<uint16_t>(scaleToWord(pressureBar, maxScaleBar));
        return status;
    }

    status.messageError = true;
    return status;
}

StatusWord PressureSensorRT::handleReceive(uint8_t subAddress, const DataBlock1553& inData) {
    StatusWord status{address_};

    if (subAddress == 1 && inData.count == 1) { // Subaddress 1: Calibration / Simulated Pressure Set
        int16_t rawVal = static_cast<int16_t>(inData.words[0]);
        pressureBar = scaleFromWord(rawVal, maxScaleBar);
        return status;
    }

    status.messageError = true;
    return status;
}
