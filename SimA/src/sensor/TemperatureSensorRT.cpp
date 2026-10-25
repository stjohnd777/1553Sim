//
// Created by Daniel St. John St. John on 10/25/26.
//

#include "sensor/TemperatureSensorRT.hpp"
#include <algorithm>

TemperatureSensorRT::TemperatureSensorRT(uint8_t address) : RT(address) {}

void TemperatureSensorRT::stepSimulation(double dtSeconds) {
    temperatureCelsius += tempRate * dtSeconds;
}

StatusWord TemperatureSensorRT::handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) {
    StatusWord status{address_};

    if (subAddress == 1) { // Subaddress 1: Temperature Telemetry (1 word)
        if (wordCount != 1) {
            status.messageError = true;
            return status;
        }
        outData.count = 1;
        outData.words[0] = static_cast<uint16_t>(scaleToWord(temperatureCelsius, maxScaleCelsius));
        return status;
    }

    status.messageError = true;
    return status;
}

StatusWord TemperatureSensorRT::handleReceive(uint8_t subAddress, const DataBlock1553& inData) {
    StatusWord status{address_};

    if (subAddress == 1 && inData.count == 1) { // Subaddress 1: Calibration / Simulated Temperature Set
        int16_t rawVal = static_cast<int16_t>(inData.words[0]);
        temperatureCelsius = scaleFromWord(rawVal, maxScaleCelsius);
        return status;
    }

    status.messageError = true;
    return status;
}
