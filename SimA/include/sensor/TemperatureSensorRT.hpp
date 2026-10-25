//
// Created by Daniel St. John St. John on 10/25/26.
//

#pragma once

#include "mil1553/RT.hpp"

// Concrete Temperature Sensor Remote Terminal for engine thermal monitoring
class TemperatureSensorRT : public RT {
public:
    explicit TemperatureSensorRT(uint8_t address);
    ~TemperatureSensorRT() override = default;

    double temperatureCelsius = 25.0; // Current sensor reading (deg C)
    double tempRate = 0.0;             // Rate of change (deg C/sec)
    double maxScaleCelsius = 1000.0;   // Scaling range for 16-bit word (±1000.0 deg C)

    void stepSimulation(double dtSeconds) override;
    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override;
    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override;
};
