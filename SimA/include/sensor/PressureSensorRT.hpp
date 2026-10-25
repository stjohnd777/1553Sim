//
// Created by Daniel St. John St. John on 10/25/26.
//

#pragma once

#include "mil1553/RT.hpp"

// Concrete Pressure Sensor Remote Terminal for engine chamber and propellant line pressure monitoring
class PressureSensorRT : public RT {
public:
    explicit PressureSensorRT(uint8_t address);
    ~PressureSensorRT() override = default;

    double pressureBar = 1.0;        // Current sensor reading (bar)
    double pressureRate = 0.0;       // Rate of change (bar/sec)
    double maxScaleBar = 250.0;      // Scaling range for 16-bit word (±250.0 bar)

    void stepSimulation(double dtSeconds) override;
    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override;
    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override;
};
