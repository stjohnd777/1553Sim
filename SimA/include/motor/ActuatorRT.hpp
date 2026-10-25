//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include "mil1553/RT.hpp"

// Concrete Control Surface Actuator Remote Terminal
class ActuatorRT : public RT {
public:
    explicit ActuatorRT(uint8_t address);
    ~ActuatorRT() override = default;

    double commandedDeflectionDeg = 0.0;
    double actualDeflectionDeg = 0.0;

    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override;
    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override;
};
