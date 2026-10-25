//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include "mil1553/RT.hpp"

// Measurement unit selection for IMU attitude telemetry
enum class ImuUnits : uint16_t {
    DEGREES = 0,
    RADIANS = 1,
    MILLIRADIANS = 2
};

// Concrete IMU Remote Terminal
class ImuRT : public RT {
public:
    explicit ImuRT(uint8_t address);
    ~ImuRT() override = default;

    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    double yawDeg = 0.0;
    double pitchRate = 0.5; // deg/sec drift/rate for simulation
    ImuUnits units = ImuUnits::DEGREES;

    // Cross-IMU received reference telemetry (from RT-to-RT transfers)
    double crossPitchDeg = 0.0;
    double crossRollDeg = 0.0;
    double crossYawDeg = 0.0;
    bool hasCrossData = false;

    void stepSimulation(double dtSeconds) override;
    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override;
    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override;
};
