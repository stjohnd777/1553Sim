//
// Created by Daniel St. John St. John on 10/21/26.
//

#include "sensor/ImuRT.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ImuRT::ImuRT(uint8_t address) : RT(address) {}

void ImuRT::stepSimulation(double dtSeconds) {
    pitchDeg += pitchRate * dtSeconds;
}

static double getUnitsMax(ImuUnits units) {
    switch (units) {
        case ImuUnits::RADIANS:
            return M_PI;
        case ImuUnits::MILLIRADIANS:
            return M_PI * 1000.0;
        case ImuUnits::DEGREES:
        default:
            return 180.0;
    }
}

static double convertFromDeg(double deg, ImuUnits units) {
    switch (units) {
        case ImuUnits::RADIANS:
            return deg * (M_PI / 180.0);
        case ImuUnits::MILLIRADIANS:
            return deg * (M_PI * 1000.0 / 180.0);
        case ImuUnits::DEGREES:
        default:
            return deg;
    }
}

static double convertToDeg(double val, ImuUnits units) {
    switch (units) {
        case ImuUnits::RADIANS:
            return val * (180.0 / M_PI);
        case ImuUnits::MILLIRADIANS:
            return val * (180.0 / (M_PI * 1000.0));
        case ImuUnits::DEGREES:
        default:
            return val;
    }
}

StatusWord ImuRT::handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) {
    StatusWord status{address_};

    if (subAddress == 1) { // Subaddress 1: Attitude Data (Pitch, Roll, Yaw - up to 3 words)
        if (wordCount == 0 || wordCount > 3) {
            status.messageError = true;
            return status;
        }
        outData.count = wordCount;
        const double maxVal = getUnitsMax(units);
        if (outData.count >= 1) outData.words[0] = static_cast<uint16_t>(scaleToWord(convertFromDeg(pitchDeg, units), maxVal));
        if (outData.count >= 2) outData.words[1] = static_cast<uint16_t>(scaleToWord(convertFromDeg(rollDeg, units), maxVal));
        if (outData.count >= 3) outData.words[2] = static_cast<uint16_t>(scaleToWord(convertFromDeg(yawDeg, units), maxVal));
        return status;
    }

    if (subAddress == 2) { // Subaddress 2: Units / Configuration (1 word)
        if (wordCount != 1) {
            status.messageError = true;
            return status;
        }
        outData.count = 1;
        outData.words[0] = static_cast<uint16_t>(units);
        return status;
    }

    if (subAddress == 3) { // Subaddress 3: Cross-IMU Reference Telemetry (up to 3 words)
        if (wordCount == 0 || wordCount > 3) {
            status.messageError = true;
            return status;
        }
        outData.count = wordCount;
        const double maxVal = getUnitsMax(units);
        if (outData.count >= 1) outData.words[0] = static_cast<uint16_t>(scaleToWord(convertFromDeg(crossPitchDeg, units), maxVal));
        if (outData.count >= 2) outData.words[1] = static_cast<uint16_t>(scaleToWord(convertFromDeg(crossRollDeg, units), maxVal));
        if (outData.count >= 3) outData.words[2] = static_cast<uint16_t>(scaleToWord(convertFromDeg(crossYawDeg, units), maxVal));
        return status;
    }

    status.messageError = true;
    return status;
}

StatusWord ImuRT::handleReceive(uint8_t subAddress, const DataBlock1553& inData) {
    StatusWord status{address_};

    if (subAddress == 2) { // Subaddress 2: Set Units / Configuration (exact 1 word)
        if (inData.count == 1) {
            auto rawUnit = inData.words[0];
            if (rawUnit <= static_cast<uint16_t>(ImuUnits::MILLIRADIANS)) {
                units = static_cast<ImuUnits>(rawUnit);
                return status;
            }
        }
        status.messageError = true;
        return status;
    }

    if (subAddress == 3) { // Subaddress 3: Receive Cross-IMU attitude reference (exact 3 words)
        if (inData.count == 3) {
            const double maxVal = getUnitsMax(units);
            double p = scaleFromWord(static_cast<int16_t>(inData.words[0]), maxVal);
            double r = scaleFromWord(static_cast<int16_t>(inData.words[1]), maxVal);
            double y = scaleFromWord(static_cast<int16_t>(inData.words[2]), maxVal);
            crossPitchDeg = convertToDeg(p, units);
            crossRollDeg = convertToDeg(r, units);
            crossYawDeg = convertToDeg(y, units);
            hasCrossData = true;
            return status;
        }
        status.messageError = true;
        return status;
    }

    status.messageError = true;
    return status;
}
