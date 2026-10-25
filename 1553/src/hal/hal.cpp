#include "hal/hal.hpp"
#include <stdexcept>

void Hal1553::Clock::setFrequency(double hz) {
    if (hz <= 0.0) {
        throw std::invalid_argument("Invalid frequency");
    }
    frequencyHz = hz;
}

double Hal1553::Clock::halfBitPeriodNs() const {
    return 500'000'000.0 / frequencyHz;
}

void Hal1553::Clock::tick() {
    ++tickCount;
}

void Hal1553::Clock::reset() {
    tickCount = 0;
}

double Hal1553::Wire::differentialVoltage() const {
    return positiveVoltage - negativeVoltage;
}

void Hal1553::Wire::drive(Level level, double amplitude) {
    const double v = static_cast<int>(level) * amplitude / 2.0;
    positiveVoltage = v;
    negativeVoltage = -v;
}

void Hal1553::Wire::release() {
    positiveVoltage = 0.0;
    negativeVoltage = 0.0;
}

void Hal1553::Wire::clear() {
    positiveVoltage = 0.0;
    negativeVoltage = 0.0;
}

bool Hal1553::Manchester::oddParity(u16 value) {
#if defined(__GNUC__) || defined(__clang__)
    return (__builtin_popcount(static_cast<unsigned>(value)) % 2) == 0;
#else
    unsigned int v = value;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1) == 0;
#endif
}

Hal1553::Manchester::Waveform Hal1553::Manchester::encode(u16 value, WordType type) {
    Waveform wave{};
    // Special sync waveform: 3 bit-times.
    // Command/status: +++---
    // Data:           ---+++
    const bool dataSync = (type == WordType::Data);

    for (int i = 0; i < 6; ++i) {
        const bool positive = dataSync ? (i >= 3) : (i < 3);
        wave[i] = positive ? Level::Positive : Level::Negative;
    }

    // 16 bits, MSB first.
    for (int bit = 15; bit >= 0; --bit) {
        const bool one = (value & (1u << bit)) != 0;
        const int offset = 6 + (15 - bit) * 2;
        // MIL-STD-1553 Manchester II:
        // 1 = positive then negative.
        // 0 = negative then positive.
        wave[offset] = one ? Level::Positive : Level::Negative;
        wave[offset + 1] = one ? Level::Negative : Level::Positive;
    }
    // Odd parity: final bit.
    const bool parity = oddParity(value);
    wave[38] = parity ? Level::Positive : Level::Negative;
    wave[39] = parity ? Level::Negative : Level::Positive;
    return wave;
}

bool Hal1553::Manchester::decode(const Waveform& wave, WordType& outType, u16& outValue) {
    // Check sync waveform (first 6 cells)
    const bool isCommandOrStatus = (wave[0] == Level::Positive && wave[1] == Level::Positive && wave[2] == Level::Positive &&
                                    wave[3] == Level::Negative && wave[4] == Level::Negative && wave[5] == Level::Negative);
    const bool isData = (wave[0] == Level::Negative && wave[1] == Level::Negative && wave[2] == Level::Negative &&
                         wave[3] == Level::Positive && wave[4] == Level::Positive && wave[5] == Level::Positive);

    if (isCommandOrStatus) {
        outType = WordType::Command; // Caller distinguishes Command vs Status
    } else if (isData) {
        outType = WordType::Data;
    } else {
        return false; // Sync invalid
    }

    // Decode 16 data bits
    u16 value = 0;
    for (int bit = 15; bit >= 0; --bit) {
        const int offset = 6 + (15 - bit) * 2;
        const Level first = wave[offset];
        const Level second = wave[offset + 1];

        if (first == Level::Positive && second == Level::Negative) {
            value |= (1u << bit);
        } else if (first == Level::Negative && second == Level::Positive) {
            // bit is 0
        } else {
            return false; // Manchester violation
        }
    }

    // Decode parity bit (cells 38, 39)
    bool parityBit = false;
    if (wave[38] == Level::Positive && wave[39] == Level::Negative) {
        parityBit = true;
    } else if (wave[38] == Level::Negative && wave[39] == Level::Positive) {
        parityBit = false;
    } else {
        return false; // Parity cell Manchester violation
    }

    // Verify odd parity
    if (oddParity(value) != parityBit) {
        return false; // Parity mismatch
    }

    outValue = value;
    return true;
}

void Hal1553::Transceiver::TX::enable() {
    enabled = true;
}

void Hal1553::Transceiver::TX::disable() {
    enabled = false;
}

Level Hal1553::Transceiver::RX::sample(const Wire& wire) const {
    if (!enabled) {
        return Level::Idle;
    }
    const double v = wire.differentialVoltage();
    if (v > 0.5) {
        return Level::Positive;
    }
    if (v < -0.5) {
        return Level::Negative;
    }
    return Level::Idle;
}

void Hal1553::driveWord(Transceiver& txXcvr, Wire& wire, u16 value, WordType type) {
    txXcvr.tx.enable();
    auto wave = Manchester::encode(value, type);
    for (Level level : wave) {
        txXcvr.clock.tick();
        wire.drive(level);
    }
    wire.release();
    txXcvr.tx.disable();
}
