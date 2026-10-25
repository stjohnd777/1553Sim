//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include <cstdint>
#include <vector>
#include "hal/hal.hpp"
#include "mil1553/types.hpp"

// Base Remote Terminal (RT)
class RT {
public:
    explicit RT(uint8_t address);
    virtual ~RT() = default;

    uint8_t getAddress() const;

    // Called when the BC issues a Transmit Command (RT -> BC)
    virtual StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData);

    // Called when the BC issues a Receive Command (BC -> RT)
    virtual StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData);

    // Advance subsystem dynamics or internal state
    virtual void stepSimulation(double dtSeconds);

    // Hardware Abstraction Layer interface
    Hal1553::Transceiver& getTransceiver();
    const Hal1553::Transceiver& getTransceiver() const;

    // Execute Receive over HAL: samples data waveforms, decodes them, calls handleReceive, and transmits StatusWord
    bool executeHalReceive(Hal1553::Wire& wire, const CommandWord& cmd, const std::vector<Hal1553::Manchester::Waveform>& dataWaveforms, Hal1553::Manchester::Waveform& outStatusWave);

    // Execute Transmit over HAL: calls handleTransmit, and transmits StatusWord followed by data waveforms onto wire
    bool executeHalTransmit(Hal1553::Wire& wire, const CommandWord& cmd, Hal1553::Manchester::Waveform& outStatusWave, std::vector<Hal1553::Manchester::Waveform>& outDataWaves);

protected:
    uint8_t address_;
    Hal1553::Transceiver transceiver_{};
};
