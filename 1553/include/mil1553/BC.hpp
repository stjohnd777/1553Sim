//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "hal/hal.hpp"
#include "mil1553/BM.hpp"
#include "mil1553/RT.hpp"
#include "mil1553/SimTime.hpp"
#include "mil1553/types.hpp"

// Bus Controller (BC) and Simulated 1553 Bus with Discrete-Event Simulation Engine
class BC {
public:
    BC();
    virtual ~BC() = default;

    // Attach a Remote Terminal to the bus (addresses 0 - 30)
    void attachTerminal(std::shared_ptr<RT> terminal);

    // Get an attached Remote Terminal by address
    std::shared_ptr<RT> getTerminal(uint8_t address) const;

    // Attach a Bus Monitor to passively record bus transactions
    void attachMonitor(std::shared_ptr<BM> monitor);

    // =========================================================================
    // Discrete-Event Simulation & Asynchronous Transaction API
    // =========================================================================

    // Non-blocking submission of a standard BC command transaction.
    // If earliestStart is not specified or is <= current simulation time, the transaction
    // will be scheduled as soon as the bus becomes available.
    TransactionId submit(const CommandWord& cmd, const DataBlock1553& data = {}, SimTime earliestStart = SimTime::zero());

    // Non-blocking convenience: BC writes data to RT subaddress
    TransactionId submitTransmitToRT(uint8_t rtAddress, uint8_t subAddress, const DataBlock1553& data, SimTime earliestStart = SimTime::zero());

    // Non-blocking convenience: BC reads data from RT subaddress
    TransactionId submitReceiveFromRT(uint8_t rtAddress, uint8_t subAddress, uint8_t wordCount, SimTime earliestStart = SimTime::zero());

    // Non-blocking submission of an RT-to-RT cross-terminal transfer
    TransactionId submitRtToRt(uint8_t rxAddress, uint8_t rxSubAddress,
                               uint8_t txAddress, uint8_t txSubAddress,
                               uint8_t wordCount, SimTime earliestStart = SimTime::zero());

    // Poll status and results of a submitted transaction.
    // Returns std::nullopt if the transaction ID is unknown.
    // If current simulation time < completionTime, returns record with isCompleted = false.
    // If current simulation time >= completionTime, returns record with isCompleted = true and populated result.
    std::optional<TransactionCompletion> poll(TransactionId id) const;

    // Advances simulation time to upToTime and returns all transactions completed up to that point.
    std::vector<TransactionCompletion> processPending(SimTime upToTime);

    // Timeline control and inspection
    SimTime getCurrentTime() const;
    void setCurrentTime(SimTime time);
    void advanceTime(SimTime delta);
    void advanceTo(SimTime targetTime);
    SimTime getBusAvailableTime() const;
    bool isBusBusy(SimTime atTime = SimTime::zero()) const;
    void resetTimeline();

    // Waveform simulation fidelity toggle:
    // - false (default): Fast transaction engine (direct protocol parsing & exact time calculation)
    // - true: Full physical waveform simulation (Manchester encoding/decoding & differential wire driving)
    void setWaveformFidelity(bool enabled);
    bool isWaveformFidelityEnabled() const;

    // Protocol timing configuration
    void setRtResponseTime(SimTime responseTime);
    SimTime getRtResponseTime() const;
    void setIntermessageGap(SimTime gap);
    SimTime getIntermessageGap() const;
    void setResponseTimeout(SimTime timeout);
    SimTime getResponseTimeout() const;

    // =========================================================================
    // Synchronous Execution API (Advances simulation clock to completion time)
    // =========================================================================

    // Execute a 1553 command transaction across the bus and advance simulation time
    TransactionResult execute(const CommandWord& cmd, const DataBlock1553& data = {});

    // Convenience helper: BC writes data to RT subaddress (Receive command from RT perspective)
    TransactionResult transmitToRT(uint8_t rtAddress, uint8_t subAddress, const DataBlock1553& data);

    // Convenience helper: BC reads data from RT subaddress (Transmit command from RT perspective)
    TransactionResult receiveFromRT(uint8_t rtAddress, uint8_t subAddress, uint8_t wordCount);

    // RT-to-RT Transfer: BC directs transmitting RT to send data directly to receiving RT over the bus
    RtToRtTransactionResult transferRtToRt(uint8_t rxAddress, uint8_t rxSubAddress,
                                           uint8_t txAddress, uint8_t txSubAddress,
                                           uint8_t wordCount);

    // Hardware Abstraction Layer interface accessors
    Hal1553::Wire& getWire();
    const Hal1553::Wire& getWire() const;
    Hal1553::Transceiver& getTransceiver();
    const Hal1553::Transceiver& getTransceiver() const;

private:
    // Core transaction execution helpers
    TransactionResult executeBcToRtFast(const CommandWord& cmd, const DataBlock1553& data);
    TransactionResult executeRtToBcFast(const CommandWord& cmd);
    RtToRtTransactionResult executeRtToRtFast(uint8_t rxAddress, uint8_t rxSubAddress,
                                             uint8_t txAddress, uint8_t txSubAddress,
                                             uint8_t wordCount);

    TransactionResult executeBcToRtWaveform(const CommandWord& cmd, const DataBlock1553& data);
    TransactionResult executeRtToBcWaveform(const CommandWord& cmd);
    RtToRtTransactionResult executeRtToRtWaveform(uint8_t rxAddress, uint8_t rxSubAddress,
                                                 uint8_t txAddress, uint8_t txSubAddress,
                                                 uint8_t wordCount);

    Hal1553::Wire busWire_{};
    Hal1553::Transceiver transceiver_{};
    std::array<std::shared_ptr<RT>, 32> terminals_{};
    std::vector<std::shared_ptr<BM>> monitors_{};

    // Discrete-event timeline state
    SimTime currentTime_{SimTime::zero()};
    SimTime busAvailableTime_{SimTime::zero()};
    TransactionId nextTransactionId_{1};
    bool waveformFidelity_{true};

    SimTime rtResponseTime_{Mil1553Timing::NominalRtResponse};
    SimTime intermessageGap_{Mil1553Timing::MinIntermessageGap};
    SimTime responseTimeout_{Mil1553Timing::ResponseTimeout};

    std::map<TransactionId, TransactionCompletion> transactions_{};
};

// Backwards-compatible alias for simulation code
using Simulated1553Bus = BC;
