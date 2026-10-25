//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include <cstddef>
#include <iostream>
#include <vector>

#include "hal/hal.hpp"
#include "mil1553/types.hpp"

// Log entry for a monitored 1553 transaction
struct MonitoredMessage {
    CommandWord command{};
    DataBlock1553 data{};
    StatusWord status{};
    bool success = false;
};

// Bus Monitor (BM): Passive listener recording bus traffic
class BM {
public:
    BM() = default;
    virtual ~BM() = default;

    // Record a completed transaction from the bus
    void recordTransaction(const CommandWord& cmd, const DataBlock1553& data, const StatusWord& status, bool success);

    // Access recorded transactions
    const std::vector<MonitoredMessage>& getLog() const;
    size_t getMessageCount() const;
    void clear();

    // Print summary of recorded bus traffic
    void printLog(std::ostream& os = std::cout) const;

    // Hardware Abstraction Layer interface
    Hal1553::Transceiver& getTransceiver();
    const Hal1553::Transceiver& getTransceiver() const;

private:
    Hal1553::Transceiver transceiver_{};
    std::vector<MonitoredMessage> log_{};
};
