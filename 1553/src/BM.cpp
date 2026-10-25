//
// Created by Daniel St. John St. John on 10/21/26.
//

#include "mil1553/BM.hpp"
#include <iomanip>

void BM::recordTransaction(const CommandWord& cmd, const DataBlock1553& data, const StatusWord& status, bool success) {
    MonitoredMessage msg;
    msg.command = cmd;
    msg.data = data;
    msg.status = status;
    msg.success = success;
    log_.push_back(msg);
}

const std::vector<MonitoredMessage>& BM::getLog() const {
    return log_;
}

Hal1553::Transceiver& BM::getTransceiver() {
    return transceiver_;
}

const Hal1553::Transceiver& BM::getTransceiver() const {
    return transceiver_;
}

size_t BM::getMessageCount() const {
    return log_.size();
}

void BM::clear() {
    log_.clear();
}

void BM::printLog(std::ostream& os) const {
    os << "=== MIL-STD-1553 Bus Monitor Log (" << log_.size() << " entries) ===\n";
    for (size_t i = 0; i < log_.size(); ++i) {
        const auto& entry = log_[i];
        os << "[" << std::setw(3) << i << "] "
           << (entry.command.isTransmit ? "TX" : "RX")
           << " RT:" << static_cast<int>(entry.command.rtAddress)
           << " SA:" << static_cast<int>(entry.command.subAddress)
           << " WC:" << static_cast<int>(entry.command.wordCount)
           << " | Status: " << (entry.success ? "OK" : "ERR")
           << " (Words: " << static_cast<int>(entry.data.count) << ")\n";
    }
}
