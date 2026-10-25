//
// Created by Daniel St. John St. John on 10/21/26.
//

#include "mil1553/BC.hpp"
#include <algorithm>

BC::BC() : transceiver_() {
    transceiver_.rx.enabled = true;
}

void BC::attachTerminal(std::shared_ptr<RT> terminal) {
    if (terminal && terminal->getAddress() <= 30 && terminal->getAddress() < terminals_.size()) {
        terminals_[terminal->getAddress()] = terminal;
    }
}

std::shared_ptr<RT> BC::getTerminal(uint8_t address) const {
    if (address <= 30 && address < terminals_.size()) {
        return terminals_[address];
    }
    return nullptr;
}

void BC::attachMonitor(std::shared_ptr<BM> monitor) {
    if (monitor) {
        monitors_.push_back(monitor);
    }
}

Hal1553::Wire& BC::getWire() {
    return busWire_;
}

const Hal1553::Wire& BC::getWire() const {
    return busWire_;
}

Hal1553::Transceiver& BC::getTransceiver() {
    return transceiver_;
}

const Hal1553::Transceiver& BC::getTransceiver() const {
    return transceiver_;
}

// =========================================================================
// Discrete-Event Timeline Control
// =========================================================================

SimTime BC::getCurrentTime() const {
    return currentTime_;
}

void BC::setCurrentTime(SimTime time) {
    currentTime_ = time;
}

void BC::advanceTime(SimTime delta) {
    currentTime_ += delta;
}

void BC::advanceTo(SimTime targetTime) {
    if (targetTime > currentTime_) {
        currentTime_ = targetTime;
    }
}

SimTime BC::getBusAvailableTime() const {
    return busAvailableTime_;
}

bool BC::isBusBusy(SimTime atTime) const {
    SimTime checkTime = (atTime == SimTime::zero()) ? currentTime_ : atTime;
    for (const auto& [id, record] : transactions_) {
        if (record.actualStart <= checkTime && checkTime < record.completionTime) {
            return true;
        }
    }
    return false;
}

void BC::resetTimeline() {
    currentTime_ = SimTime::zero();
    busAvailableTime_ = SimTime::zero();
    transactions_.clear();
    transceiver_.clock.reset();
}

void BC::setWaveformFidelity(bool enabled) {
    waveformFidelity_ = enabled;
}

bool BC::isWaveformFidelityEnabled() const {
    return waveformFidelity_;
}

void BC::setRtResponseTime(SimTime responseTime) {
    rtResponseTime_ = responseTime;
}

SimTime BC::getRtResponseTime() const {
    return rtResponseTime_;
}

void BC::setIntermessageGap(SimTime gap) {
    intermessageGap_ = gap;
}

SimTime BC::getIntermessageGap() const {
    return intermessageGap_;
}

void BC::setResponseTimeout(SimTime timeout) {
    responseTimeout_ = timeout;
}

SimTime BC::getResponseTimeout() const {
    return responseTimeout_;
}

// =========================================================================
// Asynchronous Transaction Submission & Polling
// =========================================================================

TransactionId BC::submit(const CommandWord& cmd, const DataBlock1553& data, SimTime earliestStart) {
    TransactionId id = nextTransactionId_++;

    TransactionCompletion record{};
    record.id = id;
    record.scheduledStart = earliestStart;
    record.actualStart = std::max(earliestStart, std::max(currentTime_, busAvailableTime_));

    // Handle broadcast or invalid terminal addresses (> 30)
    if (cmd.rtAddress > 30 || cmd.isBroadcast()) {
        record.type = cmd.isTransmit ? TransactionType::RtToBc : TransactionType::BcToRt;
        record.duration = Mil1553Timing::calculateTimeoutDuration(responseTimeout_, intermessageGap_);
        record.completionTime = record.actualStart + record.duration;
        busAvailableTime_ = record.completionTime;

        record.result.success = false;
        record.result.status.rtAddress = cmd.rtAddress;
        record.result.status.messageError = true;

        for (const auto& monitor : monitors_) {
            if (monitor && monitor->getTransceiver().rx.enabled) {
                monitor->recordTransaction(cmd, data, record.result.status, false);
            }
        }

        transactions_[id] = record;
        return id;
    }

    const uint8_t expectedWordCount = cmd.getEffectiveWordCount();

    // Validate payload parameter bounds for Receive commands (BC -> RT)
    if (!cmd.isTransmit) {
        if (!cmd.isModeCode()) {
            if (data.count != expectedWordCount || data.count == 0 || data.count > DataBlock1553::MAX_WORDS || !data.isValid()) {
                record.type = TransactionType::BcToRt;
                record.duration = Mil1553Timing::calculateTimeoutDuration(responseTimeout_, intermessageGap_);
                record.completionTime = record.actualStart + record.duration;
                busAvailableTime_ = record.completionTime;

                record.result.success = false;
                record.result.status.rtAddress = cmd.rtAddress;
                record.result.status.messageError = true;

                for (const auto& monitor : monitors_) {
                    if (monitor && monitor->getTransceiver().rx.enabled) {
                        monitor->recordTransaction(cmd, data, record.result.status, false);
                    }
                }

                transactions_[id] = record;
                return id;
            }
        } else {
            // Mode code receive with data
            if (data.count != expectedWordCount) {
                record.type = TransactionType::ModeCode;
                record.duration = Mil1553Timing::calculateTimeoutDuration(responseTimeout_, intermessageGap_);
                record.completionTime = record.actualStart + record.duration;
                busAvailableTime_ = record.completionTime;

                record.result.success = false;
                record.result.status.rtAddress = cmd.rtAddress;
                record.result.status.messageError = true;

                transactions_[id] = record;
                return id;
            }
        }
    }

    // Check if terminal is attached
    if (!terminals_[cmd.rtAddress]) {
        record.type = cmd.isTransmit ? TransactionType::RtToBc : TransactionType::BcToRt;
        record.duration = Mil1553Timing::calculateTimeoutDuration(responseTimeout_, intermessageGap_);
        record.completionTime = record.actualStart + record.duration;
        busAvailableTime_ = record.completionTime;

        record.result.success = false;
        record.result.status.rtAddress = cmd.rtAddress;
        record.result.status.messageError = true;

        for (const auto& monitor : monitors_) {
            if (monitor && monitor->getTransceiver().rx.enabled) {
                monitor->recordTransaction(cmd, data, record.result.status, false);
            }
        }

        transactions_[id] = record;
        return id;
    }

    // Execute transfer based on fidelity mode and direction
    if (cmd.isTransmit) {
        record.type = cmd.isModeCode() ? TransactionType::ModeCode : TransactionType::RtToBc;
        record.duration = Mil1553Timing::calculateRtToBcDuration(expectedWordCount, rtResponseTime_, intermessageGap_);
        record.completionTime = record.actualStart + record.duration;
        busAvailableTime_ = record.completionTime;

        if (waveformFidelity_) {
            record.result = executeRtToBcWaveform(cmd);
        } else {
            record.result = executeRtToBcFast(cmd);
        }
    } else {
        record.type = cmd.isModeCode() ? TransactionType::ModeCode : TransactionType::BcToRt;
        record.duration = Mil1553Timing::calculateBcToRtDuration(expectedWordCount, rtResponseTime_, intermessageGap_);
        record.completionTime = record.actualStart + record.duration;
        busAvailableTime_ = record.completionTime;

        if (waveformFidelity_) {
            record.result = executeBcToRtWaveform(cmd, data);
        } else {
            record.result = executeBcToRtFast(cmd, data);
        }
    }

    transactions_[id] = record;
    return id;
}

TransactionId BC::submitTransmitToRT(uint8_t rtAddress, uint8_t subAddress, const DataBlock1553& data, SimTime earliestStart) {
    CommandWord cmd;
    cmd.rtAddress = rtAddress;
    cmd.isTransmit = false;
    cmd.subAddress = subAddress;
    cmd.wordCount = (data.count == 32) ? 0 : data.count;
    return submit(cmd, data, earliestStart);
}

TransactionId BC::submitReceiveFromRT(uint8_t rtAddress, uint8_t subAddress, uint8_t wordCount, SimTime earliestStart) {
    CommandWord cmd;
    cmd.rtAddress = rtAddress;
    cmd.isTransmit = true;
    cmd.subAddress = subAddress;
    cmd.wordCount = wordCount;
    return submit(cmd, {}, earliestStart);
}

TransactionId BC::submitRtToRt(uint8_t rxAddress, uint8_t rxSubAddress,
                               uint8_t txAddress, uint8_t txSubAddress,
                               uint8_t wordCount, SimTime earliestStart) {
    TransactionId id = nextTransactionId_++;

    TransactionCompletion record{};
    record.id = id;
    record.type = TransactionType::RtToRt;
    record.scheduledStart = earliestStart;
    record.actualStart = std::max(earliestStart, std::max(currentTime_, busAvailableTime_));

    const uint8_t effectiveWordCount = (wordCount == 0 || wordCount == 32) ? 32 : (wordCount & 0x1F);

    if (rxAddress > 30 || txAddress > 30 || rxAddress == txAddress ||
        rxSubAddress == 0 || rxSubAddress == 31 || txSubAddress == 0 || txSubAddress == 31 ||
        effectiveWordCount == 0 || effectiveWordCount > DataBlock1553::MAX_WORDS ||
        !terminals_[rxAddress] || !terminals_[txAddress]) {

        record.duration = Mil1553Timing::calculateTimeoutDuration(responseTimeout_, intermessageGap_);
        record.completionTime = record.actualStart + record.duration;
        busAvailableTime_ = record.completionTime;

        record.rtToRtResult.success = false;
        record.rtToRtResult.rxStatus.rtAddress = rxAddress;
        record.rtToRtResult.rxStatus.messageError = (rxAddress > 30 || !terminals_[rxAddress]);
        record.rtToRtResult.txStatus.rtAddress = txAddress;
        record.rtToRtResult.txStatus.messageError = (txAddress > 30 || !terminals_[txAddress]);

        transactions_[id] = record;
        return id;
    }

    record.duration = Mil1553Timing::calculateRtToRtDuration(effectiveWordCount, rtResponseTime_, rtResponseTime_, intermessageGap_);
    record.completionTime = record.actualStart + record.duration;
    busAvailableTime_ = record.completionTime;

    if (waveformFidelity_) {
        record.rtToRtResult = executeRtToRtWaveform(rxAddress, rxSubAddress, txAddress, txSubAddress, wordCount);
    } else {
        record.rtToRtResult = executeRtToRtFast(rxAddress, rxSubAddress, txAddress, txSubAddress, wordCount);
    }

    transactions_[id] = record;
    return id;
}

std::optional<TransactionCompletion> BC::poll(TransactionId id) const {
    auto it = transactions_.find(id);
    if (it == transactions_.end()) {
        return std::nullopt;
    }

    TransactionCompletion recordCopy = it->second;
    if (currentTime_ >= recordCopy.completionTime) {
        recordCopy.isCompleted = true;
    } else {
        recordCopy.isCompleted = false;
        // Obscure in-flight result payload until simulated completion timestamp is reached
        recordCopy.result = TransactionResult{};
        recordCopy.rtToRtResult = RtToRtTransactionResult{};
    }
    return recordCopy;
}

std::vector<TransactionCompletion> BC::processPending(SimTime upToTime) {
    if (upToTime > currentTime_) {
        currentTime_ = upToTime;
    }

    std::vector<TransactionCompletion> completed;
    for (const auto& [id, record] : transactions_) {
        if (record.completionTime <= currentTime_) {
            TransactionCompletion rec = record;
            rec.isCompleted = true;
            completed.push_back(rec);
        }
    }
    return completed;
}

// =========================================================================
// Synchronous API Implementation (Timeline-Aware)
// =========================================================================

TransactionResult BC::execute(const CommandWord& cmd, const DataBlock1553& data) {
    TransactionId id = submit(cmd, data, currentTime_);
    auto it = transactions_.find(id);
    if (it != transactions_.end()) {
        currentTime_ = it->second.completionTime;
        return it->second.result;
    }
    return {};
}

TransactionResult BC::transmitToRT(uint8_t rtAddress, uint8_t subAddress, const DataBlock1553& data) {
    CommandWord cmd;
    cmd.rtAddress = rtAddress;
    cmd.isTransmit = false;
    cmd.subAddress = subAddress;
    cmd.wordCount = (data.count == 32) ? 0 : data.count;
    return execute(cmd, data);
}

TransactionResult BC::receiveFromRT(uint8_t rtAddress, uint8_t subAddress, uint8_t wordCount) {
    CommandWord cmd;
    cmd.rtAddress = rtAddress;
    cmd.isTransmit = true;
    cmd.subAddress = subAddress;
    cmd.wordCount = wordCount;
    return execute(cmd);
}

RtToRtTransactionResult BC::transferRtToRt(uint8_t rxAddress, uint8_t rxSubAddress,
                                           uint8_t txAddress, uint8_t txSubAddress,
                                           uint8_t wordCount) {
    TransactionId id = submitRtToRt(rxAddress, rxSubAddress, txAddress, txSubAddress, wordCount, currentTime_);
    auto it = transactions_.find(id);
    if (it != transactions_.end()) {
        currentTime_ = it->second.completionTime;
        return it->second.rtToRtResult;
    }
    return {};
}

// =========================================================================
// Fast Transaction Engine Implementations
// =========================================================================

TransactionResult BC::executeBcToRtFast(const CommandWord& cmd, const DataBlock1553& data) {
    TransactionResult result{};
    auto& rt = terminals_[cmd.rtAddress];
    result.status = rt->handleReceive(cmd.subAddress, data);
    result.data = data;
    result.success = result.status.isOk();

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(cmd, data, result.status, result.success);
        }
    }
    return result;
}

TransactionResult BC::executeRtToBcFast(const CommandWord& cmd) {
    TransactionResult result{};
    auto& rt = terminals_[cmd.rtAddress];
    const uint8_t expectedWordCount = cmd.getEffectiveWordCount();

    result.status = rt->handleTransmit(cmd.subAddress, expectedWordCount, result.data);

    if (result.status.isOk()) {
        if (result.data.count != expectedWordCount || result.data.count > DataBlock1553::MAX_WORDS || !result.data.isValid()) {
            result.status.messageError = true;
        }
    }

    result.success = result.status.isOk() && (result.data.count == expectedWordCount);

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(cmd, result.data, result.status, result.success);
        }
    }
    return result;
}

RtToRtTransactionResult BC::executeRtToRtFast(uint8_t rxAddress, uint8_t rxSubAddress,
                                             uint8_t txAddress, uint8_t txSubAddress,
                                             uint8_t wordCount) {
    RtToRtTransactionResult result{};
    const uint8_t effectiveWordCount = (wordCount == 0 || wordCount == 32) ? 32 : (wordCount & 0x1F);

    auto& txRt = terminals_[txAddress];
    auto& rxRt = terminals_[rxAddress];

    // Transmitting RT handles transmit
    result.txStatus = txRt->handleTransmit(txSubAddress, effectiveWordCount, result.data);
    if (result.txStatus.isOk()) {
        if (result.data.count != effectiveWordCount || result.data.count > DataBlock1553::MAX_WORDS || !result.data.isValid()) {
            result.txStatus.messageError = true;
        }
    }

    if (result.txStatus.isOk()) {
        // Receiving RT handles receive
        result.rxStatus = rxRt->handleReceive(rxSubAddress, result.data);
    } else {
        result.rxStatus.rtAddress = rxAddress;
        result.rxStatus.messageError = true;
    }

    result.success = result.txStatus.isOk() && result.rxStatus.isOk() && (result.data.count == effectiveWordCount);

    CommandWord rxCmd{rxAddress, false, rxSubAddress, wordCount};
    CommandWord txCmd{txAddress, true, txSubAddress, wordCount};

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(rxCmd, result.data, result.rxStatus, result.success);
            monitor->recordTransaction(txCmd, result.data, result.txStatus, result.success);
        }
    }

    return result;
}

// =========================================================================
// Waveform Fidelity Implementations (Manchester Bit Coding & Wire Driving)
// =========================================================================

TransactionResult BC::executeBcToRtWaveform(const CommandWord& cmd, const DataBlock1553& data) {
    TransactionResult result{};

    // 1. BC transmits CommandWord onto wire
    transceiver_.tx.enable();
    auto cmdWave = Hal1553::Manchester::encode(cmd.toRaw(), WordType::Command);
    for (Level lvl : cmdWave) {
        transceiver_.clock.tick();
        busWire_.drive(lvl);
    }
    busWire_.release();

    // 2. BC transmits DataWords onto wire
    std::vector<Hal1553::Manchester::Waveform> dataWaves;
    const uint8_t transmitCount = std::min<uint8_t>(data.count, DataBlock1553::MAX_WORDS);
    for (uint8_t i = 0; i < transmitCount; ++i) {
        auto dataWave = Hal1553::Manchester::encode(data.words[i], WordType::Data);
        for (Level lvl : dataWave) {
            transceiver_.clock.tick();
            busWire_.drive(lvl);
        }
        dataWaves.push_back(dataWave);
    }
    busWire_.release();
    transceiver_.tx.disable();

    // 3. RT receives waveform and generates status
    auto& rt = terminals_[cmd.rtAddress];
    Hal1553::Manchester::Waveform statusWave{};
    rt->executeHalReceive(busWire_, cmd, dataWaves, statusWave);

    // 4. BC samples StatusWord
    WordType statusType{};
    u16 rawStatus = 0;
    if (transceiver_.rx.enabled && Hal1553::Manchester::decode(statusWave, statusType, rawStatus) && statusType != WordType::Data) {
        result.status = StatusWord::fromRaw(rawStatus);
    } else {
        result.status.rtAddress = cmd.rtAddress;
        result.status.messageError = true;
    }

    result.data = data;
    result.success = result.status.isOk();

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(cmd, data, result.status, result.success);
        }
    }
    return result;
}

TransactionResult BC::executeRtToBcWaveform(const CommandWord& cmd) {
    TransactionResult result{};
    const uint8_t expectedWordCount = cmd.getEffectiveWordCount();

    // 1. BC transmits CommandWord onto wire
    transceiver_.tx.enable();
    auto cmdWave = Hal1553::Manchester::encode(cmd.toRaw(), WordType::Command);
    for (Level lvl : cmdWave) {
        transceiver_.clock.tick();
        busWire_.drive(lvl);
    }
    busWire_.release();
    transceiver_.tx.disable();

    // 2. RT transmits StatusWord + DataWords
    auto& rt = terminals_[cmd.rtAddress];
    Hal1553::Manchester::Waveform statusWave{};
    std::vector<Hal1553::Manchester::Waveform> dataWaves;
    rt->executeHalTransmit(busWire_, cmd, statusWave, dataWaves);

    // 3. BC samples StatusWord
    WordType statusType{};
    u16 rawStatus = 0;
    if (transceiver_.rx.enabled && Hal1553::Manchester::decode(statusWave, statusType, rawStatus) && statusType != WordType::Data) {
        result.status = StatusWord::fromRaw(rawStatus);
    } else {
        result.status.rtAddress = cmd.rtAddress;
        result.status.messageError = true;
    }

    if (dataWaves.size() != expectedWordCount || dataWaves.size() > DataBlock1553::MAX_WORDS) {
        result.status.messageError = true;
    }

    // 4. BC samples DataWords
    const size_t wordsToDecode = std::min<size_t>(dataWaves.size(), DataBlock1553::MAX_WORDS);
    result.data.count = static_cast<uint8_t>(wordsToDecode);
    for (size_t i = 0; i < wordsToDecode; ++i) {
        WordType dataType{};
        u16 rawData = 0;
        if (transceiver_.rx.enabled && Hal1553::Manchester::decode(dataWaves[i], dataType, rawData) && dataType == WordType::Data) {
            result.data.words[i] = rawData;
        } else {
            result.status.messageError = true;
        }
    }

    result.success = result.status.isOk() && (dataWaves.size() == expectedWordCount);

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(cmd, result.data, result.status, result.success);
        }
    }
    return result;
}

RtToRtTransactionResult BC::executeRtToRtWaveform(uint8_t rxAddress, uint8_t rxSubAddress,
                                                 uint8_t txAddress, uint8_t txSubAddress,
                                                 uint8_t wordCount) {
    RtToRtTransactionResult result{};
    const uint8_t effectiveWordCount = (wordCount == 0 || wordCount == 32) ? 32 : (wordCount & 0x1F);

    CommandWord rxCmd{rxAddress, false, rxSubAddress, wordCount};
    CommandWord txCmd{txAddress, true, txSubAddress, wordCount};

    // 1. BC transmits Receive Command Word
    transceiver_.tx.enable();
    auto rxCmdWave = Hal1553::Manchester::encode(rxCmd.toRaw(), WordType::Command);
    for (Level lvl : rxCmdWave) {
        transceiver_.clock.tick();
        busWire_.drive(lvl);
    }
    busWire_.release();

    // 2. BC transmits Transmit Command Word
    auto txCmdWave = Hal1553::Manchester::encode(txCmd.toRaw(), WordType::Command);
    for (Level lvl : txCmdWave) {
        transceiver_.clock.tick();
        busWire_.drive(lvl);
    }
    busWire_.release();
    transceiver_.tx.disable();

    auto& rxRt = terminals_[rxAddress];
    auto& txRt = terminals_[txAddress];

    // 3. Transmitting RT outputs Tx StatusWord + DataWords
    Hal1553::Manchester::Waveform txStatusWave{};
    std::vector<Hal1553::Manchester::Waveform> dataWaves;
    txRt->executeHalTransmit(busWire_, txCmd, txStatusWave, dataWaves);

    WordType txStatusType{};
    u16 rawTxStatus = 0;
    if (transceiver_.rx.enabled && Hal1553::Manchester::decode(txStatusWave, txStatusType, rawTxStatus) && txStatusType != WordType::Data) {
        result.txStatus = StatusWord::fromRaw(rawTxStatus);
    } else {
        result.txStatus.rtAddress = txAddress;
        result.txStatus.messageError = true;
    }

    if (dataWaves.size() != effectiveWordCount || dataWaves.size() > DataBlock1553::MAX_WORDS) {
        result.txStatus.messageError = true;
    }

    const size_t wordsToDecode = std::min<size_t>(dataWaves.size(), DataBlock1553::MAX_WORDS);
    result.data.count = static_cast<uint8_t>(wordsToDecode);
    for (size_t i = 0; i < wordsToDecode; ++i) {
        WordType dataType{};
        u16 rawData = 0;
        if (transceiver_.rx.enabled && Hal1553::Manchester::decode(dataWaves[i], dataType, rawData) && dataType == WordType::Data) {
            result.data.words[i] = rawData;
        } else {
            result.txStatus.messageError = true;
        }
    }

    // 4. Receiving RT receives DataWords and responds with Rx StatusWord
    Hal1553::Manchester::Waveform rxStatusWave{};
    rxRt->executeHalReceive(busWire_, rxCmd, dataWaves, rxStatusWave);

    WordType rxStatusType{};
    u16 rawRxStatus = 0;
    if (transceiver_.rx.enabled && Hal1553::Manchester::decode(rxStatusWave, rxStatusType, rawRxStatus) && rxStatusType != WordType::Data) {
        result.rxStatus = StatusWord::fromRaw(rawRxStatus);
    } else {
        result.rxStatus.rtAddress = rxAddress;
        result.rxStatus.messageError = true;
    }

    result.success = result.txStatus.isOk() && result.rxStatus.isOk() && (dataWaves.size() == effectiveWordCount);

    for (const auto& monitor : monitors_) {
        if (monitor && monitor->getTransceiver().rx.enabled) {
            monitor->recordTransaction(rxCmd, result.data, result.rxStatus, result.success);
            monitor->recordTransaction(txCmd, result.data, result.txStatus, result.success);
        }
    }

    return result;
}
