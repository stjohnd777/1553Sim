//
// Created by Daniel St. John St. John on 10/21/26.
//

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#include "mil1553/SimTime.hpp"

// 16-bit Command Word representation
struct CommandWord {
    uint8_t rtAddress = 0;   // 0 - 30 (31 = Broadcast)
    bool isTransmit = false; // true = RT->BC (Transmit), false = BC->RT (Receive)
    uint8_t subAddress = 0;  // 1 - 30 (0 & 31 are Mode Codes)
    uint8_t wordCount = 0;   // 1 - 32 words (0 represents 32 in 1553 spec for subaddresses 1-30; 0-31 is Mode Code for subaddresses 0/31)

    // Check if subaddress indicates a Mode Code (subaddresses 00000 or 11111)
    bool isModeCode() const {
        return (subAddress == 0 || subAddress == 31);
    }

    // Check if address is the standard MIL-STD-1553 Broadcast address (31)
    bool isBroadcast() const {
        return (rtAddress == 31);
    }

    // Returns the effective payload word count (1 to 32 for data transfers, 0 or 1 for Mode Codes)
    uint8_t getEffectiveWordCount() const {
        if (isModeCode()) {
            // MIL-STD-1553B: Mode codes 16-31 have 1 data word; mode codes 0-15 have 0 data words
            return (wordCount >= 16) ? 1 : 0;
        }
        return (wordCount == 0 || wordCount == 32) ? 32 : (wordCount & 0x1F);
    }

    // Returns the 5-bit Mode Code value (when isModeCode() is true)
    uint8_t getModeCode() const {
        return (wordCount & 0x1F);
    }

    uint16_t toRaw() const {
        uint16_t wcField = 0;
        if (isModeCode()) {
            wcField = static_cast<uint16_t>(wordCount & 0x1F);
        } else {
            wcField = static_cast<uint16_t>((wordCount == 32 || wordCount == 0) ? 0 : (wordCount & 0x1F));
        }

        uint16_t raw = (static_cast<uint16_t>(rtAddress & 0x1F) << 11) |
                       (static_cast<uint16_t>(isTransmit ? 1 : 0) << 10) |
                       (static_cast<uint16_t>(subAddress & 0x1F) << 5) |
                       wcField;
        return raw;
    }

    static CommandWord fromRaw(uint16_t raw) {
        CommandWord cmd;
        cmd.rtAddress = static_cast<uint8_t>((raw >> 11) & 0x1F);
        cmd.isTransmit = ((raw >> 10) & 0x01) != 0;
        cmd.subAddress = static_cast<uint8_t>((raw >> 5) & 0x1F);
        uint8_t wc = static_cast<uint8_t>(raw & 0x1F);
        if (cmd.isModeCode()) {
            cmd.wordCount = wc; // Preserve raw mode code (0-31) without converting 0 to 32
        } else {
            cmd.wordCount = (wc == 0) ? 32 : wc;
        }
        return cmd;
    }
};

// 16-bit Status Word representation
struct StatusWord {
    uint8_t rtAddress = 0;
    bool messageError = false;
    bool busy = false;
    bool serviceRequest = false;
    bool subsystemFlag = false;

    bool isOk() const {
        return !messageError && !busy && !subsystemFlag;
    }

    uint16_t toRaw() const {
        uint16_t raw = (static_cast<uint16_t>(rtAddress & 0x1F) << 11);
        if (messageError)   raw |= (1u << 10);
        if (serviceRequest) raw |= (1u << 8);
        if (busy)           raw |= (1u << 3);
        if (subsystemFlag)  raw |= (1u << 2);
        return raw;
    }

    static StatusWord fromRaw(uint16_t raw) {
        StatusWord status;
        status.rtAddress = static_cast<uint8_t>((raw >> 11) & 0x1F);
        status.messageError = ((raw >> 10) & 0x01) != 0;
        status.serviceRequest = ((raw >> 8) & 0x01) != 0;
        status.busy = ((raw >> 3) & 0x01) != 0;
        status.subsystemFlag = ((raw >> 2) & 0x01) != 0;
        return status;
    }
};

// Buffer container for 1553 data words (up to 32 words)
struct DataBlock1553 {
    static constexpr size_t MAX_WORDS = 32;
    uint8_t count = 0;
    std::array<uint16_t, MAX_WORDS> words{};

    bool isValid() const {
        return count <= MAX_WORDS;
    }

    uint16_t getWord(size_t index) const {
        if (index < count && index < MAX_WORDS) {
            return words[index];
        }
        return 0;
    }

    bool setWord(size_t index, uint16_t val) {
        if (index < MAX_WORDS) {
            words[index] = val;
            if (index >= count) {
                count = static_cast<uint8_t>(index + 1);
            }
            return true;
        }
        return false;
    }
};

// Result of a standard BC-RT bus transaction
struct TransactionResult {
    bool success = false;
    StatusWord status{};
    DataBlock1553 data{};
};

// Result of an RT-to-RT cross-terminal bus transaction
struct RtToRtTransactionResult {
    bool success = false;
    StatusWord rxStatus{};
    StatusWord txStatus{};
    DataBlock1553 data{};
};

// Unique identifier for scheduled transactions
using TransactionId = uint64_t;

// Type of 1553 bus transaction
enum class TransactionType {
    BcToRt,   // BC writes data to RT (Receive)
    RtToBc,   // BC reads data from RT (Transmit)
    RtToRt,   // RT transfers data directly to another RT
    ModeCode  // Mode command transfer
};

// Detailed event completion record for discrete-event simulation
struct TransactionCompletion {
    TransactionId id = 0;
    TransactionType type = TransactionType::BcToRt;
    SimTime scheduledStart = SimTime::zero();
    SimTime actualStart = SimTime::zero();
    SimTime completionTime = SimTime::zero();
    SimTime duration = SimTime::zero();
    bool isCompleted = false;

    // Results (populated once completionTime is reached)
    TransactionResult result{};
    RtToRtTransactionResult rtToRtResult{};
};

// Conversion helpers: Scales physical floating-point quantities to signed 16-bit integers
inline int16_t scaleToWord(double value, double maxVal) {
    if (maxVal == 0.0) return 0;
    double normalized = std::clamp(value / maxVal, -1.0, 1.0);
    return static_cast<int16_t>(normalized * 32767.0);
}

inline double scaleFromWord(int16_t word, double maxVal) {
    return (static_cast<double>(word) / 32767.0) * maxVal;
}
