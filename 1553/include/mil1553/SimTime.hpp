//
// Created by Daniel St. John St. John on 10/24/26.
//

#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

// High-precision simulation time representation (integer nanoseconds)
class SimTime {
public:
    constexpr SimTime() : nanoseconds_(0) {}
    constexpr explicit SimTime(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}

    // Factory methods
    static constexpr SimTime fromNanoseconds(int64_t ns) {
        return SimTime(ns);
    }

    static constexpr SimTime fromMicroseconds(double us) {
        return SimTime(static_cast<int64_t>(us * 1000.0));
    }

    static constexpr SimTime fromMilliseconds(double ms) {
        return SimTime(static_cast<int64_t>(ms * 1'000'000.0));
    }

    static constexpr SimTime fromSeconds(double s) {
        return SimTime(static_cast<int64_t>(s * 1'000'000'000.0));
    }

    static constexpr SimTime zero() {
        return SimTime(0);
    }

    static constexpr SimTime max() {
        return SimTime(INT64_MAX);
    }

    // Accessors
    constexpr int64_t toNanoseconds() const { return nanoseconds_; }
    constexpr double toMicroseconds() const { return static_cast<double>(nanoseconds_) / 1000.0; }
    constexpr double toMilliseconds() const { return static_cast<double>(nanoseconds_) / 1'000'000.0; }
    constexpr double toSeconds() const { return static_cast<double>(nanoseconds_) / 1'000'000'000.0; }

    // Operators
    constexpr SimTime operator+(const SimTime& rhs) const { return SimTime(nanoseconds_ + rhs.nanoseconds_); }
    constexpr SimTime operator-(const SimTime& rhs) const { return SimTime(nanoseconds_ - rhs.nanoseconds_); }
    constexpr SimTime& operator+=(const SimTime& rhs) { nanoseconds_ += rhs.nanoseconds_; return *this; }
    constexpr SimTime& operator-=(const SimTime& rhs) { nanoseconds_ -= rhs.nanoseconds_; return *this; }

    template <typename T, typename std::enable_if<std::is_integral_v<T>, int>::type = 0>
    constexpr SimTime operator*(T scalar) const {
        return SimTime(nanoseconds_ * static_cast<int64_t>(scalar));
    }

    template <typename T, typename std::enable_if<std::is_floating_point_v<T>, int>::type = 0>
    constexpr SimTime operator*(T scalar) const {
        return SimTime(static_cast<int64_t>(static_cast<double>(nanoseconds_) * static_cast<double>(scalar)));
    }

    template <typename T, typename std::enable_if<std::is_integral_v<T>, int>::type = 0>
    constexpr SimTime operator/(T scalar) const {
        return SimTime(nanoseconds_ / static_cast<int64_t>(scalar));
    }

    constexpr bool operator==(const SimTime& rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
    constexpr bool operator!=(const SimTime& rhs) const { return nanoseconds_ != rhs.nanoseconds_; }
    constexpr bool operator<(const SimTime& rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
    constexpr bool operator<=(const SimTime& rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
    constexpr bool operator>(const SimTime& rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
    constexpr bool operator>=(const SimTime& rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

    // Human-readable string representation
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        if (nanoseconds_ < 1000) {
            oss << nanoseconds_ << " ns";
        } else if (nanoseconds_ < 1'000'000) {
            oss << toMicroseconds() << " us";
        } else if (nanoseconds_ < 1'000'000'000) {
            oss << toMilliseconds() << " ms";
        } else {
            oss << toSeconds() << " s";
        }
        return oss.str();
    }

private:
    int64_t nanoseconds_{0};
};

inline std::ostream& operator<<(std::ostream& os, const SimTime& t) {
    return os << t.toString();
}

// Standard MIL-STD-1553 Timing Constants and Calculation Utilities
namespace Mil1553Timing {

    // 1553 Bit Rate: 1.0 MHz (1 bit = 1.0 us = 1000 ns)
    inline constexpr int64_t BIT_TIME_NS = 1000;
    inline constexpr int64_t HALF_BIT_TIME_NS = 500;

    // Word Length: 20 bits = 20.0 us = 20,000 ns (3 bits Sync + 16 bits Data + 1 bit Parity)
    inline constexpr int64_t WORD_DURATION_NS = 20 * BIT_TIME_NS;

    // RT Response Time: MIL-STD-1553B requires 4.0 us to 12.0 us (nominal 8.0 us)
    inline constexpr int64_t MIN_RT_RESPONSE_TIME_NS = 4000;
    inline constexpr int64_t MAX_RT_RESPONSE_TIME_NS = 12000;
    inline constexpr int64_t NOMINAL_RT_RESPONSE_TIME_NS = 8000;

    // Intermessage Gap: MIL-STD-1553B minimum 4.0 us
    inline constexpr int64_t MIN_INTERMESSAGE_GAP_NS = 4000;

    // RT Response Timeout: BC waits up to 14.0 us before declaring No Response
    inline constexpr int64_t RESPONSE_TIMEOUT_NS = 14000;

    // Convenience SimTime constants
    inline const SimTime WordDuration = SimTime::fromNanoseconds(WORD_DURATION_NS);
    inline const SimTime NominalRtResponse = SimTime::fromNanoseconds(NOMINAL_RT_RESPONSE_TIME_NS);
    inline const SimTime MinIntermessageGap = SimTime::fromNanoseconds(MIN_INTERMESSAGE_GAP_NS);
    inline const SimTime ResponseTimeout = SimTime::fromNanoseconds(RESPONSE_TIMEOUT_NS);

    // Calculate total duration for BC -> RT (Receive) transaction:
    // T_total = T_cmd(20us) + (N_data * 20us) + T_rt_response + T_status(20us) + T_gap
    inline SimTime calculateBcToRtDuration(uint8_t wordCount,
                                           SimTime rtResponseTime = NominalRtResponse,
                                           SimTime intermessageGap = MinIntermessageGap) {
        int64_t totalNs = WORD_DURATION_NS; // Command Word
        totalNs += static_cast<int64_t>(wordCount) * WORD_DURATION_NS; // Data Words
        totalNs += rtResponseTime.toNanoseconds(); // RT Response Delay
        totalNs += WORD_DURATION_NS; // Status Word
        totalNs += intermessageGap.toNanoseconds(); // Intermessage Gap
        return SimTime::fromNanoseconds(totalNs);
    }

    // Calculate total duration for RT -> BC (Transmit) transaction:
    // T_total = T_cmd(20us) + T_rt_response + T_status(20us) + (N_data * 20us) + T_gap
    inline SimTime calculateRtToBcDuration(uint8_t wordCount,
                                           SimTime rtResponseTime = NominalRtResponse,
                                           SimTime intermessageGap = MinIntermessageGap) {
        int64_t totalNs = WORD_DURATION_NS; // Command Word
        totalNs += rtResponseTime.toNanoseconds(); // RT Response Delay
        totalNs += WORD_DURATION_NS; // Status Word
        totalNs += static_cast<int64_t>(wordCount) * WORD_DURATION_NS; // Data Words
        totalNs += intermessageGap.toNanoseconds(); // Intermessage Gap
        return SimTime::fromNanoseconds(totalNs);
    }

    // Calculate total duration for RT -> RT cross-terminal transaction:
    // T_total = T_rxCmd(20us) + T_txCmd(20us) + T_tx_response + T_txStatus(20us) + (N_data * 20us) + T_rx_response + T_rxStatus(20us) + T_gap
    inline SimTime calculateRtToRtDuration(uint8_t wordCount,
                                           SimTime txRtResponse = NominalRtResponse,
                                           SimTime rxRtResponse = NominalRtResponse,
                                           SimTime intermessageGap = MinIntermessageGap) {
        int64_t totalNs = 2 * WORD_DURATION_NS; // Rx Command + Tx Command
        totalNs += txRtResponse.toNanoseconds(); // Tx RT response delay
        totalNs += WORD_DURATION_NS; // Tx Status Word
        totalNs += static_cast<int64_t>(wordCount) * WORD_DURATION_NS; // Data Words
        totalNs += rxRtResponse.toNanoseconds(); // Rx RT response delay
        totalNs += WORD_DURATION_NS; // Rx Status Word
        totalNs += intermessageGap.toNanoseconds(); // Intermessage Gap
        return SimTime::fromNanoseconds(totalNs);
    }

    // Calculate total duration for a No-Response / Timeout transaction:
    // T_total = T_cmd(20us) + T_timeout(14us) + T_gap
    inline SimTime calculateTimeoutDuration(SimTime timeout = ResponseTimeout,
                                            SimTime intermessageGap = MinIntermessageGap) {
        int64_t totalNs = WORD_DURATION_NS + timeout.toNanoseconds() + intermessageGap.toNanoseconds();
        return SimTime::fromNanoseconds(totalNs);
    }

} // namespace Mil1553Timing
