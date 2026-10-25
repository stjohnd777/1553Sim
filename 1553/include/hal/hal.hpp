
#pragma once

#include <array>
#include <cstdint>

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u64 = std::uint64_t;

// One half-bit interval = 500 ns nominally.
enum class Level : std::int8_t {
    Negative = -1,
    Idle     =  0,
    Positive =  1
};

enum class Channel : u8 {
    A, B
};

enum class WordType : u8 {
    Command,
    Status,
    Data
};

class Hal1553 {
public:
    // A local oscillator, not the global Trick clock.
    struct Clock {
        double frequencyHz{1'000'000.0};
        u64 tickCount{};

        void setFrequency(double hz);
        // One half-bit tick.
        double halfBitPeriodNs() const;
        void tick();
        void reset();
    };

    // A nominal differential voltage pair.
    struct Wire {
        double positiveVoltage{};
        double negativeVoltage{};

        double differentialVoltage() const;
        void drive(Level level, double amplitude = 5.0);
        void release();
        void clear();
    };

    // 20 bit-times, each split into two cells.
    struct Manchester {
        static constexpr int CellsPerWord = 40;
        using Waveform = std::array<Level, CellsPerWord>;

        static bool oddParity(u16 value);
        static Waveform encode(u16 value, WordType type);
        static bool decode(const Waveform& wave, WordType& outType, u16& outValue);
    };

    // One transceiver attaches to one physical channel.
    struct Transceiver {
        struct TX {
            bool enabled{false};
            void enable();
            void disable();
        };
        struct RX {
            bool enabled{true};
            Level sample(const Wire& wire) const;
        };
        TX tx;
        RX rx;
        Clock clock;
    };

    // Drive a word onto the physical wire via transmitter
    static void driveWord(Transceiver& txXcvr, Wire& wire, u16 value, WordType type);
};
