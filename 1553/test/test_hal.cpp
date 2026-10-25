#include <gtest/gtest.h>
#include "hal/hal.hpp"

TEST(HalTest, ClockOperations) {
    Hal1553::Clock clock;
    EXPECT_DOUBLE_EQ(clock.frequencyHz, 1'000'000.0);
    EXPECT_DOUBLE_EQ(clock.halfBitPeriodNs(), 500.0);

    clock.tick();
    EXPECT_EQ(clock.tickCount, 1u);

    clock.setFrequency(2'000'000.0);
    EXPECT_DOUBLE_EQ(clock.halfBitPeriodNs(), 250.0);

    EXPECT_THROW(clock.setFrequency(0.0), std::invalid_argument);
    EXPECT_THROW(clock.setFrequency(-100.0), std::invalid_argument);
}

TEST(HalTest, WireOperations) {
    Hal1553::Wire wire;
    EXPECT_DOUBLE_EQ(wire.differentialVoltage(), 0.0);

    wire.drive(Level::Positive, 5.0);
    EXPECT_DOUBLE_EQ(wire.positiveVoltage, 2.5);
    EXPECT_DOUBLE_EQ(wire.negativeVoltage, -2.5);
    EXPECT_DOUBLE_EQ(wire.differentialVoltage(), 5.0);

    wire.drive(Level::Negative, 5.0);
    EXPECT_DOUBLE_EQ(wire.positiveVoltage, -2.5);
    EXPECT_DOUBLE_EQ(wire.negativeVoltage, 2.5);
    EXPECT_DOUBLE_EQ(wire.differentialVoltage(), -5.0);

    wire.release();
    EXPECT_DOUBLE_EQ(wire.differentialVoltage(), 0.0);
}

TEST(HalTest, ManchesterEncoding) {
    // Parity: count of 1s in 0x0000 is 0 (even), so oddParity should return true to make total odd
    EXPECT_TRUE(Hal1553::Manchester::oddParity(0x0000));
    // 0x0001 has 1 '1' (odd), so odd parity bit should be false
    EXPECT_FALSE(Hal1553::Manchester::oddParity(0x0001));

    auto cmdWave = Hal1553::Manchester::encode(0x0820, WordType::Command);
    EXPECT_EQ(cmdWave.size(), 40u);
    // Command sync: +++--- (first 3 positive, next 3 negative)
    EXPECT_EQ(cmdWave[0], Level::Positive);
    EXPECT_EQ(cmdWave[1], Level::Positive);
    EXPECT_EQ(cmdWave[2], Level::Positive);
    EXPECT_EQ(cmdWave[3], Level::Negative);
    EXPECT_EQ(cmdWave[4], Level::Negative);
    EXPECT_EQ(cmdWave[5], Level::Negative);

    auto dataWave = Hal1553::Manchester::encode(0x1234, WordType::Data);
    EXPECT_EQ(dataWave.size(), 40u);
    // Data sync: ---+++ (first 3 negative, next 3 positive)
    EXPECT_EQ(dataWave[0], Level::Negative);
    EXPECT_EQ(dataWave[1], Level::Negative);
    EXPECT_EQ(dataWave[2], Level::Negative);
    EXPECT_EQ(dataWave[3], Level::Positive);
    EXPECT_EQ(dataWave[4], Level::Positive);
    EXPECT_EQ(dataWave[5], Level::Positive);
}

TEST(HalTest, TransceiverOperations) {
    Hal1553::Transceiver xcvr;
    Hal1553::Wire wire;

    EXPECT_FALSE(xcvr.tx.enabled);
    xcvr.tx.enable();
    EXPECT_TRUE(xcvr.tx.enabled);
    xcvr.tx.disable();
    EXPECT_FALSE(xcvr.tx.enabled);

    EXPECT_TRUE(xcvr.rx.enabled);
    EXPECT_EQ(xcvr.rx.sample(wire), Level::Idle);

    wire.drive(Level::Positive, 5.0);
    EXPECT_EQ(xcvr.rx.sample(wire), Level::Positive);

    wire.drive(Level::Negative, 5.0);
    EXPECT_EQ(xcvr.rx.sample(wire), Level::Negative);

    xcvr.rx.enabled = false;
    EXPECT_EQ(xcvr.rx.sample(wire), Level::Idle);
}

TEST(HalTest, ManchesterDecoding) {
    // 1. Encode and Decode Command Word
    u16 cmdVal = 0x1C46;
    auto cmdWave = Hal1553::Manchester::encode(cmdVal, WordType::Command);
    WordType decodedType{};
    u16 decodedVal = 0;
    EXPECT_TRUE(Hal1553::Manchester::decode(cmdWave, decodedType, decodedVal));
    EXPECT_EQ(decodedType, WordType::Command);
    EXPECT_EQ(decodedVal, cmdVal);

    // 2. Encode and Decode Data Word
    u16 dataVal = 0xA55A;
    auto dataWave = Hal1553::Manchester::encode(dataVal, WordType::Data);
    EXPECT_TRUE(Hal1553::Manchester::decode(dataWave, decodedType, decodedVal));
    EXPECT_EQ(decodedType, WordType::Data);
    EXPECT_EQ(decodedVal, dataVal);

    // 3. Invalid Sync
    auto invalidSyncWave = cmdWave;
    invalidSyncWave[0] = Level::Idle;
    EXPECT_FALSE(Hal1553::Manchester::decode(invalidSyncWave, decodedType, decodedVal));

    // 4. Manchester Transition Violation
    auto manchesterViolationWave = cmdWave;
    manchesterViolationWave[6] = Level::Positive;
    manchesterViolationWave[7] = Level::Positive; // Positive-Positive is illegal in Manchester II
    EXPECT_FALSE(Hal1553::Manchester::decode(manchesterViolationWave, decodedType, decodedVal));

    // 5. Parity Violation
    auto parityViolationWave = cmdWave;
    // Invert parity bit levels
    parityViolationWave[38] = (cmdWave[38] == Level::Positive) ? Level::Negative : Level::Positive;
    parityViolationWave[39] = (cmdWave[39] == Level::Positive) ? Level::Negative : Level::Positive;
    EXPECT_FALSE(Hal1553::Manchester::decode(parityViolationWave, decodedType, decodedVal));
}

TEST(HalTest, DriveWordOnWire) {
    Hal1553::Transceiver txXcvr;
    Hal1553::Transceiver rxXcvr;
    Hal1553::Wire wire;

    Hal1553::Manchester::Waveform sampled{};
    txXcvr.tx.enable();
    auto wave = Hal1553::Manchester::encode(0x1234, WordType::Data);
    for (size_t i = 0; i < wave.size(); ++i) {
        txXcvr.clock.tick();
        wire.drive(wave[i]);
        sampled[i] = rxXcvr.rx.sample(wire);
    }
    wire.release();
    txXcvr.tx.disable();

    EXPECT_DOUBLE_EQ(wire.differentialVoltage(), 0.0);
    EXPECT_EQ(txXcvr.clock.tickCount, 40u);

    WordType outType{};
    u16 outVal = 0;
    EXPECT_TRUE(Hal1553::Manchester::decode(sampled, outType, outVal));
    EXPECT_EQ(outType, WordType::Data);
    EXPECT_EQ(outVal, 0x1234);
}
