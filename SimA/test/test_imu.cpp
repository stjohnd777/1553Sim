//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include "mil1553/BC.hpp"
#include "sensor/ImuRT.hpp"

TEST(ImuRTTest, AttitudeTransmit) {
    ImuRT imu(3);
    imu.pitchDeg = 15.0;
    imu.rollDeg = -10.0;
    imu.yawDeg = 45.0;

    DataBlock1553 outData{};
    StatusWord status = imu.handleTransmit(1, 3, outData);

    EXPECT_TRUE(status.isOk());
    EXPECT_EQ(outData.count, 3);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(outData.words[0]), 180.0), 15.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(outData.words[1]), 180.0), -10.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(outData.words[2]), 180.0), 45.0, 0.1);
}

TEST(ImuRTTest, WordCountValidation) {
    ImuRT imu(3);
    DataBlock1553 outData{};

    // Requesting 1 word
    StatusWord status = imu.handleTransmit(1, 1, outData);
    EXPECT_TRUE(status.isOk());
    EXPECT_EQ(outData.count, 1);

    // Requesting 2 words
    status = imu.handleTransmit(1, 2, outData);
    EXPECT_TRUE(status.isOk());
    EXPECT_EQ(outData.count, 2);

    // Requesting 3 words
    status = imu.handleTransmit(1, 3, outData);
    EXPECT_TRUE(status.isOk());
    EXPECT_EQ(outData.count, 3);

    // Requesting more than 3 words (subaddress 1 only has 3 attitude words -> word count error)
    status = imu.handleTransmit(1, 10, outData);
    EXPECT_FALSE(status.isOk());
    EXPECT_TRUE(status.messageError);

    // Requesting 0 words (invalid word count -> word count error)
    status = imu.handleTransmit(1, 0, outData);
    EXPECT_FALSE(status.isOk());
    EXPECT_TRUE(status.messageError);
}

TEST(ImuRTTest, InvalidSubaddress) {
    ImuRT imu(3);
    DataBlock1553 outData{};
    StatusWord status = imu.handleTransmit(15, 3, outData);
    EXPECT_FALSE(status.isOk());
    EXPECT_TRUE(status.messageError);
}

TEST(ImuRTTest, StepSimulation) {
    ImuRT imu(3);
    imu.pitchDeg = 0.0;
    imu.pitchRate = 10.0; // 10 deg/sec

    imu.stepSimulation(0.1); // 100 ms
    EXPECT_NEAR(imu.pitchDeg, 1.0, 1e-4);
}

TEST(ImuRTTest, BcToImuCommandAndTiming) {
    BC bc;
    auto imu = std::make_shared<ImuRT>(3); // RT Address 3 (00011)
    imu->pitchDeg = 10.0;
    imu->rollDeg = -20.0;
    imu->yawDeg = 30.0;

    bc.attachTerminal(imu);

    // Command Word Construction:
    // RT Address: 00011 (3)
    // T/R:        1     (Transmit: RT -> BC)
    // Subaddress: 00001 (1: Attitude subaddress in ImuRT)
    // Word Count: 00011 (3 words: Pitch, Roll, Yaw)
    // Raw binary: 00011 1 00001 00011 -> 0x1C23
    CommandWord cmd{3, true, 1, 3};
    EXPECT_EQ(cmd.toRaw(), 0x1C23);

    // Initial state check
    EXPECT_EQ(bc.getTransceiver().clock.tickCount, 0u);
    EXPECT_EQ(imu->getTransceiver().clock.tickCount, 0u);

    // Execute BC read transaction across HAL
    auto result = bc.execute(cmd);

    // 1. Validate Transaction Success & Telemetry Integrity
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.status.isOk());
    EXPECT_EQ(result.status.rtAddress, 3);
    EXPECT_EQ(result.data.count, 3);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[0]), 180.0), 10.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[1]), 180.0), -20.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[2]), 180.0), 30.0, 0.1);

    // 2. Validate Physical Bus and Protocol Timing
    // - 1553 standard word = 20 bits (3-bit sync + 16-bit payload + 1-bit odd parity)
    // - At 1 Mbps bit rate, 1 bit = 1.0 us, 1 word = 20.0 us
    // - HAL clock = 2.0 MHz (2 half-bit ticks per bit -> 40 ticks per word)
    const uint64_t bcTicks = bc.getTransceiver().clock.tickCount;
    const uint64_t rtTicks = imu->getTransceiver().clock.tickCount;

    // BC transmits 1 Command Word = 20 bits = 40 ticks (20.0 us)
    EXPECT_EQ(bcTicks, 40u);
    const double bcTransmitDurationUs = bcTicks * 0.5; // 0.5 us per tick
    EXPECT_DOUBLE_EQ(bcTransmitDurationUs, 20.0);

    // RT transmits 1 Status Word + 3 Data Words = 4 words = 80 bits = 160 ticks (80.0 us)
    EXPECT_EQ(rtTicks, 160u);
    const double rtTransmitDurationUs = rtTicks * 0.5;
    EXPECT_DOUBLE_EQ(rtTransmitDurationUs, 80.0);

    // Total active transmission duration on 1553 bus = 5 words * 20.0 us = 100.0 us (200 ticks)
    const double totalBusActiveDurationUs = bcTransmitDurationUs + rtTransmitDurationUs;
    EXPECT_DOUBLE_EQ(totalBusActiveDurationUs, 100.0);

    // Bus wire returns to idle (0.0 V differential)
    EXPECT_DOUBLE_EQ(bc.getWire().differentialVoltage(), 0.0);
}

TEST(ImuRTTest, BcToRtWriteUnitsAndTiming) {
    BC bc;
    auto imu = std::make_shared<ImuRT>(3);
    bc.attachTerminal(imu);

    EXPECT_EQ(imu->units, ImuUnits::DEGREES);

    // BC writes unit configuration to IMU (Subaddress 2: Units = RADIANS)
    DataBlock1553 configData{};
    configData.count = 1;
    configData.words[0] = static_cast<uint16_t>(ImuUnits::RADIANS);

    auto result = bc.transmitToRT(3, 2, configData);

    // 1. Validate Transaction Success & State Update
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.status.isOk());
    EXPECT_EQ(result.status.rtAddress, 3);
    EXPECT_EQ(imu->units, ImuUnits::RADIANS);

    // 2. Validate Physical Bus and Protocol Timing
    // - BC transmits 1 Command Word + 1 Data Word = 2 words = 40 bits = 80 ticks (40.0 us)
    // - RT responds with 1 Status Word = 1 word = 20 bits = 40 ticks (20.0 us)
    const uint64_t bcTicks = bc.getTransceiver().clock.tickCount;
    const uint64_t rtTicks = imu->getTransceiver().clock.tickCount;

    EXPECT_EQ(bcTicks, 80u);
    const double bcTransmitDurationUs = bcTicks * 0.5;
    EXPECT_DOUBLE_EQ(bcTransmitDurationUs, 40.0);

    EXPECT_EQ(rtTicks, 40u);
    const double rtTransmitDurationUs = rtTicks * 0.5;
    EXPECT_DOUBLE_EQ(rtTransmitDurationUs, 20.0);

    const double totalBusActiveDurationUs = bcTransmitDurationUs + rtTransmitDurationUs;
    EXPECT_DOUBLE_EQ(totalBusActiveDurationUs, 60.0);
    EXPECT_DOUBLE_EQ(bc.getWire().differentialVoltage(), 0.0);
}

TEST(ImuRTTest, RtToBcReadAttitudeInSelectedUnits) {
    BC bc;
    auto imu = std::make_shared<ImuRT>(3);
    bc.attachTerminal(imu);

    // Configure IMU to Radians
    imu->units = ImuUnits::RADIANS;
    imu->pitchDeg = 45.0;  // pi / 4 rad
    imu->rollDeg = -30.0;  // -pi / 6 rad
    imu->yawDeg = 90.0;   // pi / 2 rad

    // BC issues RT->BC read command (Subaddress 1, 3 words)
    auto result = bc.receiveFromRT(3, 1, 3);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.status.isOk());
    EXPECT_EQ(result.data.count, 3);

    // In RADIANS mode, words are scaled against pi (3.14159...)
    const double maxVal = 3.14159265358979323846;
    double pRad = scaleFromWord(static_cast<int16_t>(result.data.words[0]), maxVal);
    double rRad = scaleFromWord(static_cast<int16_t>(result.data.words[1]), maxVal);
    double yRad = scaleFromWord(static_cast<int16_t>(result.data.words[2]), maxVal);

    EXPECT_NEAR(pRad * (180.0 / maxVal), 45.0, 0.1);
    EXPECT_NEAR(rRad * (180.0 / maxVal), -30.0, 0.1);
    EXPECT_NEAR(yRad * (180.0 / maxVal), 90.0, 0.1);
}

TEST(ImuRTTest, DualImuRtToRtCrossTransferAndTiming) {
    BC bc;
    auto imu1 = std::make_shared<ImuRT>(3); // Primary IMU (RT 3)
    auto imu2 = std::make_shared<ImuRT>(4); // Secondary IMU (RT 4)

    imu1->pitchDeg = 25.0;
    imu1->rollDeg = -15.0;
    imu1->yawDeg = 60.0;

    bc.attachTerminal(imu1);
    bc.attachTerminal(imu2);

    EXPECT_FALSE(imu2->hasCrossData);

    // Initial state check
    EXPECT_EQ(bc.getTransceiver().clock.tickCount, 0u);
    EXPECT_EQ(imu1->getTransceiver().clock.tickCount, 0u);
    EXPECT_EQ(imu2->getTransceiver().clock.tickCount, 0u);

    // Execute RT-to-RT Transfer:
    // BC commands IMU 1 (Tx RT 3, Subaddress 1: Attitude) to transmit directly
    // over the 1553 bus to IMU 2 (Rx RT 4, Subaddress 3: Cross-IMU attitude buffer)
    auto result = bc.transferRtToRt(/*rxAddress=*/4, /*rxSubAddress=*/3,
                                    /*txAddress=*/3, /*txSubAddress=*/1,
                                    /*wordCount=*/3);

    // 1. Validate Transaction Success & Telemetry Integrity across Terminals
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.txStatus.isOk());
    EXPECT_EQ(result.txStatus.rtAddress, 3);
    EXPECT_TRUE(result.rxStatus.isOk());
    EXPECT_EQ(result.rxStatus.rtAddress, 4);

    EXPECT_EQ(result.data.count, 3);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[0]), 180.0), 25.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[1]), 180.0), -15.0, 0.1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(result.data.words[2]), 180.0), 60.0, 0.1);

    // Validate that IMU 2 received and stored IMU 1's telemetry directly
    EXPECT_TRUE(imu2->hasCrossData);
    EXPECT_NEAR(imu2->crossPitchDeg, 25.0, 0.1);
    EXPECT_NEAR(imu2->crossRollDeg, -15.0, 0.1);
    EXPECT_NEAR(imu2->crossYawDeg, 60.0, 0.1);

    // 2. Validate Physical Bus and Protocol Timing for RT-to-RT Message:
    // - BC transmits 2 Command Words (Rx Cmd + Tx Cmd) = 2 * 20 bits = 40 bits = 80 ticks (40.0 us)
    // - Tx RT (IMU 1) transmits 1 Status Word + 3 Data Words = 4 * 20 bits = 80 bits = 160 ticks (80.0 us)
    // - Rx RT (IMU 2) transmits 1 Status Word = 1 * 20 bits = 20 bits = 40 ticks (20.0 us)
    // Total active bus duration = 7 words = 140 bits = 280 ticks (140.0 us)
    const uint64_t bcTicks = bc.getTransceiver().clock.tickCount;
    const uint64_t txTicks = imu1->getTransceiver().clock.tickCount;
    const uint64_t rxTicks = imu2->getTransceiver().clock.tickCount;

    EXPECT_EQ(bcTicks, 80u);
    const double bcTransmitDurationUs = bcTicks * 0.5;
    EXPECT_DOUBLE_EQ(bcTransmitDurationUs, 40.0);

    EXPECT_EQ(txTicks, 160u);
    const double txTransmitDurationUs = txTicks * 0.5;
    EXPECT_DOUBLE_EQ(txTransmitDurationUs, 80.0);

    EXPECT_EQ(rxTicks, 40u);
    const double rxTransmitDurationUs = rxTicks * 0.5;
    EXPECT_DOUBLE_EQ(rxTransmitDurationUs, 20.0);

    const double totalBusActiveDurationUs = bcTransmitDurationUs + txTransmitDurationUs + rxTransmitDurationUs;
    EXPECT_DOUBLE_EQ(totalBusActiveDurationUs, 140.0);

    // Bus wire returns to idle (0.0 V differential)
    EXPECT_DOUBLE_EQ(bc.getWire().differentialVoltage(), 0.0);
}
