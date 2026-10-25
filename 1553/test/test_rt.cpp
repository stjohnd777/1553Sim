//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include "mil1553/RT.hpp"

TEST(RTTest, Initialization) {
    RT rt(7);
    EXPECT_EQ(rt.getAddress(), 7);
}

TEST(RTTest, DefaultTransmitUnhandled) {
    RT rt(7);
    DataBlock1553 outData{};
    StatusWord status = rt.handleTransmit(1, 3, outData);
    EXPECT_EQ(status.rtAddress, 7);
    EXPECT_TRUE(status.messageError);
    EXPECT_FALSE(status.isOk());
}

TEST(RTTest, DefaultReceiveUnhandled) {
    RT rt(7);
    DataBlock1553 inData{};
    StatusWord status = rt.handleReceive(1, inData);
    EXPECT_EQ(status.rtAddress, 7);
    EXPECT_TRUE(status.messageError);
    EXPECT_FALSE(status.isOk());
}

TEST(RTTest, StepSimulationNoOp) {
    RT rt(7);
    EXPECT_NO_THROW(rt.stepSimulation(0.01));
}

TEST(RTTest, ExecuteHalReceiveMismatchedWaveformCount) {
    RT rt(7);
    Hal1553::Wire wire;
    CommandWord cmd{7, false, 1, 2}; // Expects 2 data words
    Hal1553::Manchester::Waveform statusWave;

    // Provide only 1 data waveform (mismatch)
    std::vector<Hal1553::Manchester::Waveform> waveforms;
    waveforms.push_back(Hal1553::Manchester::encode(1234, WordType::Data));

    bool ok = rt.executeHalReceive(wire, cmd, waveforms, statusWave);
    EXPECT_FALSE(ok);

    WordType swType{};
    u16 rawSw = 0;
    EXPECT_TRUE(Hal1553::Manchester::decode(statusWave, swType, rawSw));
    StatusWord sw = StatusWord::fromRaw(rawSw);
    EXPECT_TRUE(sw.messageError);
    EXPECT_FALSE(sw.isOk());
}
