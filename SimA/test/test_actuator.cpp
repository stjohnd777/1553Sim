//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include "motor/ActuatorRT.hpp"

TEST(ActuatorRTTest, ReceivePositionCommand) {
    ActuatorRT act(5);
    DataBlock1553 cmd{};
    cmd.count = 1;
    cmd.words[0] = static_cast<uint16_t>(scaleToWord(22.5, 45.0)); // 50% deflection = 22.5 deg

    StatusWord status = act.handleReceive(1, cmd);
    EXPECT_TRUE(status.isOk());
    EXPECT_NEAR(act.commandedDeflectionDeg, 22.5, 0.1);
    EXPECT_NEAR(act.actualDeflectionDeg, 22.5, 0.1);
}

TEST(ActuatorRTTest, TransmitTelemetry) {
    ActuatorRT act(5);
    act.actualDeflectionDeg = -15.0;

    DataBlock1553 telem{};
    StatusWord status = act.handleTransmit(1, 1, telem);
    EXPECT_TRUE(status.isOk());
    EXPECT_EQ(telem.count, 1);
    EXPECT_NEAR(scaleFromWord(static_cast<int16_t>(telem.words[0]), 45.0), -15.0, 0.1);
}

TEST(ActuatorRTTest, InvalidSubaddress) {
    ActuatorRT act(5);
    DataBlock1553 cmd{};
    cmd.count = 1;
    cmd.words[0] = 1234;

    StatusWord rxStatus = act.handleReceive(2, cmd);
    EXPECT_FALSE(rxStatus.isOk());
    EXPECT_TRUE(rxStatus.messageError);

    DataBlock1553 telem{};
    StatusWord txStatus = act.handleTransmit(2, 1, telem);
    EXPECT_FALSE(txStatus.isOk());
    EXPECT_TRUE(txStatus.messageError);
}
