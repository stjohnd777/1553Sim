//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include <memory>
#include "mil1553/BC.hpp"

// Simple mock RT for testing
class MockRT : public RT {
public:
    explicit MockRT(uint8_t address) : RT(address) {}

    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override {
        StatusWord status{address_};
        if (subAddress == 2) {
            outData.count = wordCount;
            for (uint8_t i = 0; i < wordCount; ++i) {
                outData.words[i] = 100 + i;
            }
            return status;
        }
        status.messageError = true;
        return status;
    }

    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override {
        StatusWord status{address_};
        if (subAddress == 2) {
            receivedData = inData;
            return status;
        }
        status.messageError = true;
        return status;
    }

    DataBlock1553 receivedData{};
};

TEST(BCTest, AttachAndGetTerminal) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    bc.attachTerminal(mock);

    EXPECT_EQ(bc.getTerminal(3), mock);
    EXPECT_EQ(bc.getTerminal(4), nullptr);
}

TEST(BCTest, ExecuteUnattachedTerminalFails) {
    BC bc;
    CommandWord cmd{9, true, 1, 1};
    auto result = bc.execute(cmd);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.status.messageError);
    EXPECT_EQ(result.status.rtAddress, 9);
}

TEST(BCTest, TransmitAndReceiveExecution) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    bc.attachTerminal(mock);

    // Test Transmit (RT -> BC)
    auto txRes = bc.receiveFromRT(3, 2, 2);
    EXPECT_TRUE(txRes.success);
    EXPECT_EQ(txRes.data.count, 2);
    EXPECT_EQ(txRes.data.words[0], 100);
    EXPECT_EQ(txRes.data.words[1], 101);

    // Test Receive (BC -> RT)
    DataBlock1553 sendData{};
    sendData.count = 2;
    sendData.words[0] = 555;
    sendData.words[1] = 666;

    auto rxRes = bc.transmitToRT(3, 2, sendData);
    EXPECT_TRUE(rxRes.success);
    EXPECT_EQ(mock->receivedData.count, 2);
    EXPECT_EQ(mock->receivedData.words[0], 555);
    EXPECT_EQ(mock->receivedData.words[1], 666);
}

TEST(BCTest, HalClockAndWireExecution) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    bc.attachTerminal(mock);

    // Initial clock tick counts
    EXPECT_EQ(bc.getTransceiver().clock.tickCount, 0u);
    EXPECT_EQ(mock->getTransceiver().clock.tickCount, 0u);

    // Transmit: BC sends Command (40 ticks), RT sends Status + 2 Data words (3 * 40 = 120 ticks)
    auto txRes = bc.receiveFromRT(3, 2, 2);
    EXPECT_TRUE(txRes.success);
    EXPECT_EQ(bc.getTransceiver().clock.tickCount, 40u);
    EXPECT_EQ(mock->getTransceiver().clock.tickCount, 120u);

    // Wire differential voltage should return to 0 (idle) after transaction
    EXPECT_DOUBLE_EQ(bc.getWire().differentialVoltage(), 0.0);
}

TEST(BCTest, HalTransceiverRxDisableFails) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    bc.attachTerminal(mock);

    // Disable BC transceiver RX
    bc.getTransceiver().rx.enabled = false;
    auto txRes = bc.receiveFromRT(3, 2, 2);
    EXPECT_FALSE(txRes.success);
    EXPECT_TRUE(txRes.status.messageError);
}

TEST(BCTest, RtToRtExecution) {
    BC bc;
    auto mockTx = std::make_shared<MockRT>(3);
    auto mockRx = std::make_shared<MockRT>(4);
    bc.attachTerminal(mockTx);
    bc.attachTerminal(mockRx);

    auto result = bc.transferRtToRt(/*rxAddress=*/4, /*rxSub=*/2, /*txAddress=*/3, /*txSub=*/2, /*wordCount=*/2);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.txStatus.isOk());
    EXPECT_TRUE(result.rxStatus.isOk());
    EXPECT_EQ(result.txStatus.rtAddress, 3);
    EXPECT_EQ(result.rxStatus.rtAddress, 4);

    EXPECT_EQ(result.data.count, 2);
    EXPECT_EQ(result.data.words[0], 100);
    EXPECT_EQ(result.data.words[1], 101);

    EXPECT_EQ(mockRx->receivedData.count, 2);
    EXPECT_EQ(mockRx->receivedData.words[0], 100);
    EXPECT_EQ(mockRx->receivedData.words[1], 101);
}

TEST(BCTest, BroadcastAndInvalidAddressRejection) {
    BC bc;
    auto mock31 = std::make_shared<MockRT>(31);
    bc.attachTerminal(mock31);

    // Terminal with broadcast address 31 must not be attached as a normal terminal
    EXPECT_EQ(bc.getTerminal(31), nullptr);

    // Executing command to address 31 or >30 must fail with message error
    CommandWord bcastCmd{31, false, 1, 1};
    DataBlock1553 data{};
    data.count = 1;
    data.words[0] = 42;

    auto bcastRes = bc.execute(bcastCmd, data);
    EXPECT_FALSE(bcastRes.success);
    EXPECT_TRUE(bcastRes.status.messageError);

    CommandWord invalidAddrCmd{32, false, 1, 1};
    auto invalidRes = bc.execute(invalidAddrCmd, data);
    EXPECT_FALSE(invalidRes.success);
    EXPECT_TRUE(invalidRes.status.messageError);
}

// Mock RT that deliberately responds with mismatched word count to simulate faulty/incompliant device
class FaultyWordCountRT : public RT {
public:
    explicit FaultyWordCountRT(uint8_t address) : RT(address) {}

    StatusWord handleTransmit(uint8_t subAddress, uint8_t /*wordCount*/, DataBlock1553& outData) override {
        StatusWord status{address_};
        if (subAddress == 2) {
            // Deliberately returns 2 words regardless of requested count
            outData.count = 2;
            outData.words[0] = 111;
            outData.words[1] = 222;
            return status;
        }
        status.messageError = true;
        return status;
    }
};

TEST(BCTest, ExactWordCountMismatchDetection) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    auto faulty = std::make_shared<FaultyWordCountRT>(4);
    bc.attachTerminal(mock);
    bc.attachTerminal(faulty);

    // 1. BC commands Receive of 3 words (BC -> RT), but supplies only 1 word in DataBlock
    CommandWord rxCmd3{3, false, 2, 3}; // Command specifies 3 words
    DataBlock1553 shortData{};
    shortData.count = 1; // Supplied only 1 word
    shortData.words[0] = 999;

    auto rxMismatchRes = bc.execute(rxCmd3, shortData);
    EXPECT_FALSE(rxMismatchRes.success);
    EXPECT_TRUE(rxMismatchRes.status.messageError);

    // 2. BC commands Transmit of 6 words (RT -> BC), but RT returns only 2 words
    CommandWord txCmd6{4, true, 2, 6}; // Command specifies 6 words
    auto txMismatchRes = bc.execute(txCmd6);
    EXPECT_FALSE(txMismatchRes.success);
    EXPECT_TRUE(txMismatchRes.status.messageError);

    // 3. BC commands Transmit of 2 words to faulty RT (which returns 2 words) -> Success
    CommandWord txCmd2{4, true, 2, 2};
    auto txMatchRes = bc.execute(txCmd2);
    EXPECT_TRUE(txMatchRes.success);
    EXPECT_TRUE(txMatchRes.status.isOk());
    EXPECT_EQ(txMatchRes.data.count, 2);
}
