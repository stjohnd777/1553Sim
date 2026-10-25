//
// Created by Daniel St. John St. John on 10/24/26.
//

#include <gtest/gtest.h>
#include "mil1553/BC.hpp"
#include "mil1553/RT.hpp"
#include "mil1553/SimTime.hpp"
#include "mil1553/types.hpp"

// Simple Mock RT for SimTime and Discrete-Event Testing
class SimTimeMockRT : public RT {
public:
    explicit SimTimeMockRT(uint8_t address) : RT(address) {}

    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override {
        StatusWord status{address_};
        if (subAddress == 1) {
            outData.count = wordCount;
            for (uint8_t i = 0; i < wordCount; ++i) {
                outData.words[i] = static_cast<uint16_t>(1000 + i);
            }
            return status;
        }
        status.messageError = true;
        return status;
    }

    StatusWord handleReceive(uint8_t subAddress, const DataBlock1553& inData) override {
        StatusWord status{address_};
        if (subAddress == 1) {
            lastReceived = inData;
            return status;
        }
        status.messageError = true;
        return status;
    }

    DataBlock1553 lastReceived{};
};

TEST(SimTimeTest, FactoryMethodsAndConversions) {
    auto t1 = SimTime::fromNanoseconds(1500);
    EXPECT_EQ(t1.toNanoseconds(), 1500);
    EXPECT_DOUBLE_EQ(t1.toMicroseconds(), 1.5);

    auto t2 = SimTime::fromMicroseconds(25.5);
    EXPECT_EQ(t2.toNanoseconds(), 25500);
    EXPECT_DOUBLE_EQ(t2.toMicroseconds(), 25.5);

    auto t3 = SimTime::fromMilliseconds(10.0);
    EXPECT_EQ(t3.toNanoseconds(), 10'000'000);
    EXPECT_DOUBLE_EQ(t3.toMilliseconds(), 10.0);
    EXPECT_DOUBLE_EQ(t3.toSeconds(), 0.010);

    auto t4 = SimTime::fromSeconds(2.0);
    EXPECT_EQ(t4.toNanoseconds(), 2'000'000'000);
    EXPECT_DOUBLE_EQ(t4.toSeconds(), 2.0);

    EXPECT_EQ(SimTime::zero().toNanoseconds(), 0);
    EXPECT_GT(SimTime::max().toNanoseconds(), 1'000'000'000'000LL);
}

TEST(SimTimeTest, ArithmeticAndComparisonOperators) {
    auto a = SimTime::fromMicroseconds(20.0);
    auto b = SimTime::fromMicroseconds(8.0);

    auto sum = a + b;
    EXPECT_DOUBLE_EQ(sum.toMicroseconds(), 28.0);

    auto diff = a - b;
    EXPECT_DOUBLE_EQ(diff.toMicroseconds(), 12.0);

    EXPECT_TRUE(a > b);
    EXPECT_TRUE(b < a);
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(b <= a);
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a == SimTime::fromMicroseconds(20.0));

    auto mult = b * 3;
    EXPECT_DOUBLE_EQ(mult.toMicroseconds(), 24.0);

    auto div = a / 2;
    EXPECT_DOUBLE_EQ(div.toMicroseconds(), 10.0);
}

TEST(SimTimeTest, StringFormatting) {
    EXPECT_EQ(SimTime::fromNanoseconds(500).toString(), "500 ns");
    EXPECT_EQ(SimTime::fromMicroseconds(20.5).toString(), "20.500 us");
    EXPECT_EQ(SimTime::fromMilliseconds(12.34).toString(), "12.340 ms");
    EXPECT_EQ(SimTime::fromSeconds(1.5).toString(), "1.500 s");
}

TEST(SimTimeTest, Mil1553TimingCalculations) {
    // BC -> RT with 3 data words:
    // Cmd (20us) + 3*Data (60us) + RT_response (8us) + Status (20us) + Gap (4us) = 112.0 us
    auto bcToRt = Mil1553Timing::calculateBcToRtDuration(3);
    EXPECT_DOUBLE_EQ(bcToRt.toMicroseconds(), 112.0);

    // RT -> BC with 2 data words:
    // Cmd (20us) + RT_response (8us) + Status (20us) + 2*Data (40us) + Gap (4us) = 92.0 us
    auto rtToBc = Mil1553Timing::calculateRtToBcDuration(2);
    EXPECT_DOUBLE_EQ(rtToBc.toMicroseconds(), 92.0);

    // RT -> RT with 1 data word:
    // RxCmd (20us) + TxCmd (20us) + TxResp (8us) + TxStatus (20us) + 1*Data (20us) + RxResp (8us) + RxStatus (20us) + Gap (4us) = 120.0 us
    auto rtToRt = Mil1553Timing::calculateRtToRtDuration(1);
    EXPECT_DOUBLE_EQ(rtToRt.toMicroseconds(), 120.0);

    // Timeout duration:
    // Cmd (20us) + Timeout (14us) + Gap (4us) = 38.0 us
    auto timeoutDur = Mil1553Timing::calculateTimeoutDuration();
    EXPECT_DOUBLE_EQ(timeoutDur.toMicroseconds(), 38.0);
}

TEST(SimTimeTest, AsynchronousSubmissionAndPolling) {
    BC bc;
    auto mock = std::make_shared<SimTimeMockRT>(3);
    bc.attachTerminal(mock);

    // Start simulation at t = 100.0 us
    bc.setCurrentTime(SimTime::fromMicroseconds(100.0));

    // Submit Transmit request for 3 words (expected duration = 112.0 us)
    // Scheduled start = 100.0 us, expected completion = 212.0 us
    TransactionId txId = bc.submitReceiveFromRT(3, 1, 3);
    EXPECT_EQ(txId, 1u);

    // 1. Immediately poll at t = 100.0 us (transaction is in-flight)
    auto poll1 = bc.poll(txId);
    ASSERT_TRUE(poll1.has_value());
    EXPECT_FALSE(poll1->isCompleted);
    EXPECT_DOUBLE_EQ(poll1->actualStart.toMicroseconds(), 100.0);
    EXPECT_DOUBLE_EQ(poll1->completionTime.toMicroseconds(), 212.0);
    EXPECT_DOUBLE_EQ(poll1->duration.toMicroseconds(), 112.0);
    // Payload should not be accessible yet
    EXPECT_EQ(poll1->result.data.count, 0);

    // 2. Advance time to t = 150.0 us (still in-flight)
    bc.advanceTime(SimTime::fromMicroseconds(50.0));
    EXPECT_DOUBLE_EQ(bc.getCurrentTime().toMicroseconds(), 150.0);
    EXPECT_TRUE(bc.isBusBusy());

    auto poll2 = bc.poll(txId);
    ASSERT_TRUE(poll2.has_value());
    EXPECT_FALSE(poll2->isCompleted);

    // 3. Advance time to t = 212.0 us (completed)
    bc.advanceTo(SimTime::fromMicroseconds(212.0));
    EXPECT_FALSE(bc.isBusBusy());

    auto poll3 = bc.poll(txId);
    ASSERT_TRUE(poll3.has_value());
    EXPECT_TRUE(poll3->isCompleted);
    EXPECT_TRUE(poll3->result.success);
    EXPECT_TRUE(poll3->result.status.isOk());
    EXPECT_EQ(poll3->result.data.count, 3);
    EXPECT_EQ(poll3->result.data.words[0], 1000);
    EXPECT_EQ(poll3->result.data.words[1], 1001);
    EXPECT_EQ(poll3->result.data.words[2], 1002);
}

TEST(SimTimeTest, BusContentionAndBackToBackSerialization) {
    BC bc;
    auto mock3 = std::make_shared<SimTimeMockRT>(3);
    auto mock4 = std::make_shared<SimTimeMockRT>(4);
    bc.attachTerminal(mock3);
    bc.attachTerminal(mock4);

    bc.setCurrentTime(SimTime::fromMicroseconds(0.0));

    // Submit Transaction 1 at t = 0 (BC -> RT 3 with 1 word: duration = 72.0 us, completes at 72.0 us)
    DataBlock1553 data1{};
    data1.count = 1;
    data1.words[0] = 42;
    TransactionId id1 = bc.submitTransmitToRT(3, 1, data1, SimTime::zero());

    // Submit Transaction 2 at t = 0 (RT 4 -> BC with 2 words: duration = 92.0 us)
    // Since bus is occupied until 72.0 us, Transaction 2 must automatically start at 72.0 us and complete at 164.0 us!
    TransactionId id2 = bc.submitReceiveFromRT(4, 1, 2, SimTime::zero());

    auto record1 = bc.poll(id1);
    auto record2 = bc.poll(id2);

    ASSERT_TRUE(record1.has_value());
    ASSERT_TRUE(record2.has_value());

    EXPECT_DOUBLE_EQ(record1->actualStart.toMicroseconds(), 0.0);
    EXPECT_DOUBLE_EQ(record1->completionTime.toMicroseconds(), 72.0);

    EXPECT_DOUBLE_EQ(record2->actualStart.toMicroseconds(), 72.0);
    EXPECT_DOUBLE_EQ(record2->completionTime.toMicroseconds(), 164.0);

    EXPECT_DOUBLE_EQ(bc.getBusAvailableTime().toMicroseconds(), 164.0);

    // Verify processPending batch retrieval
    auto completedAt100 = bc.processPending(SimTime::fromMicroseconds(100.0));
    EXPECT_EQ(completedAt100.size(), 1u);
    EXPECT_EQ(completedAt100[0].id, id1);

    auto completedAt200 = bc.processPending(SimTime::fromMicroseconds(200.0));
    EXPECT_EQ(completedAt200.size(), 2u);
}

TEST(SimTimeTest, UnattachedTerminalTimeoutDuration) {
    BC bc;
    bc.setCurrentTime(SimTime::fromMicroseconds(50.0));

    // Command unattached terminal RT 9 (timeout duration = 38.0 us)
    TransactionId id = bc.submitReceiveFromRT(9, 1, 1);
    auto record = bc.poll(id);
    ASSERT_TRUE(record.has_value());

    EXPECT_DOUBLE_EQ(record->actualStart.toMicroseconds(), 50.0);
    EXPECT_DOUBLE_EQ(record->duration.toMicroseconds(), 38.0);
    EXPECT_DOUBLE_EQ(record->completionTime.toMicroseconds(), 88.0);

    bc.advanceTo(SimTime::fromMicroseconds(88.0));
    auto completed = bc.poll(id);
    ASSERT_TRUE(completed.has_value());
    EXPECT_TRUE(completed->isCompleted);
    EXPECT_FALSE(completed->result.success);
    EXPECT_TRUE(completed->result.status.messageError);
}

TEST(SimTimeTest, FastTransactionModeFidelityToggle) {
    BC bc;
    auto mock = std::make_shared<SimTimeMockRT>(3);
    bc.attachTerminal(mock);

    // Fast Mode (Direct transaction without Manchester bit encoding)
    bc.setWaveformFidelity(false);
    EXPECT_FALSE(bc.isWaveformFidelityEnabled());

    DataBlock1553 data{};
    data.count = 2;
    data.words[0] = 777;
    data.words[1] = 888;

    auto resultFast = bc.execute(CommandWord{3, false, 1, 2}, data);
    EXPECT_TRUE(resultFast.success);
    EXPECT_EQ(mock->lastReceived.words[0], 777);
    EXPECT_EQ(mock->lastReceived.words[1], 888);

    // Waveform Mode (Full Manchester bit encoding and wire driving)
    bc.setWaveformFidelity(true);
    EXPECT_TRUE(bc.isWaveformFidelityEnabled());

    auto resultWave = bc.receiveFromRT(3, 1, 2);
    EXPECT_TRUE(resultWave.success);
    EXPECT_EQ(resultWave.data.count, 2);
    EXPECT_EQ(resultWave.data.words[0], 1000);
    EXPECT_EQ(resultWave.data.words[1], 1001);
}
