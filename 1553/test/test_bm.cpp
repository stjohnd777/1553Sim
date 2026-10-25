//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include "mil1553/BC.hpp"
#include "mil1553/BM.hpp"

namespace {
class MockRT : public RT {
public:
    explicit MockRT(uint8_t address) : RT(address) {}
    StatusWord handleTransmit(uint8_t subAddress, uint8_t wordCount, DataBlock1553& outData) override {
        StatusWord status{address_};
        outData.count = wordCount;
        for (uint8_t i = 0; i < wordCount; ++i) {
            outData.words[i] = 100 + i;
        }
        return status;
    }
};
}

TEST(BMTest, RecordAndInspectTraffic) {
    BC bc;
    auto mock = std::make_shared<MockRT>(3);
    bc.attachTerminal(mock);

    auto bm = std::make_shared<BM>();
    bc.attachMonitor(bm);

    EXPECT_EQ(bm->getMessageCount(), 0);

    // Perform transaction
    auto res = bc.receiveFromRT(3, 1, 3);
    EXPECT_TRUE(res.success);

    EXPECT_EQ(bm->getMessageCount(), 1);
    const auto& log = bm->getLog();
    EXPECT_EQ(log[0].command.rtAddress, 3);
    EXPECT_EQ(log[0].command.subAddress, 1);
    EXPECT_TRUE(log[0].command.isTransmit);
    EXPECT_TRUE(log[0].success);
    EXPECT_EQ(log[0].data.count, 3);

    std::ostringstream oss;
    bm->printLog(oss);
    EXPECT_FALSE(oss.str().empty());

    bm->clear();
    EXPECT_EQ(bm->getMessageCount(), 0);
}
