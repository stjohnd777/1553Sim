//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <gtest/gtest.h>
#include "mil1553/types.hpp"

TEST(TypesTest, ScaleToWordNominal) {
    EXPECT_EQ(scaleToWord(0.0, 180.0), 0);
    EXPECT_EQ(scaleToWord(180.0, 180.0), 32767);
    EXPECT_EQ(scaleToWord(-180.0, 180.0), -32767);
}

TEST(TypesTest, ScaleToWordClamping) {
    EXPECT_EQ(scaleToWord(200.0, 180.0), 32767);
    EXPECT_EQ(scaleToWord(-200.0, 180.0), -32767);
}

TEST(TypesTest, ScaleToWordZeroMax) {
    EXPECT_EQ(scaleToWord(10.0, 0.0), 0);
}

TEST(TypesTest, ScaleFromWordNominal) {
    EXPECT_NEAR(scaleFromWord(0, 180.0), 0.0, 1e-3);
    EXPECT_NEAR(scaleFromWord(32767, 180.0), 180.0, 1e-3);
    EXPECT_NEAR(scaleFromWord(-32767, 180.0), -180.0, 1e-3);
}

TEST(TypesTest, StatusWordIsOk) {
    StatusWord sw;
    sw.rtAddress = 5;
    EXPECT_TRUE(sw.isOk());

    sw.messageError = true;
    EXPECT_FALSE(sw.isOk());

    sw.messageError = false;
    sw.busy = true;
    EXPECT_FALSE(sw.isOk());

    sw.busy = false;
    sw.subsystemFlag = true;
    EXPECT_FALSE(sw.isOk());
}

TEST(TypesTest, CommandWordRawConversion) {
    CommandWord cmd;
    cmd.rtAddress = 3;
    cmd.isTransmit = true;
    cmd.subAddress = 2;
    cmd.wordCount = 6;

    uint16_t raw = cmd.toRaw();
    EXPECT_EQ(raw, 0x1C46); // 00011 1 00010 00110

    CommandWord decoded = CommandWord::fromRaw(raw);
    EXPECT_EQ(decoded.rtAddress, 3);
    EXPECT_TRUE(decoded.isTransmit);
    EXPECT_EQ(decoded.subAddress, 2);
    EXPECT_EQ(decoded.wordCount, 6);

    // Test 32 words edge case (encoded as 0)
    CommandWord cmd32{5, false, 1, 32};
    uint16_t raw32 = cmd32.toRaw();
    EXPECT_EQ(raw32 & 0x1F, 0);
    CommandWord decoded32 = CommandWord::fromRaw(raw32);
    EXPECT_EQ(decoded32.wordCount, 32);
    EXPECT_EQ(decoded32.getEffectiveWordCount(), 32);
    EXPECT_FALSE(decoded32.isModeCode());
    EXPECT_FALSE(decoded32.isBroadcast());
}

TEST(TypesTest, ModeCodeAndBroadcastHandling) {
    // Mode code 0x01 (Synchronize without data) on subaddress 0
    CommandWord modeCmd0;
    modeCmd0.rtAddress = 4;
    modeCmd0.isTransmit = true;
    modeCmd0.subAddress = 0; // Mode code subaddress
    modeCmd0.wordCount = 1;  // Mode code 1 (Synchronize)

    EXPECT_TRUE(modeCmd0.isModeCode());
    EXPECT_EQ(modeCmd0.getModeCode(), 1);
    EXPECT_EQ(modeCmd0.getEffectiveWordCount(), 0); // Mode codes 0-15 have 0 data words

    uint16_t rawMode0 = modeCmd0.toRaw();
    CommandWord decodedMode0 = CommandWord::fromRaw(rawMode0);
    EXPECT_TRUE(decodedMode0.isModeCode());
    EXPECT_EQ(decodedMode0.subAddress, 0);
    EXPECT_EQ(decodedMode0.wordCount, 1);
    EXPECT_EQ(decodedMode0.getModeCode(), 1);

    // Mode code 0x11 (Synchronize with data) on subaddress 31
    CommandWord modeCmd31;
    modeCmd31.rtAddress = 8;
    modeCmd31.isTransmit = false;
    modeCmd31.subAddress = 31; // Mode code subaddress
    modeCmd31.wordCount = 17;  // Mode code 17 (0x11)

    EXPECT_TRUE(modeCmd31.isModeCode());
    EXPECT_EQ(modeCmd31.getModeCode(), 17);
    EXPECT_EQ(modeCmd31.getEffectiveWordCount(), 1); // Mode codes 16-31 have 1 data word

    uint16_t rawMode31 = modeCmd31.toRaw();
    CommandWord decodedMode31 = CommandWord::fromRaw(rawMode31);
    EXPECT_TRUE(decodedMode31.isModeCode());
    EXPECT_EQ(decodedMode31.subAddress, 31);
    EXPECT_EQ(decodedMode31.wordCount, 17);

    // Mode code 0 on subaddress 0 should NOT be converted to 32 words
    CommandWord modeZero{2, true, 0, 0};
    EXPECT_TRUE(modeZero.isModeCode());
    uint16_t rawZero = modeZero.toRaw();
    CommandWord decodedZero = CommandWord::fromRaw(rawZero);
    EXPECT_EQ(decodedZero.wordCount, 0);
    EXPECT_EQ(decodedZero.getModeCode(), 0);

    // Broadcast address 31
    CommandWord bcastCmd{31, false, 1, 10};
    EXPECT_TRUE(bcastCmd.isBroadcast());
    EXPECT_FALSE(bcastCmd.isModeCode());
    EXPECT_EQ(bcastCmd.getEffectiveWordCount(), 10);
}

TEST(TypesTest, DataBlockBoundsAndAccessors) {
    DataBlock1553 block{};
    EXPECT_TRUE(block.isValid());
    EXPECT_EQ(block.count, 0);

    // Safe setWord
    EXPECT_TRUE(block.setWord(0, 100));
    EXPECT_EQ(block.count, 1);
    EXPECT_EQ(block.getWord(0), 100);

    EXPECT_TRUE(block.setWord(31, 999));
    EXPECT_EQ(block.count, 32);
    EXPECT_EQ(block.getWord(31), 999);

    // Out of bounds setWord & getWord
    EXPECT_FALSE(block.setWord(32, 500));
    EXPECT_EQ(block.getWord(32), 0);

    // Invalidation
    block.count = 33;
    EXPECT_FALSE(block.isValid());
}

TEST(TypesTest, StatusWordRawConversion) {
    StatusWord sw;
    sw.rtAddress = 7;
    sw.messageError = true;
    sw.serviceRequest = true;
    sw.busy = false;
    sw.subsystemFlag = true;

    uint16_t raw = sw.toRaw();
    StatusWord decoded = StatusWord::fromRaw(raw);
    EXPECT_EQ(decoded.rtAddress, 7);
    EXPECT_TRUE(decoded.messageError);
    EXPECT_TRUE(decoded.serviceRequest);
    EXPECT_FALSE(decoded.busy);
    EXPECT_TRUE(decoded.subsystemFlag);
    EXPECT_FALSE(decoded.isOk());
}
