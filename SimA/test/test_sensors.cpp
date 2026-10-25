//
// Created by Daniel St. John St. John on 10/25/26.
//

#include <gtest/gtest.h>
#include <memory>
#include "mil1553/BC.hpp"
#include "sensor/TemperatureSensorRT.hpp"
#include "sensor/PressureSensorRT.hpp"

class SensorRTTest : public ::testing::Test {
protected:
    BC bus;
    std::shared_ptr<TemperatureSensorRT> tempSensor;
    std::shared_ptr<PressureSensorRT> pressSensor;

    void SetUp() override {
        tempSensor = std::make_shared<TemperatureSensorRT>(6);
        pressSensor = std::make_shared<PressureSensorRT>(8);

        bus.attachTerminal(tempSensor);
        bus.attachTerminal(pressSensor);
    }
};

TEST_F(SensorRTTest, TemperatureSensorReadAndStep) {
    tempSensor->temperatureCelsius = 450.5;
    tempSensor->tempRate = 10.0; // 10 deg C / sec

    auto result = bus.receiveFromRT(6, 1, 1);
    ASSERT_TRUE(result.success);
    ASSERT_TRUE(result.status.isOk());
    EXPECT_EQ(result.data.count, 1);

    double readTemp = scaleFromWord(static_cast<int16_t>(result.data.words[0]), 1000.0);
    EXPECT_NEAR(readTemp, 450.5, 0.1);

    // Step simulation by 0.5s
    tempSensor->stepSimulation(0.5);
    EXPECT_NEAR(tempSensor->temperatureCelsius, 455.5, 0.001);

    auto result2 = bus.receiveFromRT(6, 1, 1);
    ASSERT_TRUE(result2.success);
    double readTemp2 = scaleFromWord(static_cast<int16_t>(result2.data.words[0]), 1000.0);
    EXPECT_NEAR(readTemp2, 455.5, 0.1);
}

TEST_F(SensorRTTest, PressureSensorReadAndStep) {
    pressSensor->pressureBar = 80.25;
    pressSensor->pressureRate = -5.0; // -5 bar / sec

    auto result = bus.receiveFromRT(8, 1, 1);
    ASSERT_TRUE(result.success);
    ASSERT_TRUE(result.status.isOk());
    EXPECT_EQ(result.data.count, 1);

    double readPress = scaleFromWord(static_cast<int16_t>(result.data.words[0]), 250.0);
    EXPECT_NEAR(readPress, 80.25, 0.05);

    // Step simulation by 1.0s
    pressSensor->stepSimulation(1.0);
    EXPECT_NEAR(pressSensor->pressureBar, 75.25, 0.001);

    auto result2 = bus.receiveFromRT(8, 1, 1);
    ASSERT_TRUE(result2.success);
    double readPress2 = scaleFromWord(static_cast<int16_t>(result2.data.words[0]), 250.0);
    EXPECT_NEAR(readPress2, 75.25, 0.05);
}

TEST_F(SensorRTTest, InvalidSubaddressOrWordCountRejection) {
    // Word count != 1 for temperature sensor must be rejected
    auto invalidWordCount = bus.receiveFromRT(6, 1, 2);
    EXPECT_FALSE(invalidWordCount.success);
    EXPECT_TRUE(invalidWordCount.status.messageError);

    // Invalid subaddress must be rejected
    auto invalidSubaddr = bus.receiveFromRT(8, 5, 1);
    EXPECT_FALSE(invalidSubaddr.success);
    EXPECT_TRUE(invalidSubaddr.status.messageError);
}
