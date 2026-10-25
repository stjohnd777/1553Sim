//
// Created by Daniel St. John St. John on 10/24/26.
//

#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include "mil1553/BC.hpp"
#include "sensor/ImuRT.hpp"
#include "motor/ActuatorRT.hpp"

// Test fixture for Rocket Flight Computer controlling TVC vectoring actuators via 1553 bus
class RocketFlightComputerBusTest : public ::testing::Test {
protected:
    BC busController; // Flight Computer's 1553 Bus Controller interface

    // Redundant Dual IMU Sensors
    std::shared_ptr<ImuRT> imuPrimary;    // RT Address 3: Primary Navigation IMU
    std::shared_ptr<ImuRT> imuSecondary;  // RT Address 4: Backup / Redundant IMU

    // Dual Thrust Vector Control (TVC) Gimbal Actuators
    std::shared_ptr<ActuatorRT> actuatorPitch; // RT Address 5: Pitch Gimbal Actuator
    std::shared_ptr<ActuatorRT> actuatorYaw;   // RT Address 6: Yaw Gimbal Actuator

    void SetUp() override {
        // Instantiate redundant sensors and TVC actuators
        imuPrimary = std::make_shared<ImuRT>(3);
        imuSecondary = std::make_shared<ImuRT>(4);
        actuatorPitch = std::make_shared<ActuatorRT>(5);
        actuatorYaw = std::make_shared<ActuatorRT>(6);

        // Attach all 4 Remote Terminals to the MIL-STD-1553 bus
        busController.attachTerminal(imuPrimary);
        busController.attachTerminal(imuSecondary);
        busController.attachTerminal(actuatorPitch);
        busController.attachTerminal(actuatorYaw);
    }
};

TEST_F(RocketFlightComputerBusTest, BusSetupAndTerminalRegistration) {
    // Verify that all 4 devices are properly mapped to their assigned RT addresses
    EXPECT_EQ(busController.getTerminal(3), imuPrimary);
    EXPECT_EQ(busController.getTerminal(4), imuSecondary);
    EXPECT_EQ(busController.getTerminal(5), actuatorPitch);
    EXPECT_EQ(busController.getTerminal(6), actuatorYaw);

    // Verify unassigned RT slots remain empty
    EXPECT_EQ(busController.getTerminal(0), nullptr);
    EXPECT_EQ(busController.getTerminal(1), nullptr);
    EXPECT_EQ(busController.getTerminal(2), nullptr);
    EXPECT_EQ(busController.getTerminal(7), nullptr);
}

TEST_F(RocketFlightComputerBusTest, ReadDualImuTelemetry) {
    // Set simulated vehicle attitude on primary and backup IMUs
    imuPrimary->pitchDeg = 4.2;
    imuPrimary->rollDeg = 0.5;
    imuPrimary->yawDeg = -2.8;

    imuSecondary->pitchDeg = 4.3; // Slight sensor variance
    imuSecondary->rollDeg = 0.48;
    imuSecondary->yawDeg = -2.75;

    // Flight Computer reads Primary IMU (RT 3, Subaddress 1, 3 words)
    auto imu1Result = busController.receiveFromRT(3, 1, 3);
    EXPECT_TRUE(imu1Result.success);
    EXPECT_TRUE(imu1Result.status.isOk());
    EXPECT_EQ(imu1Result.status.rtAddress, 3);
    EXPECT_EQ(imu1Result.data.count, 3);

    double pitch1 = scaleFromWord(static_cast<int16_t>(imu1Result.data.words[0]), 180.0);
    double roll1  = scaleFromWord(static_cast<int16_t>(imu1Result.data.words[1]), 180.0);
    double yaw1   = scaleFromWord(static_cast<int16_t>(imu1Result.data.words[2]), 180.0);

    EXPECT_NEAR(pitch1, 4.2, 0.05);
    EXPECT_NEAR(roll1, 0.5, 0.05);
    EXPECT_NEAR(yaw1, -2.8, 0.05);

    // Flight Computer reads Secondary IMU (RT 4, Subaddress 1, 3 words)
    auto imu2Result = busController.receiveFromRT(4, 1, 3);
    EXPECT_TRUE(imu2Result.success);
    EXPECT_TRUE(imu2Result.status.isOk());
    EXPECT_EQ(imu2Result.status.rtAddress, 4);
    EXPECT_EQ(imu2Result.data.count, 3);

    double pitch2 = scaleFromWord(static_cast<int16_t>(imu2Result.data.words[0]), 180.0);
    double roll2  = scaleFromWord(static_cast<int16_t>(imu2Result.data.words[1]), 180.0);
    double yaw2   = scaleFromWord(static_cast<int16_t>(imu2Result.data.words[2]), 180.0);

    EXPECT_NEAR(pitch2, 4.3, 0.05);
    EXPECT_NEAR(roll2, 0.48, 0.05);
    EXPECT_NEAR(yaw2, -2.75, 0.05);
}

TEST_F(RocketFlightComputerBusTest, CommandDualTvcActuators) {
    // Desired gimbal deflections computed by Flight Computer
    const double targetPitchDeflectionDeg = 3.75;  // +3.75 deg pitch gimbal
    const double targetYawDeflectionDeg = -2.50;  // -2.50 deg yaw gimbal

    // 1. Flight Computer writes command to Pitch Actuator (RT 5, Subaddress 1, 1 word)
    DataBlock1553 pitchCmdBlock{};
    pitchCmdBlock.count = 1;
    pitchCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(targetPitchDeflectionDeg, 45.0));

    auto pitchTxResult = busController.transmitToRT(5, 1, pitchCmdBlock);
    EXPECT_TRUE(pitchTxResult.success);
    EXPECT_TRUE(pitchTxResult.status.isOk());
    EXPECT_EQ(pitchTxResult.status.rtAddress, 5);
    EXPECT_NEAR(actuatorPitch->commandedDeflectionDeg, targetPitchDeflectionDeg, 0.05);
    EXPECT_NEAR(actuatorPitch->actualDeflectionDeg, targetPitchDeflectionDeg, 0.05);

    // 2. Flight Computer writes command to Yaw Actuator (RT 6, Subaddress 1, 1 word)
    DataBlock1553 yawCmdBlock{};
    yawCmdBlock.count = 1;
    yawCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(targetYawDeflectionDeg, 45.0));

    auto yawTxResult = busController.transmitToRT(6, 1, yawCmdBlock);
    EXPECT_TRUE(yawTxResult.success);
    EXPECT_TRUE(yawTxResult.status.isOk());
    EXPECT_EQ(yawTxResult.status.rtAddress, 6);
    EXPECT_NEAR(actuatorYaw->commandedDeflectionDeg, targetYawDeflectionDeg, 0.05);
    EXPECT_NEAR(actuatorYaw->actualDeflectionDeg, targetYawDeflectionDeg, 0.05);

    // 3. Flight Computer reads back telemetry from Pitch Actuator (RT 5, Subaddress 1, 1 word)
    auto pitchTelemResult = busController.receiveFromRT(5, 1, 1);
    EXPECT_TRUE(pitchTelemResult.success);
    EXPECT_TRUE(pitchTelemResult.status.isOk());
    EXPECT_EQ(pitchTelemResult.data.count, 1);
    double pitchReadback = scaleFromWord(static_cast<int16_t>(pitchTelemResult.data.words[0]), 45.0);
    EXPECT_NEAR(pitchReadback, targetPitchDeflectionDeg, 0.05);

    // 4. Flight Computer reads back telemetry from Yaw Actuator (RT 6, Subaddress 1, 1 word)
    auto yawTelemResult = busController.receiveFromRT(6, 1, 1);
    EXPECT_TRUE(yawTelemResult.success);
    EXPECT_TRUE(yawTelemResult.status.isOk());
    EXPECT_EQ(yawTelemResult.data.count, 1);
    double yawReadback = scaleFromWord(static_cast<int16_t>(yawTelemResult.data.words[0]), 45.0);
    EXPECT_NEAR(yawReadback, targetYawDeflectionDeg, 0.05);
}

TEST_F(RocketFlightComputerBusTest, SingleCycleSensorsToActuatorsWithTiming) {
    // 1. Initial State: rocket has pitch and yaw attitude error
    imuPrimary->pitchDeg = 5.0;
    imuPrimary->yawDeg = -3.0;
    imuSecondary->pitchDeg = 5.1;
    imuSecondary->yawDeg = -2.9;

    // Reset clocks for precise cycle timing verification
    busController.getTransceiver().clock.reset();
    imuPrimary->getTransceiver().clock.reset();
    imuSecondary->getTransceiver().clock.reset();
    actuatorPitch->getTransceiver().clock.reset();
    actuatorYaw->getTransceiver().clock.reset();

    // 2. Read IMU 1 (RT 3, Subaddress 1, 3 words)
    auto imu1Result = busController.receiveFromRT(3, 1, 3);
    EXPECT_TRUE(imu1Result.success);
    double pitch1 = scaleFromWord(static_cast<int16_t>(imu1Result.data.words[0]), 180.0);
    double yaw1   = scaleFromWord(static_cast<int16_t>(imu1Result.data.words[2]), 180.0);

    // 3. Read IMU 2 (RT 4, Subaddress 1, 3 words)
    auto imu2Result = busController.receiveFromRT(4, 1, 3);
    EXPECT_TRUE(imu2Result.success);
    double pitch2 = scaleFromWord(static_cast<int16_t>(imu2Result.data.words[0]), 180.0);
    double yaw2   = scaleFromWord(static_cast<int16_t>(imu2Result.data.words[2]), 180.0);

    // 4. Flight Computer Sensor Fusion & Vectoring Guidance Calculation
    double fusedPitch = (pitch1 + pitch2) / 2.0;
    double fusedYaw   = (yaw1 + yaw2) / 2.0;

    // Proportional TVC gimbal steering law: vectoring counters vehicle attitude error
    double targetPitchGimbal = std::clamp(-1.2 * fusedPitch, -15.0, 15.0);
    double targetYawGimbal   = std::clamp(-1.2 * fusedYaw,   -15.0, 15.0);

    // 5. Command Pitch Gimbal Actuator (RT 5, Subaddress 1, 1 word)
    DataBlock1553 pitchCmdBlock{};
    pitchCmdBlock.count = 1;
    pitchCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(targetPitchGimbal, 45.0));
    auto pitchTxResult = busController.transmitToRT(5, 1, pitchCmdBlock);
    EXPECT_TRUE(pitchTxResult.success);

    // 6. Command Yaw Gimbal Actuator (RT 6, Subaddress 1, 1 word)
    DataBlock1553 yawCmdBlock{};
    yawCmdBlock.count = 1;
    yawCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(targetYawGimbal, 45.0));
    auto yawTxResult = busController.transmitToRT(6, 1, yawCmdBlock);
    EXPECT_TRUE(yawTxResult.success);

    // 7. Verify Actuator Deflection Execution
    EXPECT_NEAR(actuatorPitch->actualDeflectionDeg, targetPitchGimbal, 0.05);
    EXPECT_NEAR(actuatorYaw->actualDeflectionDeg, targetYawGimbal, 0.05);

    // 8. Validate 1553 Bus Physical Protocol Timing for Full Sensor-to-Actuator Vectoring Cycle:
    // - 2x RT Read transactions (IMU 1 & IMU 2):
    //     BC sends 1 Cmd Word (40 ticks = 20 us) each -> 2 * 40 = 80 ticks
    //     Each IMU sends 1 Status + 3 Data = 4 Words (160 ticks = 80 us) -> 2 * 160 = 320 ticks
    // - 2x RT Write transactions (Pitch & Yaw Actuators):
    //     BC sends 1 Cmd Word + 1 Data Word = 2 Words (80 ticks = 40 us) each -> 2 * 80 = 160 ticks
    //     Each Actuator sends 1 Status = 1 Word (40 ticks = 20 us) -> 2 * 40 = 80 ticks
    // Total BC Transmit ticks = 80 + 160 = 240 ticks (120.0 us)
    // Total Bus Active time = 2 * (20 + 80) + 2 * (40 + 20) = 200 + 120 = 320.0 us
    EXPECT_EQ(busController.getTransceiver().clock.tickCount, 240u);
    EXPECT_EQ(imuPrimary->getTransceiver().clock.tickCount, 160u);
    EXPECT_EQ(imuSecondary->getTransceiver().clock.tickCount, 160u);
    EXPECT_EQ(actuatorPitch->getTransceiver().clock.tickCount, 40u);
    EXPECT_EQ(actuatorYaw->getTransceiver().clock.tickCount, 40u);

    const double totalBusActiveUs = (busController.getTransceiver().clock.tickCount +
                                     imuPrimary->getTransceiver().clock.tickCount +
                                     imuSecondary->getTransceiver().clock.tickCount +
                                     actuatorPitch->getTransceiver().clock.tickCount +
                                     actuatorYaw->getTransceiver().clock.tickCount) * 0.5;

    EXPECT_DOUBLE_EQ(totalBusActiveUs, 320.0);

    // Bus wire returns to idle (0.0 V differential)
    EXPECT_DOUBLE_EQ(busController.getWire().differentialVoltage(), 0.0);
}
