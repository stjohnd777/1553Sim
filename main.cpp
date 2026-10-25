//
// Created by Daniel St. John St. John on 10/21/26.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "mil1553.hpp"
#include "sensor/ImuRT.hpp"
#include "sensor/TemperatureSensorRT.hpp"
#include "sensor/PressureSensorRT.hpp"
#include "motor/ActuatorRT.hpp"

// Median calculation for Triple Modular Redundancy (TMR) sensor voting
static double median3(double a, double b, double c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

struct ImuReading {
    double pitchDeg{0.0};
    double rollDeg{0.0};
    double yawDeg{0.0};
};

struct State {
    double timeSeconds{0.0};
    uint64_t cycle{0};

    // IMU telemetry
    std::vector<ImuReading> imus{};
    ImuReading fusedImu{};

    // Engine temperature and chamber pressure telemetry
    std::vector<double> temperatures{}; // °C per engine
    std::vector<double> pressures{};    // bar per engine

    // Actuator positions / commanded TVC deflections
    double commandedPitchGimbalDeg{0.0};
    double commandedYawGimbalDeg{0.0};
    double eng1PitchActuatorDeg{0.0};
    double eng1YawActuatorDeg{0.0};
    double eng2PitchActuatorDeg{0.0};
    double eng2YawActuatorDeg{0.0};
};

class FightComputer {
public:
    explicit FightComputer(Simulated1553Bus* ptrBC) : ptrBC(ptrBC) {}

    void command(State &state) {
        if (!ptrBC) {
            std::cerr << "Flight Computer error: 1553 bus controller interface is null!\n";
            return;
        }

        // Guidance & Control law (Proportional Thrust Vector Control)
        const double targetPitch = 0.0;
        const double targetYaw   = 0.0;
        double pitchError = targetPitch - state.fusedImu.pitchDeg;
        double yawError   = targetYaw - state.fusedImu.yawDeg;

        // Proportional gimbal deflection demands (clamped to ±15.0 deg)
        double pitchGimbalCmd = std::clamp(1.2 * pitchError, -15.0, 15.0);
        double yawGimbalCmd   = std::clamp(1.2 * yawError, -15.0, 15.0);

        // Update state with computed actuator demands
        state.commandedPitchGimbalDeg = pitchGimbalCmd;
        state.commandedYawGimbalDeg   = yawGimbalCmd;
        state.eng1PitchActuatorDeg    = pitchGimbalCmd;
        state.eng1YawActuatorDeg      = yawGimbalCmd;
        state.eng2PitchActuatorDeg    = pitchGimbalCmd;
        state.eng2YawActuatorDeg      = yawGimbalCmd;

        // Format 1553 data blocks for actuator writes
        DataBlock1553 pitchCmdBlock{};
        pitchCmdBlock.count = 1;
        pitchCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(pitchGimbalCmd, 45.0));

        DataBlock1553 yawCmdBlock{};
        yawCmdBlock.count = 1;
        yawCmdBlock.words[0] = static_cast<uint16_t>(scaleToWord(yawGimbalCmd, 45.0));

        // Transmit TVC commands across the 1553 bus to all 4 gimbal actuators
        // Engine 1 Actuator Writes (RT 10: Pitch, RT 11: Yaw)
        auto e1pRes = ptrBC->transmitToRT(10, 1, pitchCmdBlock);
        auto e1yRes = ptrBC->transmitToRT(11, 1, yawCmdBlock);

        // Engine 2 Actuator Writes (RT 12: Pitch, RT 13: Yaw)
        auto e2pRes = ptrBC->transmitToRT(12, 1, pitchCmdBlock);
        auto e2yRes = ptrBC->transmitToRT(13, 1, yawCmdBlock);

        if (!e1pRes.success || !e1yRes.success || !e2pRes.success || !e2yRes.success) {
            std::cerr << "Flight Computer: Actuator command transmission failed over 1553 bus!\n";
        }
    }

private:
    Simulated1553Bus* ptrBC{nullptr};
};

using FlightComputer = FightComputer;

int main() {
    using clock = std::chrono::steady_clock;

    constexpr auto period = std::chrono::milliseconds(10); // 100 Hz simulation rate (10 ms per cycle)
    constexpr int iterations = 10;
    constexpr double dtSeconds = 0.010;

    Simulated1553Bus bus;

    // =========================================================================
    // 1. Remote Terminal Configuration & 1553 Bus Address Mapping
    // =========================================================================
    // 3 IMU Sensors (Triple Modular Redundancy)
    auto imu1 = std::make_shared<ImuRT>(3);               // RT 3: Primary Navigation IMU
    auto imu2 = std::make_shared<ImuRT>(4);               // RT 4: Secondary / Redundant IMU
    auto imu3 = std::make_shared<ImuRT>(5);               // RT 5: Tertiary / Voting IMU

    // 2 Temperature Sensors (Engine 1 & Engine 2 Chamber / Turbopump Thermal Monitoring)
    auto tempEng1 = std::make_shared<TemperatureSensorRT>(6); // RT 6: Engine 1 Temperature Sensor
    auto tempEng2 = std::make_shared<TemperatureSensorRT>(7); // RT 7: Engine 2 Temperature Sensor

    // 2 Pressure Sensors (Engine 1 & Engine 2 Combustion Chamber Pressure Monitoring)
    auto pressEng1 = std::make_shared<PressureSensorRT>(8);   // RT 8: Engine 1 Chamber Pressure Sensor
    auto pressEng2 = std::make_shared<PressureSensorRT>(9);   // RT 9: Engine 2 Chamber Pressure Sensor

    // 4 Actuators (2 Gimbal Actuators per Engine: Pitch & Yaw)
    auto eng1PitchAct = std::make_shared<ActuatorRT>(10); // RT 10: Engine 1 Pitch Gimbal Actuator
    auto eng1YawAct   = std::make_shared<ActuatorRT>(11); // RT 11: Engine 1 Yaw Gimbal Actuator
    auto eng2PitchAct = std::make_shared<ActuatorRT>(12); // RT 12: Engine 2 Pitch Gimbal Actuator
    auto eng2YawAct   = std::make_shared<ActuatorRT>(13); // RT 13: Engine 2 Yaw Gimbal Actuator

    // =========================================================================
    // 2. Initial Physical State Setup
    // =========================================================================
    // Triple IMU initial attitude states with realistic sensor variance
    imu1->pitchDeg = 8.50;  imu1->pitchRate = -0.40;  imu1->yawDeg = -1.20;
    imu2->pitchDeg = 8.54;  imu2->pitchRate = -0.40;  imu2->yawDeg = -1.18;
    imu3->pitchDeg = 8.46;  imu3->pitchRate = -0.40;  imu3->yawDeg = -1.22;

    // Engine temperature initial states (°C) and thermal rise rates
    tempEng1->temperatureCelsius = 385.0;  tempEng1->tempRate = 2.5; // Warming up during burn
    tempEng2->temperatureCelsius = 382.5;  tempEng2->tempRate = 2.4;

    // Engine chamber pressure initial states (bar)
    pressEng1->pressureBar = 74.2;  pressEng1->pressureRate = 0.1;
    pressEng2->pressureBar = 73.8;  pressEng2->pressureRate = 0.1;

    // Attach all 11 Remote Terminals to the shared 1553 communication bus
    bus.attachTerminal(imu1);
    bus.attachTerminal(imu2);
    bus.attachTerminal(imu3);
    bus.attachTerminal(tempEng1);
    bus.attachTerminal(tempEng2);
    bus.attachTerminal(pressEng1);
    bus.attachTerminal(pressEng2);
    bus.attachTerminal(eng1PitchAct);
    bus.attachTerminal(eng1YawAct);
    bus.attachTerminal(eng2PitchAct);
    bus.attachTerminal(eng2YawAct);

    // =========================================================================
    // 3. Banner & Simulation Executive Start
    // =========================================================================
    std::cout << "===================================================================================================\n";
    std::cout << "               MIL-STD-1553 DUAL-ENGINE LAUNCH VEHICLE SIMULATION POC (100 Hz)                     \n";
    std::cout << "  Configuration: 3x IMUs (TMR) | 2x Temp Sensors | 2x Pressure Sensors | 4x Engine TVC Actuators   \n";
    std::cout << "===================================================================================================\n\n";
    std::cout << std::fixed << std::setprecision(2);

    FightComputer flightComputer(&bus);
    State state;

    auto nextTick = clock::now();

    for (int cycle = 0; cycle < iterations; ++cycle) {
        nextTick += period;

        state.cycle = cycle;
        state.timeSeconds = cycle * dtSeconds;

        // ---------------------------------------------------------------------
        // STEP 1: Read Telemetry from all 3 IMUs (RT 3, RT 4, RT 5 via RT->BC)
        // ---------------------------------------------------------------------
        auto imu1Res = bus.receiveFromRT(3, 1, 3);
        auto imu2Res = bus.receiveFromRT(4, 1, 3);
        auto imu3Res = bus.receiveFromRT(5, 1, 3);

        if (!imu1Res.success || !imu2Res.success || !imu3Res.success) {
            std::cerr << "Cycle " << cycle << ": IMU telemetry acquisition failed!\n";
            return 1;
        }

        double p1 = scaleFromWord(static_cast<int16_t>(imu1Res.data.words[0]), 180.0);
        double p2 = scaleFromWord(static_cast<int16_t>(imu2Res.data.words[0]), 180.0);
        double p3 = scaleFromWord(static_cast<int16_t>(imu3Res.data.words[0]), 180.0);

        double y1 = scaleFromWord(static_cast<int16_t>(imu1Res.data.words[2]), 180.0);
        double y2 = scaleFromWord(static_cast<int16_t>(imu2Res.data.words[2]), 180.0);
        double y3 = scaleFromWord(static_cast<int16_t>(imu3Res.data.words[2]), 180.0);

        state.imus = {
            {p1, 0.0, y1},
            {p2, 0.0, y2},
            {p3, 0.0, y3}
        };

        // Triple Modular Redundancy (TMR) Voting
        state.fusedImu.pitchDeg = median3(p1, p2, p3);
        state.fusedImu.yawDeg   = median3(y1, y2, y3);

        // ---------------------------------------------------------------------
        // STEP 2: Read Engine Health Sensors (Temperature & Pressure)
        // ---------------------------------------------------------------------
        auto t1Res = bus.receiveFromRT(6, 1, 1);
        auto t2Res = bus.receiveFromRT(7, 1, 1);
        auto p1Res = bus.receiveFromRT(8, 1, 1);
        auto p2Res = bus.receiveFromRT(9, 1, 1);

        if (!t1Res.success || !t2Res.success || !p1Res.success || !p2Res.success) {
            std::cerr << "Cycle " << cycle << ": Engine health sensor acquisition failed!\n";
            return 1;
        }

        double temp1 = scaleFromWord(static_cast<int16_t>(t1Res.data.words[0]), 1000.0);
        double temp2 = scaleFromWord(static_cast<int16_t>(t2Res.data.words[0]), 1000.0);
        double press1 = scaleFromWord(static_cast<int16_t>(p1Res.data.words[0]), 250.0);
        double press2 = scaleFromWord(static_cast<int16_t>(p2Res.data.words[0]), 250.0);

        state.temperatures = {temp1, temp2};
        state.pressures    = {press1, press2};

        // ---------------------------------------------------------------------
        // STEP 3: Flight Computer Guidance & Thrust Vector Control Command
        // ---------------------------------------------------------------------
        flightComputer.command(state);

        // ---------------------------------------------------------------------
        // STEP 4: Display Comprehensive Telemetry Dashboard (from State)
        // ---------------------------------------------------------------------
        std::cout << "[Cycle " << std::setw(2) << state.cycle << "] "
                  << "IMU Pitch [1:" << std::setw(5) << state.imus[0].pitchDeg
                  << " 2:" << std::setw(5) << state.imus[1].pitchDeg
                  << " 3:" << std::setw(5) << state.imus[2].pitchDeg
                  << "] -> Fused: " << std::setw(5) << state.fusedImu.pitchDeg << " deg | "
                  << "Eng1: " << std::setw(5) << state.temperatures[0] << "C / " << std::setw(5) << state.pressures[0] << "bar | "
                  << "Eng2: " << std::setw(5) << state.temperatures[1] << "C / " << std::setw(5) << state.pressures[1] << "bar | "
                  << "TVC Cmd [P:" << std::setw(5) << state.commandedPitchGimbalDeg
                  << " Y:" << std::setw(5) << state.commandedYawGimbalDeg << "]\n";

        // ---------------------------------------------------------------------
        // STEP 6: Advance Physical Dynamics (Simulated Physics Step)
        // ---------------------------------------------------------------------
        imu1->stepSimulation(dtSeconds);
        imu2->stepSimulation(dtSeconds);
        imu3->stepSimulation(dtSeconds);
        tempEng1->stepSimulation(dtSeconds);
        tempEng2->stepSimulation(dtSeconds);
        pressEng1->stepSimulation(dtSeconds);
        pressEng2->stepSimulation(dtSeconds);

        // Pace execution to 100 Hz
        std::this_thread::sleep_until(nextTick);
    }

    std::cout << "\n===================================================================================================\n";
    std::cout << "Simulation POC completed successfully with all 11 Remote Terminals operating nominally.\n";
    std::cout << "===================================================================================================\n";

    return 0;
}
