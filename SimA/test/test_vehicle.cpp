//
// Created by Daniel St. John St. John on 10/24/26.
//

#include <gtest/gtest.h>
#include "Vehicle.h"

TEST(VehicleTest, DefaultConstruction) {
    Vehicle v;
    EXPECT_GT(v.translation.totalMass(), 0.0);
    EXPECT_GT(v.translation.position.norm(), sim::constants::EARTH_RADIUS);
    EXPECT_NEAR(v.rotation.attitude.norm(), 1.0, 1e-6);
    EXPECT_EQ(v.simulationTime, 0.0);
}

TEST(VehicleTest, VectorOperations) {
    sim::Vector3D v1{1.0, 2.0, 3.0};
    sim::Vector3D v2{4.0, 5.0, 6.0};

    sim::Vector3D sum = v1 + v2;
    EXPECT_DOUBLE_EQ(sum.x, 5.0);
    EXPECT_DOUBLE_EQ(sum.y, 7.0);
    EXPECT_DOUBLE_EQ(sum.z, 9.0);

    double dotProd = v1.dot(v2);
    EXPECT_DOUBLE_EQ(dotProd, 32.0);

    sim::Vector3D crossProd = v1.cross(v2);
    EXPECT_DOUBLE_EQ(crossProd.x, -3.0);
    EXPECT_DOUBLE_EQ(crossProd.y, 6.0);
    EXPECT_DOUBLE_EQ(crossProd.z, -3.0);
}

TEST(VehicleTest, QuaternionRotation) {
    // 90 degree rotation around Z axis
    sim::Quaternion q = sim::Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, M_PI / 2.0);
    sim::Vector3D bodyVec{1.0, 0.0, 0.0};
    sim::Vector3D eciVec = q.rotateBodyToEci(bodyVec);

    EXPECT_NEAR(eciVec.x, 0.0, 1e-6);
    EXPECT_NEAR(eciVec.y, 1.0, 1e-6);
    EXPECT_NEAR(eciVec.z, 0.0, 1e-6);

    sim::Vector3D backToBody = q.rotateEciToBody(eciVec);
    EXPECT_NEAR(backToBody.x, 1.0, 1e-6);
    EXPECT_NEAR(backToBody.y, 0.0, 1e-6);
    EXPECT_NEAR(backToBody.z, 0.0, 1e-6);
}

TEST(VehicleTest, OrbitPropagationStep) {
    // 500 km circular LEO
    double r0 = sim::constants::EARTH_RADIUS + 500000.0;
    double v0 = std::sqrt(sim::constants::MU_EARTH / r0);

    Vehicle v({r0, 0.0, 0.0}, {0.0, v0, 0.0}, 500.0, sim::Matrix3x3::fromDiagonal(50.0, 60.0, 70.0));
    v.enableAtmosphericDrag = false;
    v.enableJ2Perturbation = false;

    double initialRadius = v.translation.position.norm();
    double initialSpeed = v.translation.velocity.norm();

    // Advance 100 steps of 0.01s (1 second)
    for (int i = 0; i < 100; ++i) {
        v.cycle(0.01);
    }

    EXPECT_NEAR(v.translation.position.norm(), initialRadius, 10.0);
    EXPECT_NEAR(v.translation.velocity.norm(), initialSpeed, 1.0);
    EXPECT_NEAR(v.simulationTime, 1.0, 1e-5);
}

TEST(VehicleTest, AttitudeDynamicsWithTorque) {
    Vehicle v;
    sim::Vector3D torque{1.0, 0.0, 0.0}; // 1 N*m on X axis
    v.setControlTorque(torque);

    // Initial angular velocity is 0
    EXPECT_NEAR(v.rotation.angularVelocity.x, 0.0, 1e-6);

    // Step 0.1s
    v.cycle(0.1);

    // dw/dt = tau / Ixx = 1.0 / 50.0 = 0.02 rad/s^2
    // w_x ~ 0.02 * 0.1 = 0.002 rad/s
    EXPECT_NEAR(v.rotation.angularVelocity.x, 0.002, 1e-5);
}

TEST(VehicleTest, OrbitalElementsCalculation) {
    double r0 = sim::constants::EARTH_RADIUS + 500000.0;
    double v0 = std::sqrt(sim::constants::MU_EARTH / r0);

    Vehicle v({r0, 0.0, 0.0}, {0.0, v0, 0.0}, 500.0, sim::Matrix3x3::fromDiagonal(50.0, 60.0, 70.0));
    auto elem = v.getOrbitalElements();

    EXPECT_NEAR(elem.semiMajorAxis, r0, 100.0);
    EXPECT_NEAR(elem.eccentricity, 0.0, 1e-3);
    EXPECT_NEAR(elem.inclinationDeg, 0.0, 1e-3);
    EXPECT_GT(elem.orbitalPeriodSec, 5000.0);
}

TEST(VehicleTest, EigenInteroperability) {
    // Vector3D <-> Eigen::Vector3d
    sim::Vector3D v1{1.0, 2.0, 3.0};
    Eigen::Vector3d ev1 = v1;
    EXPECT_DOUBLE_EQ(ev1.x(), 1.0);
    EXPECT_DOUBLE_EQ(ev1.y(), 2.0);
    EXPECT_DOUBLE_EQ(ev1.z(), 3.0);

    sim::Vector3D v2 = ev1;
    EXPECT_DOUBLE_EQ(v2.x, 1.0);
    EXPECT_DOUBLE_EQ(v2.y, 2.0);
    EXPECT_DOUBLE_EQ(v2.z, 3.0);

    // Matrix3x3 <-> Eigen::Matrix3d
    sim::Matrix3x3 m1 = sim::Matrix3x3::fromDiagonal(10.0, 20.0, 30.0);
    Eigen::Matrix3d em1 = m1;
    EXPECT_DOUBLE_EQ(em1(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(em1(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(em1(2, 2), 30.0);
    EXPECT_DOUBLE_EQ(em1(0, 1), 0.0);

    sim::Matrix3x3 m2 = em1;
    EXPECT_DOUBLE_EQ(m2.m[0][0], 10.0);
    EXPECT_DOUBLE_EQ(m2.m[1][1], 20.0);
    EXPECT_DOUBLE_EQ(m2.m[2][2], 30.0);

    // Quaternion <-> Eigen::Quaterniond
    sim::Quaternion q1{1.0, 0.0, 0.0, 0.0};
    Eigen::Quaterniond eq1 = q1;
    EXPECT_DOUBLE_EQ(eq1.w(), 1.0);
    EXPECT_DOUBLE_EQ(eq1.x(), 0.0);
    EXPECT_DOUBLE_EQ(eq1.y(), 0.0);
    EXPECT_DOUBLE_EQ(eq1.z(), 0.0);

    sim::Quaternion q2 = eq1;
    EXPECT_DOUBLE_EQ(q2.w, 1.0);
    EXPECT_DOUBLE_EQ(q2.x, 0.0);
}
