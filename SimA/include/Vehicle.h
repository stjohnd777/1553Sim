//
// Created by Daniel St. John St. John on 10/24/26.
//

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sim {

// Standard Earth constants (WGS-84 / EGM96)
namespace constants {
    constexpr double MU_EARTH = 3.986004418e14;       // Earth gravitational parameter (m^3/s^2)
    constexpr double EARTH_RADIUS = 6378137.0;        // Earth equatorial radius (m)
    constexpr double EARTH_J2 = 1.08262668e-3;        // Second dynamic form factor (oblateness)
    constexpr double EARTH_ROTATION_RATE = 7.2921159e-5; // Earth rotation rate (rad/s)
} // namespace constants

// Reference frames for coordinate transformations
enum class Frame {
    ECI,   // Earth-Centered Inertial (e.g., J2000 / GCRF)
    ECEF,  // Earth-Centered Earth-Fixed (ITRF / WGS-84)
    LVLH,  // Local-Vertical Local-Horizontal (Orbit-relative)
    BODY   // Satellite Body-Fixed frame
};

// 3D Vector with linear algebra operations
struct Vector3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3D() = default;
    constexpr Vector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vector3D(const Eigen::Vector3d& v) : x(v.x()), y(v.y()), z(v.z()) {}

    Eigen::Vector3d toEigen() const {
        return {x, y, z};
    }

    operator Eigen::Vector3d() const {
        return toEigen();
    }

    Vector3D operator+(const Vector3D& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    Vector3D operator-(const Vector3D& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    Vector3D operator-() const {
        return {-x, -y, -z};
    }

    Vector3D operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    friend Vector3D operator*(double scalar, const Vector3D& vec) {
        return vec * scalar;
    }

    Vector3D operator/(double scalar) const {
        if (std::abs(scalar) < 1e-15) return {0.0, 0.0, 0.0};
        return {x / scalar, y / scalar, z / scalar};
    }

    Vector3D& operator+=(const Vector3D& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    Vector3D& operator-=(const Vector3D& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    Vector3D& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    Vector3D& operator/=(double scalar) {
        if (std::abs(scalar) >= 1e-15) {
            x /= scalar; y /= scalar; z /= scalar;
        }
        return *this;
    }

    double dot(const Vector3D& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vector3D cross(const Vector3D& rhs) const {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    double normSq() const {
        return x * x + y * y + z * z;
    }

    double norm() const {
        return std::sqrt(normSq());
    }

    Vector3D normalized() const {
        double n = norm();
        if (n < 1e-15) return {0.0, 0.0, 0.0};
        return *this / n;
    }
};

// 3x3 Matrix representation for inertia tensors and rotations
struct Matrix3x3 {
    std::array<std::array<double, 3>, 3> m{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

    constexpr Matrix3x3() = default;

    constexpr Matrix3x3(double m00, double m01, double m02,
                        double m10, double m11, double m12,
                        double m20, double m21, double m22)
        : m{{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}}} {}

    Matrix3x3(const Eigen::Matrix3d& mat)
        : m{{{mat(0, 0), mat(0, 1), mat(0, 2)},
             {mat(1, 0), mat(1, 1), mat(1, 2)},
             {mat(2, 0), mat(2, 1), mat(2, 2)}}} {}

    Eigen::Matrix3d toEigen() const {
        Eigen::Matrix3d res;
        res << m[0][0], m[0][1], m[0][2],
               m[1][0], m[1][1], m[1][2],
               m[2][0], m[2][1], m[2][2];
        return res;
    }

    operator Eigen::Matrix3d() const {
        return toEigen();
    }

    static constexpr Matrix3x3 identity() {
        return {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
    }

    static constexpr Matrix3x3 fromDiagonal(double ixx, double iyy, double izz) {
        return {ixx, 0.0, 0.0,
                0.0, iyy, 0.0,
                0.0, 0.0, izz};
    }

    Vector3D operator*(const Vector3D& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    double determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    Matrix3x3 inverse() const {
        double det = determinant();
        if (std::abs(det) < 1e-15) {
            return identity();
        }
        double invDet = 1.0 / det;
        return {
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet,

            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet,

            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet
        };
    }
};

// Unit Quaternion representation for attitude orientation [w, x, y, z]
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quaternion() = default;
    constexpr Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    Quaternion(const Eigen::Quaterniond& q) : w(q.w()), x(q.x()), y(q.y()), z(q.z()) {}

    Eigen::Quaterniond toEigen() const {
        return {w, x, y, z};
    }

    operator Eigen::Quaterniond() const {
        return toEigen();
    }

    static Quaternion fromAxisAngle(const Vector3D& axis, double angleRad) {
        Vector3D n = axis.normalized();
        double halfAngle = 0.5 * angleRad;
        double s = std::sin(halfAngle);
        return {std::cos(halfAngle), n.x * s, n.y * s, n.z * s};
    }

    static Quaternion fromEuler(double rollRad, double pitchRad, double yawRad) {
        double cr = std::cos(rollRad * 0.5);
        double sr = std::sin(rollRad * 0.5);
        double cp = std::cos(pitchRad * 0.5);
        double sp = std::sin(pitchRad * 0.5);
        double cy = std::cos(yawRad * 0.5);
        double sy = std::sin(yawRad * 0.5);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    double norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalized() const {
        double n = norm();
        if (n < 1e-15) return {1.0, 0.0, 0.0, 0.0};
        return {w / n, x / n, y / n, z / n};
    }

    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }

    // Hamilton quaternion product
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Rotate vector from Body frame to ECI frame using q * [0, v] * q*
    Vector3D rotateBodyToEci(const Vector3D& v) const {
        Vector3D qv{x, y, z};
        Vector3D t = 2.0 * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    // Rotate vector from ECI frame to Body frame using q* * [0, v] * q
    Vector3D rotateEciToBody(const Vector3D& v) const {
        return conjugate().rotateBodyToEci(v);
    }

    // Time derivative: dq/dt = 0.5 * q * [0, omega]
    Quaternion derivative(const Vector3D& omegaBody) const {
        return {
            0.5 * (-x * omegaBody.x - y * omegaBody.y - z * omegaBody.z),
            0.5 * ( w * omegaBody.x + y * omegaBody.z - z * omegaBody.y),
            0.5 * ( w * omegaBody.y - x * omegaBody.z + z * omegaBody.x),
            0.5 * ( w * omegaBody.z + x * omegaBody.y - y * omegaBody.x)
        };
    }

    // Conversion to Euler angles in radians (Roll, Pitch, Yaw / ZYX sequence)
    void toEuler(double& roll, double& pitch, double& yaw) const {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp); // 90 deg gimbal lock
        } else {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};

// Translational dynamic state (Orbit)
struct TranslationalState {
    Vector3D position{constants::EARTH_RADIUS + 500000.0, 0.0, 0.0}; // ECI position (m)
    Vector3D velocity{0.0, 7612.6, 0.0};                             // ECI velocity (m/s)
    Vector3D acceleration{0.0, 0.0, 0.0};                           // ECI acceleration (m/s^2)

    double dryMass{400.0};                                           // Dry mass (kg)
    double propellantMass{100.0};                                    // Propellant mass (kg)
    double massDepletionRate{0.0};                                   // Mass consumption rate (kg/s)

    double totalMass() const {
        return dryMass + propellantMass;
    }
};

// Rotational dynamic state (Attitude)
struct RotationalState {
    Quaternion attitude{1.0, 0.0, 0.0, 0.0};                      // Body to ECI attitude quaternion
    Vector3D angularVelocity{0.0, 0.0, 0.0};                      // Body frame rates (rad/s)
    Vector3D angularAcceleration{0.0, 0.0, 0.0};                  // Body frame angular acceleration (rad/s^2)
    Matrix3x3 inertiaTensor{Matrix3x3::fromDiagonal(50.0, 60.0, 70.0)}; // Inertia tensor in body frame (kg*m^2)
    Vector3D wheelMomentum{0.0, 0.0, 0.0};                        // Momentum stored in reaction wheels (N*m*s)
};

// External forces acting on the satellite
struct EnvironmentalForces {
    Vector3D gravity{0.0, 0.0, 0.0};      // Gravitational force in ECI (N)
    Vector3D drag{0.0, 0.0, 0.0};         // Aerodynamic drag force in ECI (N)
    Vector3D srp{0.0, 0.0, 0.0};          // Solar radiation pressure force in ECI (N)
    Vector3D thrust{0.0, 0.0, 0.0};       // Thruster propulsion force in Body or ECI (N)

    Vector3D totalForce() const {
        return gravity + drag + srp + thrust;
    }
};

// Torques acting on the satellite body
struct Torques {
    Vector3D controlTorque{0.0, 0.0, 0.0};     // Active control torque (Reaction wheels / RCS / Magtorquers) in Body frame (N*m)
    Vector3D disturbanceTorque{0.0, 0.0, 0.0}; // Environmental disturbance torques in Body frame (N*m)

    Vector3D totalTorque() const {
        return controlTorque + disturbanceTorque;
    }
};

// Classical Keplerian Orbital Elements
struct OrbitalElements {
    double semiMajorAxis{0.0};    // a (m)
    double eccentricity{0.0};     // e
    double inclinationDeg{0.0};   // i (deg)
    double raanDeg{0.0};          // Right Ascension of Ascending Node (deg)
    double argOfPerigeeDeg{0.0};  // Argument of Perigee (deg)
    double trueAnomalyDeg{0.0};   // True Anomaly (deg)
    double orbitalPeriodSec{0.0}; // Orbital period (s)
};

} // namespace sim

class Vehicle {
public:
    std::string name{"Satellite-01"};
    double simulationTime{0.0}; // Epoch / Simulation time in seconds

    sim::TranslationalState translation{};
    sim::RotationalState rotation{};
    sim::EnvironmentalForces forces{};
    sim::Torques torques{};

    // Drag and SRP coefficients
    double dragCoefficient{2.2};
    double crossSectionalArea{2.5}; // m^2
    bool enableJ2Perturbation{true};
    bool enableAtmosphericDrag{true};

public:
    Vehicle() = default;
    virtual ~Vehicle() = default;

    // Parameterized constructor
    Vehicle(const sim::Vector3D& initialPositionEci,
            const sim::Vector3D& initialVelocityEci,
            double massKg,
            const sim::Matrix3x3& inertia) {
        translation.position = initialPositionEci;
        translation.velocity = initialVelocityEci;
        translation.dryMass = massKg * 0.8;
        translation.propellantMass = massKg * 0.2;
        rotation.inertiaTensor = inertia;
    }

    // Step simulation dynamics (default 10 ms / 100 Hz step)
    virtual void cycle(double dt = 0.01) {
        if (dt <= 0.0) return;

        // 1. Calculate environmental forces in ECI frame
        updateEnvironmentalForces();

        // 2. Translational Dynamics Integration (Newton's 2nd Law)
        double totalMass = translation.totalMass();
        if (totalMass > 0.0) {
            translation.acceleration = forces.totalForce() / totalMass;
            translation.velocity += translation.acceleration * dt;
            translation.position += translation.velocity * dt;

            // Mass depletion
            if (translation.massDepletionRate > 0.0 && translation.propellantMass > 0.0) {
                double consumed = translation.massDepletionRate * dt;
                translation.propellantMass = std::max(0.0, translation.propellantMass - consumed);
            }
        }

        // 3. Rotational Dynamics Integration (Euler's Rotational Equations)
        // J * dw/dt + w x (J*w + h_rw) = tau_total
        // dw/dt = J^-1 * (tau_total - w x (J*w + h_rw))
        sim::Matrix3x3 invInertia = rotation.inertiaTensor.inverse();
        sim::Vector3D totalH = rotation.inertiaTensor * rotation.angularVelocity + rotation.wheelMomentum;
        sim::Vector3D gyroscopicTorque = rotation.angularVelocity.cross(totalH);
        sim::Vector3D netTorque = torques.totalTorque() - gyroscopicTorque;

        rotation.angularAcceleration = invInertia * netTorque;
        rotation.angularVelocity += rotation.angularAcceleration * dt;

        // 4. Attitude Kinematics Integration (dq/dt = 0.5 * q * [0, w])
        sim::Quaternion qDot = rotation.attitude.derivative(rotation.angularVelocity);
        rotation.attitude.w += qDot.w * dt;
        rotation.attitude.x += qDot.x * dt;
        rotation.attitude.y += qDot.y * dt;
        rotation.attitude.z += qDot.z * dt;
        rotation.attitude = rotation.attitude.normalized();

        // 5. Advance simulation epoch
        simulationTime += dt;
    }

    // Overload without arguments for default cycle
    void cycle() {
        cycle(0.01);
    }

    // Control & Actuation interfaces
    void setControlTorque(const sim::Vector3D& torqueBody) {
        torques.controlTorque = torqueBody;
    }

    void setDisturbanceTorque(const sim::Vector3D& disturbanceBody) {
        torques.disturbanceTorque = disturbanceBody;
    }

    void setWheelMomentum(const sim::Vector3D& wheelH) {
        rotation.wheelMomentum = wheelH;
    }

    void applyThrust(const sim::Vector3D& thrustVec, sim::Frame frame = sim::Frame::BODY) {
        if (frame == sim::Frame::BODY) {
            forces.thrust = rotation.attitude.rotateBodyToEci(thrustVec);
        } else {
            forces.thrust = thrustVec;
        }
    }

    void clearThrust() {
        forces.thrust = {0.0, 0.0, 0.0};
    }

    // Frame transformation helpers
    sim::Vector3D bodyToEci(const sim::Vector3D& bodyVec) const {
        return rotation.attitude.rotateBodyToEci(bodyVec);
    }

    sim::Vector3D eciToBody(const sim::Vector3D& eciVec) const {
        return rotation.attitude.rotateEciToBody(eciVec);
    }

    // Orbital state metrics
    double getAltitude() const {
        return translation.position.norm() - sim::constants::EARTH_RADIUS;
    }

    double getSpeed() const {
        return translation.velocity.norm();
    }

    sim::OrbitalElements getOrbitalElements() const {
        sim::OrbitalElements elem{};
        double r = translation.position.norm();
        double v = translation.velocity.norm();
        if (r < 1e-3) return elem;

        // Specific angular momentum h = r x v
        sim::Vector3D h = translation.position.cross(translation.velocity);
        double hNorm = h.norm();

        // Specific mechanical energy eps = v^2/2 - mu/r
        double energy = (v * v) / 2.0 - sim::constants::MU_EARTH / r;

        // Semi-major axis a = -mu / (2*eps)
        if (std::abs(energy) > 1e-9) {
            elem.semiMajorAxis = -sim::constants::MU_EARTH / (2.0 * energy);
            if (elem.semiMajorAxis > 0.0) {
                elem.orbitalPeriodSec = 2.0 * M_PI * std::sqrt(std::pow(elem.semiMajorAxis, 3) / sim::constants::MU_EARTH);
            }
        }

        // Eccentricity vector e = ((v^2 - mu/r)*r - (r.dot(v))*v) / mu
        sim::Vector3D eVec = (translation.position * (v * v - sim::constants::MU_EARTH / r) -
                              translation.velocity * translation.position.dot(translation.velocity)) / sim::constants::MU_EARTH;
        elem.eccentricity = eVec.norm();

        // Inclination i = acos(hz / ||h||)
        if (hNorm > 1e-9) {
            double incRad = std::acos(std::clamp(h.z / hNorm, -1.0, 1.0));
            elem.inclinationDeg = incRad * (180.0 / M_PI);

            // Node vector n = k x h = [-hy, hx, 0]
            sim::Vector3D n{-h.y, h.x, 0.0};
            double nNorm = n.norm();

            if (nNorm > 1e-9) {
                // RAAN
                double raanRad = std::acos(std::clamp(n.x / nNorm, -1.0, 1.0));
                if (n.y < 0.0) raanRad = 2.0 * M_PI - raanRad;
                elem.raanDeg = raanRad * (180.0 / M_PI);

                // Argument of Perigee
                if (elem.eccentricity > 1e-6) {
                    double argPRad = std::acos(std::clamp(n.dot(eVec) / (nNorm * elem.eccentricity), -1.0, 1.0));
                    if (eVec.z < 0.0) argPRad = 2.0 * M_PI - argPRad;
                    elem.argOfPerigeeDeg = argPRad * (180.0 / M_PI);
                }
            }

            // True Anomaly
            if (elem.eccentricity > 1e-6) {
                double nuRad = std::acos(std::clamp(eVec.dot(translation.position) / (elem.eccentricity * r), -1.0, 1.0));
                if (translation.position.dot(translation.velocity) < 0.0) {
                    nuRad = 2.0 * M_PI - nuRad;
                }
                elem.trueAnomalyDeg = nuRad * (180.0 / M_PI);
            }
        }

        return elem;
    }

private:
    void updateEnvironmentalForces() {
        double r = translation.position.norm();
        if (r < 1e-3) return;

        // 1. Two-Body Point Mass Gravity: a_grav = -mu / r^3 * r
        double r3 = r * r * r;
        sim::Vector3D aGrav = translation.position * (-sim::constants::MU_EARTH / r3);

        // 2. Earth Oblateness Perturbation (J2 harmonic)
        if (enableJ2Perturbation) {
            double r2 = r * r;
            double r5 = r3 * r2;
            double z2 = translation.position.z * translation.position.z;
            double factor = 1.5 * sim::constants::EARTH_J2 * sim::constants::MU_EARTH * (sim::constants::EARTH_RADIUS * sim::constants::EARTH_RADIUS) / r5;

            double j2_x = -factor * translation.position.x * (1.0 - 5.0 * z2 / r2);
            double j2_y = -factor * translation.position.y * (1.0 - 5.0 * z2 / r2);
            double j2_z = -factor * translation.position.z * (3.0 - 5.0 * z2 / r2);

            aGrav += sim::Vector3D{j2_x, j2_y, j2_z};
        }

        forces.gravity = aGrav * translation.totalMass();

        // 3. Atmospheric Drag (Exponential density model for LEO)
        if (enableAtmosphericDrag) {
            double altitude = r - sim::constants::EARTH_RADIUS;
            if (altitude > 0.0 && altitude < 1000000.0) { // Under 1000 km
                // Simple exponential scale height model
                double rho0 = 1.225; // kg/m^3 sea level
                double hScale = 8500.0; // scale height ~8.5 km
                if (altitude > 100000.0) { // Upper thermosphere adjustments
                    rho0 = 5.0e-7;
                    hScale = 60000.0;
                }
                double rho = rho0 * std::exp(-altitude / hScale);

                // Earth co-rotation velocity v_atm = omega_earth x r
                sim::Vector3D vAtm{-sim::constants::EARTH_ROTATION_RATE * translation.position.y,
                                    sim::constants::EARTH_ROTATION_RATE * translation.position.x,
                                    0.0};
                sim::Vector3D vRel = translation.velocity - vAtm;
                double vRelNorm = vRel.norm();

                if (vRelNorm > 1e-3) {
                    // F_drag = -0.5 * rho * Cd * A * |v_rel| * v_rel
                    forces.drag = vRel * (-0.5 * rho * dragCoefficient * crossSectionalArea * vRelNorm);
                } else {
                    forces.drag = {0.0, 0.0, 0.0};
                }
            } else {
                forces.drag = {0.0, 0.0, 0.0};
            }
        } else {
            forces.drag = {0.0, 0.0, 0.0};
        }
    }
};
