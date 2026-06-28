#pragma once 

#include <Eigen/Dense>
#include <cmath>

namespace RobotLocalization
{

// ======================================================
// STATE ENUM
// ======================================================



// ======================================================
// TYPEDEFS
// ======================================================

using StateVector =
    Eigen::Matrix<double,
                  config::DimWheelchairStateVector,
                  1>;

using StateMatrix =
    Eigen::Matrix<double,
                  config::DimWheelchairStateVector,
                  config::DimWheelchairStateVector>;


// ======================================================
// EKF
// ======================================================

template<int StateSize>
class EKF
{
public:

    using Vector =
        Eigen::Matrix<double, StateSize, 1>;

    using Matrix =
        Eigen::Matrix<double, StateSize, StateSize>;

    template<int MeasSize>
    using MeasurementVector =
        Eigen::Matrix<double, MeasSize, 1>;

    template<int MeasSize>
    using MeasurementMatrix =
        Eigen::Matrix<double, MeasSize, MeasSize>;

    template<int MeasSize>
    using HMatrix =
        Eigen::Matrix<double, MeasSize, StateSize>;

public:

    // ==================================================
    // STATE
    // ==================================================

    Vector x;

    // covariance
    Matrix P;

public:

    // ==================================================
    // CONSTRUCTOR
    // ==================================================

    EKF()
    {
        x.setZero();

        P.setIdentity();
    }

    // ==================================================
    // ANGLE NORMALIZATION
    // ==================================================

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    // ==================================================
    // PREDICT
    // ==================================================

    void predict(
        const Vector& x_next,
        const Matrix& F,
        const Matrix& Q)
    {
        // state prediction
        x = x_next;

        // covariance prediction
        P = F * P * F.transpose() + Q;

        normalizeStateAngles();
    }

    // ==================================================
    // UPDATE
    // ==================================================

    template<int MeasSize>
    void update(
        const MeasurementVector<MeasSize>& z,
        const HMatrix<MeasSize>& H,
        const MeasurementMatrix<MeasSize>& R,
        const MeasurementVector<MeasSize>& z_pred,
        bool normalize_yaw_innovation = false,
        int yaw_index_in_measurement = -1)
    {

        MeasurementVector<MeasSize> y =
            z - z_pred;

        if (normalize_yaw_innovation &&
            yaw_index_in_measurement >= 0)
        {
            y(yaw_index_in_measurement) =
                normalizeAngle(
                    y(yaw_index_in_measurement));
        }

        MeasurementMatrix<MeasSize> S =
            H * P * H.transpose() + R;


        Eigen::Matrix<double, StateSize, MeasSize> K =
            P * H.transpose() *
            S.ldlt().solve(
                MeasurementMatrix<MeasSize>::Identity());

        x = x + K * y;

        normalizeStateAngles();

        Matrix I = Matrix::Identity();

        P =
            (I - K * H) *
            P *
            (I - K * H).transpose()
            +
            K * R * K.transpose();
    }

    // ==================================================
    // GETTERS
    // ==================================================

    double getX() const
    {
        return x(StateX);
    }

    double getY() const
    {
        return x(StateY);
    }

    double getYaw() const
    {
        return x(StateYaw);
    }

    double getVx() const
    {
        return x(StateVx);
    }

    double getVyaw() const
    {
        return x(StateVyaw);
    }

    // ==================================================
    // RESET
    // ==================================================

    void reset()
    {
        x.setZero();

        P.setIdentity();
    }

private:

    // ==================================================
    // NORMALIZE STATE ANGLES
    // ==================================================

    void normalizeStateAngles()
    {
        if constexpr (StateSize > StateYaw)
        {
            x(StateYaw) =
                normalizeAngle(
                    x(StateYaw));
        }
    }
};

}



using StateVector = Eigen::Matrix<double, config::DimWheelchairStateVector, 1>;


// ======================================================
// ANGLE NORMALIZATION
// ======================================================

inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}


// ======================================================
// POSITION
// ======================================================

inline double getX(const StateVector& x)
{
    return x(StateX);
}

inline double getY(const StateVector& x)
{
    return x(StateY);
}

inline double getZ(const StateVector& x)
{
    return x(StateZ);
}


// ======================================================
// ORIENTATION
// ======================================================

inline double getPitch(const StateVector& x)
{
    return x(StatePitch);
}

inline double getYaw(const StateVector& x)
{
    return x(StateYaw);
}


// ======================================================
// VELOCITY
// ======================================================

inline double getVx(const StateVector& x)
{
    return x(StateVx);
}

inline double getVyaw(const StateVector& x)
{
    return x(StateVyaw);
}


// ======================================================
// ACCELERATION
// ======================================================

inline double getAx(const StateVector& x)
{
    return x(StateAx);
}


// ======================================================
// SETTERS
// ======================================================

inline void setX(StateVector& x, double value)
{
    x(StateX) = value;
}

inline void setY(StateVector& x, double value)
{
    x(StateY) = value;
}

inline void setZ(StateVector& x, double value)
{
    x(StateZ) = value;
}

inline void setPitch(StateVector& x, double value)
{
    x(StatePitch) = value;
}

inline void setYaw(StateVector& x, double value)
{
    x(StateYaw) = normalizeAngle(value);
}

inline void setVx(StateVector& x, double value)
{
    x(StateVx) = value;
}

inline void setVyaw(StateVector& x, double value)
{
    x(StateVyaw) = value;
}

inline void setAx(StateVector& x, double value)
{
    x(StateAx) = value;
}


// ======================================================
// SPEED
// ======================================================

inline double getSpeed(const StateVector& x)
{
    return std::abs(x(StateVx));
}


// ======================================================
// IS STOPPED
// ======================================================

inline bool isStopped(const StateVector& x,
                      double threshold = 0.05)
{
    return std::abs(x(StateVx)) < threshold;
}


// ======================================================
// IS TURNING
// ======================================================

inline bool isTurning(const StateVector& x,
                      double threshold = 0.1)
{
    return std::abs(x(StateVyaw)) > threshold;
}


// ======================================================
// FUTURE POSITION
// ======================================================

inline Eigen::Vector2d predictFuturePosition(
        const StateVector& x,
        double dt)
{
    double px = x(StateX);
    double py = x(StateY);

    double yaw = x(StateYaw);
    double vx = x(StateVx);

    double future_x = px + vx * std::cos(yaw) * dt;
    double future_y = py + vx * std::sin(yaw) * dt;

    return Eigen::Vector2d(future_x, future_y);
}


// ======================================================
// FUTURE YAW
// ======================================================

inline double predictFutureYaw(
        const StateVector& x,
        double dt)
{
    double yaw = x(StateYaw);
    double vyaw = x(StateVyaw);

    return normalizeAngle(yaw + vyaw * dt);
}


// ======================================================
// FUTURE SPEED
// ======================================================

inline double predictFutureSpeed(
        const StateVector& x,
        double dt)
{
    return x(StateVx) + x(StateAx) * dt;
}


// ======================================================
// DISTANCE BETWEEN STATES
// ======================================================

inline double distanceBetween(
        const StateVector& a,
        const StateVector& b)
{
    double dx = a(StateX) - b(StateX);
    double dy = a(StateY) - b(StateY);

    return std::sqrt(dx * dx + dy * dy);
}


// ======================================================
// DIRECTION VECTOR
// ======================================================

inline Eigen::Vector2d getDirectionVector(
        const StateVector& x)
{
    double yaw = x(StateYaw);

    return Eigen::Vector2d(
        std::cos(yaw),
        std::sin(yaw)
    );
}


// ======================================================
// MOTION MODEL
// ======================================================

inline StateVector motionModel(
        const StateVector& x,
        double dt)
{
    StateVector x_next = x;

    double px = x(StateX);
    double py = x(StateY);

    double yaw = x(StateYaw);

    double vx = x(StateVx);
    double vyaw = x(StateVyaw);

    double ax = x(StateAx);

    // position
    x_next(StateX) =
        px + vx * std::cos(yaw) * dt;

    x_next(StateY) =
        py + vx * std::sin(yaw) * dt;

    // orientation
    x_next(StateYaw) =
        normalizeAngle(yaw + vyaw * dt);

    // velocity
    x_next(StateVx) =
        vx + ax * dt;

    return x_next;
}


// ======================================================
// RISK OF TIPPING
// ======================================================

inline bool riskOfTipping(
        const StateVector& x,
        double pitch_limit = 0.35,
        double accel_limit = 2.0)
{
    return
        std::abs(x(StatePitch)) > pitch_limit ||
        std::abs(x(StateAx)) > accel_limit;
}


// ======================================================
// COLLISION CHECK
// ======================================================

inline bool willCollide(
        const StateVector& wheelchair,
        const StateVector& object,
        double dt,
        double collision_distance)
{
    Eigen::Vector2d future_wheelchair =
        predictFuturePosition(wheelchair, dt);

    Eigen::Vector2d future_object =
        predictFuturePosition(object, dt);

    double dx =
        future_wheelchair.x() - future_object.x();

    double dy =
        future_wheelchair.y() - future_object.y();

    double distance =
        std::sqrt(dx * dx + dy * dy);

    return distance < collision_distance;
}
