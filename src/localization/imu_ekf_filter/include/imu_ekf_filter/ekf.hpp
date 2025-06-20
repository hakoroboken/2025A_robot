#ifndef EKF_HPP_
#define EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace ekf
{
    typedef Matrix<double, 3, 2> Matrix3x2;
    typedef Matrix<double, 2, 3> Matrix2x3;

    class EKF
    {
        public:
        /// @brief コンストラクタ
        EKF();

        /// @brief 加速度と角速度から姿勢を推定する
        /// @param angular 角速度[rad/s]
        /// @param linear_accel 加速度[m/s^2]
        /// @param dt 時間間隔
        /// @return クォータニオン姿勢
        Quaterniond estimate(const Vector3d &angular, const Vector3d &linear_accel, const double &dt);

        private:
        Matrix3d cov_;
        Matrix3d estimation_noise_;
        Matrix2d observation_noise_;
        Matrix3x2 kalman_gain_;
        Vector3d estimation_;
    };
    Vector3d getInputMatrix(const Vector3d& angular_velocity, const double &dt);

    Matrix3x2 h();

    Matrix3d jacob(const Vector3d &input_matrix, const Vector3d &estimation);

    Vector3d predictX(const Vector3d &input_matrix, const Vector3d &estimation);

    Matrix3d predictCov(const Matrix3d &jacob, const Matrix3d &cov, const Matrix3d &est_noise);

    Vector2d updateResidual(const Vector2d &obs, const Vector3d &est);

    Matrix2d updateS(const Matrix3d &cov_, const Matrix2d &obs_noise);

    Matrix3x2 updateKalmanGain(const Matrix2d &s, const Matrix3d &cov);

    Vector3d updateX(const Vector3d &est, const Matrix3x2 &kalman_gain_, const Vector2d &residual);

    Matrix3d updateCov(const Matrix3x2 &kalman_gain, const Matrix3d &cov);

    Vector2d obsModel(const Vector3d &linear_accel);

    Vector3d createVec3(const double& x, const double& y, const double& z);
}

#endif