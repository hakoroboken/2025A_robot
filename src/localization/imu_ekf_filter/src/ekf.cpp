#include "imu_ekf_filter/ekf.hpp"

namespace ekf
{
    EKF::EKF()
    : estimation_(Vector3d(0.0, 0.0, 0.0)),
    cov_(Matrix3d::Identity()),
    estimation_noise_(Matrix3d::Identity()),
    observation_noise_(Matrix2d::Zero()),
    kalman_gain_(Matrix3x2::Zero())
    {
        cov_(0, 0) = 0.0174*0.001;
        cov_(1, 1) = 0.0174*0.001;
        cov_(2, 2) = 0.0174*0.001;

        estimation_noise_(0, 0) = 0.0174*0.001;
        estimation_noise_(1, 1) = 0.0174*0.001;
        estimation_noise_(2, 2) = 0.0174*0.001;
    }

    Quaterniond EKF::estimate(const Vector3d &angular, const Vector3d &linear_accel, const double &dt)
    {
        const auto input_matrix = getInputMatrix(angular, dt);

        const auto jacob_ = jacob(input_matrix, estimation_);
        estimation_ = predictX(input_matrix, estimation_);
        cov_ = predictCov(jacob_, cov_, estimation_noise_);
        const auto obs_model = obsModel(linear_accel);
        const auto residual = updateResidual(obs_model, estimation_);
        const auto s = updateS(cov_, observation_noise_);
        kalman_gain_ = updateKalmanGain(s, cov_);
        estimation_ = updateX(estimation_, kalman_gain_, residual);
        cov_ = updateCov(kalman_gain_, cov_);

        AngleAxisd roll(estimation_.x(), Vector3d::UnitZ());
        AngleAxisd pitch(estimation_.y(), Vector3d::UnitY());
        AngleAxisd yaw(estimation_.z(), Vector3d::UnitX());

        const Quaterniond q = roll * pitch * yaw;

        return q;
    }

    Vector3d getInputMatrix(const Vector3d& angular_velocity, const double &dt)
    {
        return Vector3d(
            angular_velocity.x()* dt,
            angular_velocity.y()* dt,
            angular_velocity.z()* dt
        );
    }

    Matrix3x2 h()
    {
        Matrix3x2 h_;
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        
        return h_;
    }

    Matrix3d jacob(const Vector3d &input_matrix, const Vector3d &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m_11 = 1.0 + input_matrix.y() * ((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z() * ((sin_roll*sin_pitch)/cos_pitch);
        auto m_12 = input_matrix.y()*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll/(cos_pitch*cos_pitch)));
        auto m_21 = -1.0*input_matrix.y()*sin_roll - input_matrix.z()*cos_roll;
        auto m_31 = input_matrix.y()*(cos_roll/cos_pitch) - input_matrix.z()*(sin_roll/cos_pitch);
        auto m_32 = input_matrix.y()*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

        Matrix3d mat;
        mat.setZero();
        mat(0, 0) = m_11;
        mat(0, 1) = m_12;
        mat(1, 0) = m_21;
        mat(1, 1) = 1.0;
        mat(2, 0) = m_31;
        mat(2, 1) = m_32;

        return mat;
    }

    Vector3d predictX(const Vector3d &input_matrix, const Vector3d &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        Vector3d est;
        est(0) = estimation.x() + input_matrix.x() + input_matrix.y()*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z()*((cos_roll*sin_pitch)/cos_pitch);
        est(1) = estimation.y() + input_matrix.y() * cos_roll - input_matrix.z()*sin_roll;
        est(2) = estimation.z() + input_matrix.z() + input_matrix.y()*(sin_roll/cos_pitch) + input_matrix.z()*(cos_roll/cos_pitch);

        return est;
    }

    Matrix3d predictCov(const Matrix3d &jacob, const Matrix3d &cov, const Matrix3d &est_noise)
    {
        Matrix3d t_jacob = jacob.transpose();
        Matrix3d jacob_cov = jacob * cov;

        Matrix3d new_cov;
        new_cov.setZero();

        Matrix3d multiplied = jacob_cov * t_jacob;

        new_cov = multiplied + est_noise;

        return new_cov;
    }

    Vector2d updateResidual(const Vector2d &obs, const Vector3d &est)
    {
        Vector2d result;
        Matrix2x3 h_ = h().transpose();
        Vector2d h_est = h_ * est;

        result(0) = obs.x() - h_est.x();
        result(1) = obs.y() - h_est.y();

        return result;
    }

    Matrix2d updateS(const Matrix3d &cov_, const Matrix2d &obs_noise)
    {
        Matrix2x3 h_ = h().transpose();
        Matrix2x3 h_cov_ = h_ * cov_;
        Matrix2d convert_cov_ = h_cov_ * h();

        return obs_noise + convert_cov_;
    }

    Matrix3x2 updateKalmanGain(const Matrix2d &s, const Matrix3d &cov)
    {
        auto h_ = h();

        Matrix2d inverse_s = s.inverse();

        Matrix3x2 cov_and_h = cov * h_;

        return cov_and_h * inverse_s;
    }

    Vector3d updateX(const Vector3d &est, const Matrix3x2 &kalman_gain_, const Vector2d &residual)
    {
        Vector3d kalman_res = kalman_gain_ * residual;

        Vector3d result;
        result.setZero();

        result(0) = est.x() + kalman_res.x();
        result(1) = est.y() + kalman_res.y();
        result(2) = est.z() + kalman_res.z();

        return result;
    }

    Matrix3d updateCov(const Matrix3x2 &kalman_gain, const Matrix3d &cov)
    {
        Matrix3d i;
        i.setIdentity();

        Matrix2x3 h_ = h().transpose();

        Matrix3d kalman_h = kalman_gain * h_;

        Matrix3d i_k_h = i - kalman_h;

        return i_k_h * cov;
    }

    Vector2d obsModel(const Vector3d &linear_accel)
    {
        Vector2d model;

        if(linear_accel.z() == 0.0)
        {
            if(linear_accel.y() > 0.0)
            {
                model(0) = acos(-1.0) / 2.0;
            }
            else
            {
                model(0) = -1.0 * acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(0) = atan(linear_accel.y() / linear_accel.z());
        }

        if(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()) == 0.0)
        {
            if(-1.0*linear_accel.x() > 0.0)
            {
                model(1) = acos(-1.0) / 2.0;
            }
            else
            {
                model(1) = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(1) = (-1.0 * linear_accel.x()) / atan(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()));
        }

        return model;
    }

    Vector3d createVec3(const double& x, const double& y, const double& z)
    {
        return Vector3d(x, y, z);
    }
}