#include "ekf_odom_estimater/odom_ekf.hpp"

namespace odom_ekf
{
    OdomEKF::OdomEKF()
    {
        x_position_ = vec3_t(0.0, 0.0, 0.0);
        x_velocity_ = vec3_t(0.0, 0.0, 0.0);
        x_quat_ = quat_t(1.0, 0.0, 0.0, 0.0);

        cov_ = mat_t<9,9>::Identity() * 0.01;
    }

    void OdomEKF::setEstNoise(const double& accel_var, const double& gyro_var, const double delta_time)
    {
        estimation_noise_.block<3,3>(0,0) = accel_var * mat3x3_t::Identity();
        estimation_noise_.block<3,3>(3,3) = gyro_var * mat3x3_t::Identity();
        estimation_noise_ *= delta_time * delta_time;
    }

    void OdomEKF::predictUpdate(vec3_t& imu_accel,vec3_t& imu_gyro, const double& delta_time)
    {
        const mat3x3_t last_rotation_matrix = x_quat_.toRotationMatrix();

        const vec3_t gravity = vec3_t(
            0.7453053593635559,
            0.22555294632911682,
            8.845598220825195);

        const vec3_t fixed_accel = last_rotation_matrix * imu_accel + gravity;

        x_position_ += delta_time * x_velocity_ + 0.5 * delta_time * delta_time * fixed_accel;
        x_velocity_ += delta_time * fixed_accel;

        x_quat_ = x_quat_ * gyro2quat(imu_gyro, delta_time);
        x_quat_.normalize();

        mat_t<9,9> F = mat_t<9,9>::Identity();
        F.block<3,3>(0,3) = delta_time * mat3x3_t::Identity();
        F.block<3,3>(3,6) = -1.0 * last_rotation_matrix * skew_symmetric(imu_accel) * delta_time;

        mat_t<9,6> l_jacob = mat_t<9,6>::Zero();
        l_jacob.block<6,6>(3,0) = mat_t<6,6>::Identity();

        cov_ = F * cov_ * F.transpose() + l_jacob * estimation_noise_ * l_jacob.transpose();
    }

    void OdomEKF::measurementUpdate(const mat_t<7,1>& observation, const double& obs_var)
    {
        mat_t<3,9> h_jacob = mat_t<3,9>::Zero();
        h_jacob.block<3,3>(0,0) = mat3x3_t::Identity();

        mat3x3_t observation_noise = obs_var * mat3x3_t::Identity();

        const mat_t<9,3> K = cov_ * h_jacob.transpose() * (h_jacob * cov_ * h_jacob.transpose() + observation_noise).inverse();

        vec3_t obs_pose = vec3_t(observation(0), observation(1), observation(2));

        const vec3_t residual = obs_pose - x_position_;
        mat_t<9,1> delta_x = K * residual;

        x_position_ += delta_x.segment<3>(0);
        x_velocity_ += delta_x.segment<3>(3);

        vec3_t d_theta = delta_x.segment<3>(6);

        x_quat_ = x_quat_ * angle2quat(d_theta);
        x_quat_.normalize();

        const mat_t<9,9> I = mat_t<9,9>::Identity();
        cov_ = (I - K * h_jacob) * cov_;   
    }

    mat_t<7,1> OdomEKF::getOdometry()
    {
        return mat_t<7,1>(
            x_position_(0),
            x_position_(1),
            x_position_(2),
            x_quat_.x(),
            x_quat_.y(),
            x_quat_.z(),
            x_quat_.w()
        );
    }

    mat3x3_t skew_symmetric(const vec3_t& v)
    {
        mat3x3_t m;

        m << 
            0, -v(2),  v(1),
            v(2),    0, -v(0),
            -v(1), v(0),    0;

        return m;
    }

    quat_t angle2quat(vec3_t& angle)
    {
        Eigen::AngleAxisd aa(angle.norm(), angle.normalized());

        return quat_t(aa);
    }

    quat_t gyro2quat(vec3_t& gyro, const double& delta_time)
    {
        Eigen::AngleAxisd aa(delta_time * gyro.norm(), gyro.normalized());

        return quat_t(aa);
    }
}