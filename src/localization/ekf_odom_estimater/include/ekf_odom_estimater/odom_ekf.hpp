#ifndef ODOM_EKF_HPP_
#define ODOM_EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace odom_ekf
{
    typedef Vector3d vec3_t;
    typedef Matrix3d mat3x3_t;
    typedef Quaterniond quat_t;

    template<size_t row, size_t column>
    using mat_t = Matrix<double, row, column>;

    class OdomEKF
    {    
        public:
        OdomEKF();

        void setEstNoise(const double& accel_var, const double& gyro_var, const double delta_time);
        void predictUpdate(vec3_t& imu_accel,vec3_t& imu_gyro, const double& delta_time);
        void measurementUpdate(const mat_t<7,1>& observation, const double& obs_var);

        mat_t<7,1> getOdometry();

        private:
        vec3_t x_position_, x_velocity_;
        quat_t x_quat_;
        mat_t<9,9> cov_;
        mat_t<6,6> estimation_noise_;
    };

    mat3x3_t skew_symmetric(const vec3_t& v);
    quat_t angle2quat(vec3_t& angle);
    quat_t gyro2quat(vec3_t& gyro, const double& delta_time);
}

#endif