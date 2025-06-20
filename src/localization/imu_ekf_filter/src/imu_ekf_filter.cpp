#include "imu_ekf_filter/imu_ekf_filter.hpp"

namespace imu_ekf_filter
{
    ImuEkfFilter::ImuEkfFilter(const rclcpp::NodeOptions& options) : Node("ImuEkfFilter", options)
    {
        frame_id_ = this->declare_parameter("frame_id", "imu");
        deg_to_rad_ = this->declare_parameter("degree_to_radian", false);

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", rclcpp::SystemDefaultsQoS());
        
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/raw_data",
            0,
            std::bind(&ImuEkfFilter::imu_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Start %s", this->get_name());
    }

    void ImuEkfFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(last_time_.seconds() > 0.0)
        {
            const auto delta_time = this->get_clock()->now() - last_time_;

            const auto accel = ekf::createVec3(
                msg->linear_acceleration.x, 
                msg->linear_acceleration.y, 
                msg->linear_acceleration.z
            );

            double gyro_x,gyro_y,gyro_z;
            if(deg_to_rad_)
            {
                gyro_x = msg->angular_velocity.x * (M_PI / 180.0);
                gyro_y = msg->angular_velocity.y * (M_PI / 180.0);
                gyro_z = msg->angular_velocity.z * (M_PI / 180.0);
            }
            else
            {
                gyro_x = msg->angular_velocity.x;
                gyro_y = msg->angular_velocity.y;
                gyro_z = msg->angular_velocity.z;
            }

            const auto gyro = ekf::createVec3(gyro_x, gyro_y, gyro_z);

            const auto quat = ekf_->estimate(gyro, accel, delta_time.seconds());

            auto result = sensor_msgs::msg::Imu();
            result.header.frame_id = frame_id_;
            result.header.stamp = this->get_clock()->now();

            result.angular_velocity_covariance = msg->angular_velocity_covariance;
            result.angular_velocity.x = gyro_x;
            result.angular_velocity.y = gyro_y;
            result.angular_velocity.z = gyro_z;
            
            result.linear_acceleration_covariance = msg->linear_acceleration_covariance;
            result.linear_acceleration.x = accel.x();
            result.linear_acceleration.y = accel.y();
            result.linear_acceleration.z = accel.z();

            result.orientation_covariance = msg->angular_velocity_covariance;
            result.orientation.w = quat.w();
            result.orientation.x = quat.x();
            result.orientation.y = quat.y();
            result.orientation.z = quat.z();

            imu_publisher_->publish(result);
        }

        last_time_ = this->get_clock()->now();
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_ekf_filter::ImuEkfFilter)