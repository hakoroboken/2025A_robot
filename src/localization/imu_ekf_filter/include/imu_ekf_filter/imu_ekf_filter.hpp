#ifndef IMU_EKF_FILTER_HPP_
#define IMU_EKF_FILTER_HPP_

#include "ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

using std::placeholders::_1;

namespace imu_ekf_filter
{
    class ImuEkfFilter : public rclcpp::Node
    {
        public:
        explicit ImuEkfFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

        std::shared_ptr<ekf::EKF> ekf_;
        rclcpp::Time last_time_;

        std::string frame_id_;
        bool deg_to_rad_;

    };
}

#endif