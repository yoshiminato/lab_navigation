#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

class ImuTestNode : public rclcpp::Node
{
public:
    ImuTestNode()
        : Node("orbslam3_imu_test"),
          count_(0),
          last_time_(-1.0)
    {
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mpu6050/imu",
            rclcpp::SensorDataQoS(),
            std::bind(
                &ImuTestNode::imuCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Waiting for /mpu6050/imu ...");
    }

private:
    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const double timestamp =
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

        if (last_time_ >= 0.0 &&
            timestamp <= last_time_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Non-increasing IMU timestamp: %.9f <= %.9f",
                timestamp,
                last_time_);
        }

        last_time_ = timestamp;
        count_++;

        // 50件ごと ≒ 約1秒ごとに表示
        if (count_ % 50 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "IMU #%zu time=%.9f  "
                "acc=[%.3f %.3f %.3f]  "
                "gyro=[%.4f %.4f %.4f]",
                count_,
                timestamp,

                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,

                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

    size_t count_;
    double last_time_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ImuTestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}