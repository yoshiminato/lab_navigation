#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <deque>
#include <mutex>
#include <memory>

class SyncTestNode : public rclcpp::Node
{
public:
    SyncTestNode()
        : Node("orbslam3_sync_test"),
          last_image_time_(-1.0),
          image_count_(0)
    {
        image_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SyncTestNode::imageCallback,
                    this,
                    std::placeholders::_1));

        imu_sub_ =
            create_subscription<sensor_msgs::msg::Imu>(
                "/mpu6050/imu",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SyncTestNode::imuCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Waiting for camera and IMU...");
    }

private:
    double stampToSec(
        const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(stamp.sec)
             + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        image_buffer_.push_back(msg);

        tryProcess();
    }

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        imu_buffer_.push_back(msg);

        tryProcess();
    }

    void tryProcess()
    {
        while (!image_buffer_.empty() &&
               !imu_buffer_.empty())
        {
            const double image_time =
                stampToSec(
                    image_buffer_.front()->header.stamp);

            const double latest_imu_time =
                stampToSec(
                    imu_buffer_.back()->header.stamp);

            // この画像時刻までIMUが届いていなければ待つ
            if (latest_imu_time < image_time)
                return;

            size_t imu_count = 0;

            while (!imu_buffer_.empty())
            {
                const double imu_time =
                    stampToSec(
                        imu_buffer_.front()->header.stamp);

                if (imu_time > image_time)
                    break;

                if (last_image_time_ < 0.0 ||
                    imu_time > last_image_time_)
                {
                    imu_count++;
                }

                imu_buffer_.pop_front();
            }

            image_count_++;

            RCLCPP_INFO(
                get_logger(),
                "Image #%zu  time=%.6f  IMU samples=%zu",
                image_count_,
                image_time,
                imu_count);

            last_image_time_ = image_time;

            image_buffer_.pop_front();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::deque<sensor_msgs::msg::Image::SharedPtr> image_buffer_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    std::mutex mutex_;

    double last_image_time_;
    size_t image_count_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SyncTestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}