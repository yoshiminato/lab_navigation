#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <System.h>
#include <ImuTypes.h>

#include <deque>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <memory>
#include <string>

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode()
        : Node("orbslam3_mono_inertial"),
          running_(true)
    {
        const std::string voc_file =
            declare_parameter<std::string>(
                "voc_file",
                "/home/user/ORB_SLAM3/Vocabulary/ORBvoc.txt");

        const std::string settings_file =
            declare_parameter<std::string>(
                "settings_file",
                "/home/user/ORB_SLAM3/Examples/Monocular-Inertial/robot.yaml");

        const bool viewer =
            declare_parameter<bool>("viewer", true);

        RCLCPP_INFO(get_logger(), "Vocabulary: %s", voc_file.c_str());
        RCLCPP_INFO(get_logger(), "Settings  : %s", settings_file.c_str());

        slam_ = std::make_unique<ORB_SLAM3::System>(
            voc_file,
            settings_file,
            ORB_SLAM3::System::IMU_MONOCULAR,
            viewer);

        image_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &MonoInertialNode::imageCallback,
                    this,
                    std::placeholders::_1));

        imu_sub_ =
            create_subscription<sensor_msgs::msg::Imu>(
                "/mpu6050/imu",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &MonoInertialNode::imuCallback,
                    this,
                    std::placeholders::_1));

        worker_ =
            std::thread(
                &MonoInertialNode::processLoop,
                this);

        RCLCPP_INFO(
            get_logger(),
            "Waiting for /camera/image_raw and /mpu6050/imu ...");
    }

    ~MonoInertialNode()
    {
        running_ = false;
        cv_.notify_all();

        if (worker_.joinable())
            worker_.join();

        if (slam_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Shutting down ORB-SLAM3...");

            slam_->Shutdown();
        }
    }

private:
    static double stampToSec(
        const builtin_interfaces::msg::Time &stamp)
    {
        return
            static_cast<double>(stamp.sec) +
            static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            image_buffer_.push_back(msg);
        }

        cv_.notify_one();
    }

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            imu_buffer_.push_back(msg);
        }

        cv_.notify_one();
    }

    void processLoop()
    {
        double last_image_time = -1.0;
        size_t image_count = 0;

        while (running_ && rclcpp::ok())
        {
            sensor_msgs::msg::Image::SharedPtr image_msg;

            std::vector<ORB_SLAM3::IMU::Point> imu_measurements;

            double image_time = 0.0;

            {
                std::unique_lock<std::mutex> lock(mutex_);

                cv_.wait(
                    lock,
                    [this]()
                    {
                        if (!running_)
                            return true;

                        if (image_buffer_.empty() ||
                            imu_buffer_.empty())
                            return false;

                        const double image_time =
                            stampToSec(
                                image_buffer_.front()->header.stamp);

                        const double latest_imu_time =
                            stampToSec(
                                imu_buffer_.back()->header.stamp);

                        // この画像時刻までIMUが到着するまで待つ
                        return latest_imu_time >= image_time;
                    });

                if (!running_)
                    break;

                image_msg = image_buffer_.front();
                image_buffer_.pop_front();

                image_time =
                    stampToSec(image_msg->header.stamp);

                if (last_image_time >= 0.0 &&
                    image_time <= last_image_time)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Non-increasing image timestamp");

                    continue;
                }

                while (!imu_buffer_.empty())
                {
                    const auto &imu_msg =
                        imu_buffer_.front();

                    const double imu_time =
                        stampToSec(
                            imu_msg->header.stamp);

                    if (imu_time > image_time)
                        break;

                    if (last_image_time < 0.0 ||
                        imu_time > last_image_time)
                    {
                        imu_measurements.emplace_back(
                            static_cast<float>(
                                imu_msg->linear_acceleration.x),

                            static_cast<float>(
                                imu_msg->linear_acceleration.y),

                            static_cast<float>(
                                imu_msg->linear_acceleration.z),

                            static_cast<float>(
                                imu_msg->angular_velocity.x),

                            static_cast<float>(
                                imu_msg->angular_velocity.y),

                            static_cast<float>(
                                imu_msg->angular_velocity.z),

                            imu_time);
                    }

                    imu_buffer_.pop_front();
                }
            }

            cv::Mat image;

            try
            {
                auto cv_ptr =
                    cv_bridge::toCvCopy(
                        image_msg,
                        sensor_msgs::image_encodings::RGB8);

                image = cv_ptr->image;
            }
            catch (const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "cv_bridge error: %s",
                    e.what());

                continue;
            }

            image_count++;

            RCLCPP_INFO(
                get_logger(),
                "Frame #%zu  time=%.6f  IMU=%zu",
                image_count,
                image_time,
                imu_measurements.size());

            slam_->TrackMonocular(
                image,
                image_time,
                imu_measurements);

            last_image_time = image_time;
        }
    }

private:
    std::unique_ptr<ORB_SLAM3::System> slam_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::deque<
        sensor_msgs::msg::Image::SharedPtr> image_buffer_;

    std::deque<
        sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    std::mutex mutex_;
    std::condition_variable cv_;

    std::thread worker_;
    std::atomic<bool> running_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MonoInertialNode>();

    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();

    return 0;
}