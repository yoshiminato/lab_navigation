#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <System.h>

#include <memory>
#include <string>

class MonoNode : public rclcpp::Node
{
public:
    MonoNode()
        : Node("orbslam3_mono")
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
            declare_parameter<bool>(
                "viewer",
                true);

        RCLCPP_INFO(
            get_logger(),
            "Vocabulary: %s",
            voc_file.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Settings  : %s",
            settings_file.c_str());

        slam_ = std::make_unique<ORB_SLAM3::System>(
            voc_file,
            settings_file,
            ORB_SLAM3::System::MONOCULAR,
            viewer);

        image_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &MonoNode::imageCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Waiting for /camera/image_raw ...");
    }

    ~MonoNode()
    {
        if (slam_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Shutting down ORB-SLAM3...");

            slam_->Shutdown();
        }
    }

private:
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const double timestamp =
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

        cv::Mat image;

        try
        {
            auto cv_ptr =
                cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::RGB8);

            image = cv_ptr->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "cv_bridge error: %s",
                e.what());

            return;
        }

        frame_count_++;

        RCLCPP_INFO(
            get_logger(),
            "Frame #%zu  time=%.6f",
            frame_count_,
            timestamp);

        slam_->TrackMonocular(
            image,
            timestamp);
    }

    std::unique_ptr<ORB_SLAM3::System> slam_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    size_t frame_count_ = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MonoNode>();

    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();

    return 0;
}
