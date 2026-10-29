#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class ImageTestNode : public rclcpp::Node
{
public:
    ImageTestNode()
        : Node("orbslam3_image_test")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &ImageTestNode::imageCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Waiting for /camera/image_raw ...");
    }

private:
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(
                    msg,
                    "rgb8");

            const cv::Mat &image = cv_ptr->image;

            double timestamp =
                static_cast<double>(msg->header.stamp.sec) +
                static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

            RCLCPP_INFO(
                get_logger(),
                "Image received: %dx%d  encoding=%s  time=%.9f",
                image.cols,
                image.rows,
                msg->encoding.c_str(),
                timestamp);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "cv_bridge error: %s",
                e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ImageTestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
