#include <rclcpp/rclcpp.hpp>
#include <System.h>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("orbslam3_smoke_test");

    const std::string voc_file =
        node->declare_parameter<std::string>(
            "voc_file",
            "/home/user/ORB_SLAM3/Vocabulary/ORBvoc.txt");

    const std::string settings_file =
        node->declare_parameter<std::string>(
            "settings_file",
            "/home/user/ORB_SLAM3/Examples/Monocular-Inertial/robot.yaml");

    RCLCPP_INFO(node->get_logger(), "Vocabulary: %s", voc_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Settings  : %s", settings_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 IMU_MONOCULAR...");

    try
    {
        ORB_SLAM3::System slam(
            voc_file,
            settings_file,
            ORB_SLAM3::System::IMU_MONOCULAR,
            false);

        RCLCPP_INFO(
            node->get_logger(),
            "ORB-SLAM3 initialization OK");

        slam.Shutdown();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "ORB-SLAM3 initialization failed: %s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Smoke test completed.");

    rclcpp::shutdown();

    return 0;
}
