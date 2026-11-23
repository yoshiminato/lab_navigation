#include <rclcpp/rclcpp.hpp>
#include <System.h>

#include <memory>
#include <string>

#include <cstdlib>
#include <stdexcept>
#include <string>

static std::string lab_orbslam3_dir() {
    const char* custom = std::getenv("ORB_SLAM3_DIR");
    if (custom && *custom) {
        return std::string(custom);
    }
    const char* home = std::getenv("HOME");
    if (home && *home) {
        return std::string(home) + "/ORB_SLAM3";
    }
    throw std::runtime_error("Set ORB_SLAM3_DIR or HOME");
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("orbslam3_smoke_test");

    const std::string voc_file =
        node->declare_parameter<std::string>(
            "voc_file",
            lab_orbslam3_dir() + "/Vocabulary/ORBvoc.txt");

    const std::string settings_file =
        node->declare_parameter<std::string>(
            "settings_file",
            lab_orbslam3_dir() + "/Examples/Monocular-Inertial/robot.yaml");

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
