#include <lidar_localization/lidar_localization_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // NDT stays in the default mutually exclusive group. The publishing timer
  // has its own group so it can run while registration is busy.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  rclcpp::NodeOptions options;
  std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

  executor.add_node(pcl_l->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
