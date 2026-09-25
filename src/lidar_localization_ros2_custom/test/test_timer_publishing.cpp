#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "lidar_localization/lidar_localization_component.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class TimerPublishingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  bool waitUntil(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/timer_publish_test",
      "-r", "/tf:=/timer_publish_test/tf", "-r", "/tf_static:=/timer_publish_test/tf_static"});
    options.parameter_overrides({
      {"enable_timer_publishing", true}, {"pose_publish_frequency", 30.0},
      {"enable_map_odom_tf", true}, {"set_initial_pose", false},
      {"use_pcd_map", false}, {"use_odom", true}});
    localization_ = std::make_shared<PCLLocalization>(options);
    localization_->configure();
    observer_ = std::make_shared<rclcpp::Node>("observer", "/timer_publish_test");
    tf_sub_ = observer_->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", rclcpp::QoS(100), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        for (const auto & transform : msg->transforms) {
          if (transform.header.frame_id == "map" && transform.child_frame_id == "odom") {
            transforms_.push_back(transform);
          }
        }
      });
    pose_sub_ = observer_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        poses_.push_back(*msg);
      });
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor_->add_node(localization_->get_node_base_interface());
    executor_->add_node(observer_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
    localization_->activate();
    ASSERT_TRUE(waitUntil([this]() {return tf_sub_->get_publisher_count() > 0;}));
    source_stamp_ = localization_->now() - rclcpp::Duration::from_seconds(0.2);
    setOdom(2.0, source_stamp_);
    setPose(5.0, source_stamp_);
    ASSERT_TRUE(waitUntil([this]() {return transformCount() >= 2;}));
  }

  void TearDown() override
  {
    if (blocking_timer_) {blocking_timer_->cancel();}
    if (executor_) {executor_->cancel();}
    if (spin_thread_.joinable()) {spin_thread_.join();}
    if (localization_) {localization_->deactivate();}
    executor_.reset();
    blocking_timer_.reset();
    tf_sub_.reset();
    pose_sub_.reset();
    observer_.reset();
    localization_.reset();
  }

  void setOdom(double x, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.header.stamp = stamp;
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(localization_->tfbuffer_.setTransform(transform, "test", false));
  }

  void setPose(double x, const rclcpp::Time & stamp, double yaw = 0.0)
  {
    auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose->header.frame_id = "map";
    pose->header.stamp = stamp;
    pose->pose.pose.position.x = x;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);
    pose->pose.pose.orientation = tf2::toMsg(rotation);
    pose->pose.covariance[0] = 0.75;
    localization_->initialPoseReceived(pose);
  }

  size_t transformCount()
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return transforms_.size();
  }

  geometry_msgs::msg::TransformStamped latestTransform()
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return transforms_.empty() ? geometry_msgs::msg::TransformStamped() : transforms_.back();
  }

  geometry_msgs::msg::PoseWithCovarianceStamped latestPose()
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return poses_.empty() ? geometry_msgs::msg::PoseWithCovarianceStamped() : poses_.back();
  }

  std::shared_ptr<PCLLocalization> localization_;
  rclcpp::Node::SharedPtr observer_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr blocking_timer_;
  std::thread spin_thread_;
  std::mutex received_mutex_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> poses_;
  rclcpp::Time source_stamp_{0, 0, RCL_ROS_TIME};
};

TEST_F(TimerPublishingTest, RetainsCorrectionAndPropagatesOdometryIncludingRotation)
{
  setPose(5.0, source_stamp_, M_PI / 2.0);
  const auto odom_stamp = localization_->now();
  setOdom(4.0, odom_stamp);
  ASSERT_TRUE(waitUntil([this]() {return std::abs(latestPose().pose.pose.position.y - 2.0) < 1e-6;}));
  const auto transform = latestTransform();
  EXPECT_NEAR(transform.transform.translation.x, 5.0, 1e-6);
  EXPECT_NEAR(transform.transform.translation.y, -2.0, 1e-6);
  EXPECT_GT(rclcpp::Time(transform.header.stamp), source_stamp_);
  const auto pose = latestPose();
  EXPECT_NEAR(pose.pose.pose.position.x, 5.0, 1e-6);
  EXPECT_EQ(rclcpp::Time(pose.header.stamp), odom_stamp);
  EXPECT_DOUBLE_EQ(pose.pose.covariance[0], 0.75);
}

TEST_F(TimerPublishingTest, PublishesWhileTheRegistrationCallbackGroupIsBlocked)
{
  auto entered = std::make_shared<std::atomic<bool>>(false);
  auto finished = std::make_shared<std::atomic<bool>>(false);
  // Emulate a slow NDT callback in the same mutually exclusive group.
  blocking_timer_ = localization_->create_wall_timer(1ms, [entered, finished]() {
    if (entered->exchange(true)) {return;}
    std::this_thread::sleep_for(500ms);
    finished->store(true);
  });
  ASSERT_TRUE(waitUntil([&]() {return entered->load();}));
  const auto count = transformCount();
  const auto stamp = latestTransform().header.stamp;
  std::this_thread::sleep_for(250ms);
  EXPECT_FALSE(finished->load());
  EXPECT_GE(transformCount(), count + 4);
  EXPECT_GT(rclcpp::Time(latestTransform().header.stamp), rclcpp::Time(stamp));
  EXPECT_NEAR(latestTransform().transform.translation.x, 3.0, 1e-6);
  EXPECT_TRUE(waitUntil([&]() {return finished->load();}));
  blocking_timer_->cancel();
}

TEST_F(TimerPublishingTest, ManualPoseResetReplacesTheCachedCorrection)
{
  const auto stamp = localization_->now();
  setOdom(4.0, stamp);
  setPose(20.0, stamp);
  ASSERT_TRUE(waitUntil([this]() {
    return std::abs(latestTransform().transform.translation.x - 16.0) < 1e-6 &&
           std::abs(latestPose().pose.pose.position.x - 20.0) < 1e-6;
  }));
  EXPECT_FALSE(localization_->odom_time_initialized_);
}

TEST_F(TimerPublishingTest, DeactivationStopsPublicationAndReactivationResumesIt)
{
  localization_->deactivate();
  std::this_thread::sleep_for(100ms);  // Drain already delivered DDS messages.
  const auto count = transformCount();
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(transformCount(), count);
  localization_->activate();
  EXPECT_TRUE(waitUntil([this, count]() {return transformCount() >= count + 2;}));
}
