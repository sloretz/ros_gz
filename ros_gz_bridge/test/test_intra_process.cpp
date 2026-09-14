// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <gz/msgs/image.pb.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// The conversion must be declared before factory.hpp's template uses it.
#include "ros_gz_bridge/convert/sensor_msgs.hpp"
#include "factory.hpp"
#include "utils/gz_test_msg.hpp"
#include "utils/ros_test_msg.hpp"

using namespace std::chrono_literals;

namespace
{

// Exposes the protected gz -> ROS callback so it can be driven without gz-transport.
class ImageFactory : public ros_gz_bridge::Factory<sensor_msgs::msg::Image, gz::msgs::Image>
{
public:
  using Factory::Factory;
  using Factory::gz_callback;
};

}  // namespace

class IntraProcessTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    const auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    bridge_node_ = std::make_shared<rclcpp::Node>("bridge_node", options);
    sink_node_ = std::make_shared<rclcpp::Node>("sink_node", options);
  }

  void TearDown() override
  {
    sink_node_.reset();
    bridge_node_.reset();
    rclcpp::shutdown();
  }

  static int64_t WallTimeSec()
  {
    return std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  /// Pass the gz test Image through the bridge callback and return what an
  /// intra-process subscriber taking ownership received, or nullptr.
  std::unique_ptr<sensor_msgs::msg::Image> BridgeTestImage(
    const std::string & topic,
    const ros_gz_bridge::BridgeHandleGzToRosParameters & params)
  {
    ImageFactory factory("sensor_msgs/msg/Image", "gz.msgs.Image");
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(
      factory.create_ros_publisher(bridge_node_, topic, rclcpp::QoS(1)));
    if (pub == nullptr) {
      ADD_FAILURE() << "Factory did not create a sensor_msgs/Image publisher";
      return nullptr;
    }

    std::promise<std::unique_ptr<sensor_msgs::msg::Image>> received;
    auto future = received.get_future();
    auto sub = sink_node_->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::QoS(1),
      [&received](
        std::unique_ptr<sensor_msgs::msg::Image> msg,
        const rclcpp::MessageInfo & info)
      {
        EXPECT_TRUE(info.get_rmw_message_info().from_intra_process);
        received.set_value(std::move(msg));
      });
    EXPECT_EQ(1u, pub->get_intra_process_subscription_count());

    gz::msgs::Image gz_msg;
    ros_gz_bridge::testing::createTestMsg(gz_msg);
    ImageFactory::gz_callback(gz_msg, pub, params);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sink_node_);
    const auto result = executor.spin_until_future_complete(future, 5s);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "The intra-process subscriber received no message";
      return nullptr;
    }
    return future.get();
  }

  rclcpp::Node::SharedPtr bridge_node_;
  rclcpp::Node::SharedPtr sink_node_;
};

// A gz Image bridged by a node with intra-process comms enabled reaches an
// intra-process subscriber unchanged.
TEST_F(IntraProcessTest, DeliversImageToIntraProcessSubscriber)
{
  auto ros_msg = BridgeTestImage("/intra_process_image", {});
  ASSERT_NE(nullptr, ros_msg);
  ros_gz_bridge::testing::compareTestMsg(
    std::shared_ptr<sensor_msgs::msg::Image>(std::move(ros_msg)));
}

// The header overrides still apply to the message published by unique_ptr.
TEST_F(IntraProcessTest, AppliesHeaderOverrides)
{
  ros_gz_bridge::BridgeHandleGzToRosParameters params;
  params.override_timestamps_with_wall_time = true;
  params.override_frame_id = "overridden_frame";

  const auto before = WallTimeSec();
  auto ros_msg = BridgeTestImage("/intra_process_image_overrides", params);
  const auto after = WallTimeSec();

  ASSERT_NE(nullptr, ros_msg);
  EXPECT_EQ("overridden_frame", ros_msg->header.frame_id);
  EXPECT_GE(ros_msg->header.stamp.sec, before);
  EXPECT_LE(ros_msg->header.stamp.sec, after);
}
