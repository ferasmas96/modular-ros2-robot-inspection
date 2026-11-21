#include <glim_pose_output/pose_output.hpp>

#include <glim/odometry/callbacks.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace glim_pose_output {
class PoseExtension final : public glim::ExtensionModuleROS2 {
public:
  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override {
    if (state_) throw std::logic_error("pose extension already initialized");
    const auto T_base_imu = PoseOutput::load_base_extrinsic();
    // Latest-pose telemetry: no reliable-delivery acknowledgement wait or history replay.
    auto pub = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      kTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());
    state_ = std::make_shared<PoseOutput>(T_base_imu, [pub](const auto& msg) { pub->publish(msg); });
    const auto consume = PoseOutput::callback(state_);
    // Registered before host sensor subscriptions are created. GLIM's callback list
    // is unsynchronized: do not mutate it at teardown while its worker may run.
    // The host keeps dlopen handles for process lifetime; an expired weak state is a no-op.
    glim::OdometryEstimationCallbacks::on_update_new_frame.add(
      [consume, logger = node.get_logger(), rejected = uint64_t{0}](const auto& frame) mutable {
        try {
          consume(frame);
        } catch (const std::exception& e) {
          if (++rejected == 1 || rejected % 100 == 0) {
            RCLCPP_WARN(logger, "GLIM pose output rejected frame/publication (%lu total): %s",
                        static_cast<unsigned long>(rejected), e.what());
          }
        }
      });
    RCLCPP_INFO(node.get_logger(), "TF-free local base pose: %s, reference=%s, best_effort/volatile depth=10",
                kTopic, kReferenceFrame);
    return {};
  }

private:
  std::shared_ptr<PoseOutput> state_;
};
}  // namespace glim_pose_output

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_pose_output::PoseExtension;
}
