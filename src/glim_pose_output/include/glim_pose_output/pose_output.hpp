#pragma once

#include <functional>
#include <memory>
#include <string>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <glim/odometry/estimation_frame.hpp>

namespace glim_pose_output {

inline constexpr char kTopic[] = "/a300_00000/slam/glim/pose";
inline constexpr char kReferenceFrame[] = "glim_odom";

// All transforms use T_parent_child: p_parent = T_parent_child * p_child.
class PoseOutput {
public:
  using Sink = std::function<void(const geometry_msgs::msg::PoseStamped&)>;
  using Callback = std::function<void(const glim::EstimationFrame::ConstPtr&)>;

  explicit PoseOutput(const Eigen::Isometry3d& T_base_imu, Sink sink);
  void consume(const glim::EstimationFrame::ConstPtr& frame) const;
  static Callback callback(const std::shared_ptr<PoseOutput>& state);
  static Eigen::Isometry3d load_base_extrinsic();

private:
  const Eigen::Isometry3d T_imu_base_;
  const Sink sink_;
};

}  // namespace glim_pose_output
