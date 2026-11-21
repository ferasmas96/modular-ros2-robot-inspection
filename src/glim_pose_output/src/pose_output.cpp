#include <glim_pose_output/pose_output.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <glim/util/config.hpp>

namespace glim_pose_output {
namespace {
void require_rigid(const Eigen::Isometry3d& pose) {
  const auto& R = pose.linear();
  if (!pose.matrix().allFinite() ||
      !(R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
      std::abs(R.determinant() - 1.0) > 1e-8 ||
      !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8)) {
    throw std::invalid_argument("pose is not a finite rigid transform");
  }
}

builtin_interfaces::msg::Time sensor_time(double stamp) {
  // Round the estimator's double seconds once; do not substitute wall/ROS now().
  if (!std::isfinite(stamp) || stamp < 0.0 ||
      stamp >= static_cast<double>(std::numeric_limits<int32_t>::max()) + 1.0) {
    throw std::invalid_argument("sensor timestamp outside ROS Time range");
  }
  const int64_t ns = std::llround(stamp * 1e9);
  const int64_t sec = ns / 1000000000LL;
  if (sec > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument("rounded sensor timestamp outside ROS Time range");
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(sec);
  result.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return result;
}
}  // namespace

PoseOutput::PoseOutput(const Eigen::Isometry3d& T_base_imu, Sink sink)
: T_imu_base_([&] { require_rigid(T_base_imu); return Eigen::Isometry3d(T_base_imu.inverse()); }()),
  sink_(std::move(sink)) {
  if (!sink_) throw std::invalid_argument("pose output requires a sink");
}

Eigen::Isometry3d PoseOutput::load_base_extrinsic() {
  const glim::Config config(glim::GlobalConfig::get_config_path("config_pose_output"));
  const auto values = config.param<std::vector<double>>("pose_output", "T_base_imu");
  if (!values || values->size() != 7) {
    throw std::invalid_argument("pose_output.T_base_imu must contain seven TUM values");
  }
  for (const double value : *values) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite base extrinsic");
  }
  const Eigen::Quaterniond q((*values)[6], (*values)[3], (*values)[4], (*values)[5]);
  if (std::abs(q.norm() - 1.0) > 1e-8) {
    throw std::invalid_argument("base extrinsic quaternion must be unit length");
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d((*values)[0], (*values)[1], (*values)[2]);
  T.linear() = q.toRotationMatrix();
  require_rigid(T);
  return T;
}

void PoseOutput::consume(const glim::EstimationFrame::ConstPtr& frame) const {
  if (!frame) throw std::invalid_argument("null estimation frame");
  // Read/copy inside the estimator callback; never retain a mutable GLIM frame.
  const auto stamp = sensor_time(frame->stamp);
  const Eigen::Isometry3d T_odom_imu = frame->T_world_imu;
  require_rigid(T_odom_imu);
  const Eigen::Isometry3d T_odom_base = T_odom_imu * T_imu_base_;
  require_rigid(T_odom_base);
  const Eigen::Quaterniond q(T_odom_base.linear());
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = kReferenceFrame;
  msg.pose.position.x = T_odom_base.translation().x();
  msg.pose.position.y = T_odom_base.translation().y();
  msg.pose.position.z = T_odom_base.translation().z();
  const auto unit_q = q.normalized();
  msg.pose.orientation.x = unit_q.x();
  msg.pose.orientation.y = unit_q.y();
  msg.pose.orientation.z = unit_q.z();
  msg.pose.orientation.w = unit_q.w();
  sink_(msg);
}

PoseOutput::Callback PoseOutput::callback(const std::shared_ptr<PoseOutput>& state) {
  return [weak = std::weak_ptr<PoseOutput>(state)](const glim::EstimationFrame::ConstPtr& frame) {
    if (const auto locked = weak.lock()) locked->consume(frame);
  };
}
}  // namespace glim_pose_output
