#include <glim_pose_output/pose_output.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>
#include <glim/util/config.hpp>

using glim_pose_output::PoseOutput;
namespace {
glim::EstimationFrame::Ptr frame(double stamp = 12.125) {
  auto f = std::make_shared<glim::EstimationFrame>();
  f->stamp = stamp;
  f->T_world_imu = Eigen::Isometry3d::Identity();
  return f;
}
Eigen::Isometry3d extrinsic() {
  auto T = Eigen::Isometry3d::Identity();
  T.translation().z() = 0.294759;
  return T;
}
}

TEST(PoseOutput, VerifiedBaseOffsetAndSensorStamp) {
  geometry_msgs::msg::PoseStamped msg;
  PoseOutput output(extrinsic(), [&](const auto& m) { msg = m; });
  auto f = frame();
  f->T_world_imu.translation() = Eigen::Vector3d(1, 2, 0.294759);
  const auto before = f->T_world_imu.matrix().eval();
  output.consume(f);
  EXPECT_EQ(msg.header.frame_id, "glim_odom");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 125000000U);
  EXPECT_DOUBLE_EQ(msg.pose.position.x, 1);
  EXPECT_DOUBLE_EQ(msg.pose.position.y, 2);
  EXPECT_NEAR(msg.pose.position.z, 0, 1e-12);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 1);
  EXPECT_TRUE(before.isApprox(f->T_world_imu.matrix(), 0));
}

TEST(PoseOutput, NoncommutingRotationsAndTranslationUseCorrectInverse) {
  auto T_base_imu = extrinsic();
  T_base_imu.translation() = Eigen::Vector3d(0.3, -0.2, 0.8);
  T_base_imu.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(4, -3, 0.1);
  auto f = frame();
  f->T_world_imu = expected * T_base_imu;
  PoseOutput output(T_base_imu, [&](const auto& msg) {
    const auto& q = msg.pose.orientation;
    auto actual = Eigen::Isometry3d::Identity();
    actual.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    actual.translation() = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    EXPECT_TRUE(expected.matrix().isApprox(actual.matrix(), 1e-12));
    EXPECT_NEAR(Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm(), 1, 1e-12);
  });
  output.consume(f);
}

TEST(PoseOutput, NanosecondRoundingCarriesIntoNextSecond) {
  PoseOutput output(extrinsic(), [](const auto& msg) {
    EXPECT_EQ(msg.header.stamp.sec, 2);
    EXPECT_EQ(msg.header.stamp.nanosec, 0U);
  });
  output.consume(frame(1.9999999996));
}

TEST(PoseOutput, RejectsInvalidAndOutOfRangeTimestamps) {
  int sent = 0;
  PoseOutput output(extrinsic(), [&](const auto&) { ++sent; });
  for (double stamp : {-0.01, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(), 2147483648.0}) {
    EXPECT_THROW(output.consume(frame(stamp)), std::invalid_argument);
  }
  EXPECT_EQ(sent, 0);
}

TEST(PoseOutput, AcceptsZeroTimestampWithoutUsingWallTime) {
  PoseOutput output(extrinsic(), [](const auto& msg) {
    EXPECT_EQ(msg.header.stamp.sec, 0);
    EXPECT_EQ(msg.header.stamp.nanosec, 0U);
  });
  output.consume(frame(0));
}

TEST(PoseOutput, RejectsNullNonfiniteAndNonrigidPoses) {
  int sent = 0;
  PoseOutput output(extrinsic(), [&](const auto&) { ++sent; });
  EXPECT_THROW(output.consume(nullptr), std::invalid_argument);
  auto f = frame();
  f->T_world_imu.translation().x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(output.consume(f), std::invalid_argument);
  f = frame(); f->T_world_imu.linear()(0, 0) = 2;
  EXPECT_THROW(output.consume(f), std::invalid_argument);
  f = frame(); f->T_world_imu.linear()(0, 0) = -1;
  EXPECT_THROW(output.consume(f), std::invalid_argument);
  f = frame(); f->T_world_imu.matrix()(3, 0) = 1;
  EXPECT_THROW(output.consume(f), std::invalid_argument);
  EXPECT_EQ(sent, 0);
}

TEST(PoseOutput, RejectsInvalidExtrinsicAndEmptySink) {
  auto invalid = extrinsic(); invalid.linear()(0, 0) = 0;
  EXPECT_THROW((PoseOutput(invalid, [](const auto&) {})), std::invalid_argument);
  EXPECT_THROW((PoseOutput(extrinsic(), {})), std::invalid_argument);
}

TEST(PoseOutput, WeakCallbackDoesNotRetainStateOrFrames) {
  int sent = 0;
  auto state = std::make_shared<PoseOutput>(extrinsic(), [&](const auto&) { ++sent; });
  auto cb = PoseOutput::callback(state);
  auto f = frame();
  cb(f);
  EXPECT_EQ(f.use_count(), 1);
  state.reset();
  EXPECT_NO_THROW(cb(f));
  EXPECT_EQ(sent, 1);
}

TEST(PoseOutput, LoadsActualSnapshotUsingInstalledGlimConfigApi) {
  const char* path = std::getenv("GLIM_POSE_OUTPUT_TEST_CONFIG");
  ASSERT_NE(path, nullptr);
  glim::GlobalConfig::instance(path, true);
  EXPECT_TRUE(PoseOutput::load_base_extrinsic().matrix().isApprox(extrinsic().matrix(), 1e-12));
}

TEST(PoseOutput, NativeConfigRejectsMissingMalformedOrNonunitExtrinsic) {
  char pattern[] = "/tmp/glim-pose-config-test-XXXXXX";
  const char* directory = mkdtemp(pattern);
  ASSERT_NE(directory, nullptr);
  const std::filesystem::path root(directory);
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::filesystem::remove_all(path); }
  } cleanup{root};
  std::ofstream(root / "config.json") << R"({"global":{"config_pose_output":"pose.json"}})";
  for (const auto& invalid : {
    R"({"pose_output":{}})",
    R"({"pose_output":{"T_base_imu":[0,0,0]}})",
    R"({"pose_output":{"T_base_imu":[0,0,0,0,0,0,2]}})",
    R"({"pose_output":{"T_base_imu":[0,0,0,0,0,0,0]}})"}) {
    std::ofstream(root / "pose.json") << invalid;
    glim::GlobalConfig::instance(root.string(), true);
    EXPECT_THROW(PoseOutput::load_base_extrinsic(), std::invalid_argument);
  }
}
