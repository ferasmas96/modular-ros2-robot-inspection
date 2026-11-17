// Headless validation only: use native GLIM loaders, never save a map.
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/landlock.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/util/config.hpp>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Fail closed before loading: no file creation, deletion, write-open or truncation.
static long deny_filesystem_writes() {
  const long abi = syscall(SYS_landlock_create_ruleset, nullptr, 0,
                           LANDLOCK_CREATE_RULESET_VERSION);
  require(abi >= 3, "Landlock ABI >= 3 required; refusing unprotected load");
  landlock_ruleset_attr rules{};
  rules.handled_access_fs = LANDLOCK_ACCESS_FS_WRITE_FILE |
    LANDLOCK_ACCESS_FS_REMOVE_DIR | LANDLOCK_ACCESS_FS_REMOVE_FILE |
    LANDLOCK_ACCESS_FS_MAKE_CHAR | LANDLOCK_ACCESS_FS_MAKE_DIR |
    LANDLOCK_ACCESS_FS_MAKE_REG | LANDLOCK_ACCESS_FS_MAKE_SOCK |
    LANDLOCK_ACCESS_FS_MAKE_FIFO | LANDLOCK_ACCESS_FS_MAKE_BLOCK |
    LANDLOCK_ACCESS_FS_MAKE_SYM | LANDLOCK_ACCESS_FS_REFER |
    LANDLOCK_ACCESS_FS_TRUNCATE;
  const int fd = static_cast<int>(syscall(SYS_landlock_create_ruleset,
                                          &rules, sizeof(rules), 0));
  require(fd >= 0, "Cannot create read-only Landlock ruleset");
  const bool ok = prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) == 0 &&
                  syscall(SYS_landlock_restrict_self, fd, 0) == 0;
  close(fd);
  require(ok, "Cannot enforce read-only Landlock ruleset");
  const int probe = open("write_probe_must_not_exist", O_WRONLY | O_CREAT | O_EXCL, 0600);
  const int probe_errno = errno;
  if (probe >= 0) close(probe);
  require(probe < 0 && probe_errno == EACCES, "Filesystem write denial probe failed");
  return abi;
}

// Capture errors that GlobalMapping::load can otherwise catch and recover from.
class ValidationLog final : public spdlog::sinks::base_sink<std::mutex> {
public:
  std::size_t warnings = 0, errors = 0;
  bool recovery_or_corruption = false;
private:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    const std::string text(msg.payload.data(), msg.payload.size());
    warnings += msg.level == spdlog::level::warn;
    errors += msg.level >= spdlog::level::err;
    if (msg.level >= spdlog::level::warn &&
        (text.find("recover") != std::string::npos ||
         text.find("corrupt") != std::string::npos ||
         text.find("invalid") != std::string::npos)) recovery_or_corruption = true;
    const auto level = spdlog::level::to_string_view(msg.level);
    std::clog << '[' << std::string(level.data(), level.size()) << "] " << text << '\n';
  }
  void flush_() override { std::clog.flush(); }
};

int main(int argc, char** argv) {
  try {
    require(argc == 5, "usage: glim_dump_reload DUMP EXPECTED_SUBMAPS EXPECTED_FRAMES EXPECTED_POINTS");
    const auto dump = std::filesystem::canonical(argv[1]);
    const auto expected_submaps = std::stoull(argv[2]);
    const auto expected_frames = std::stoull(argv[3]);
    const auto expected_points = std::stoull(argv[4]);
    const rlimit no_core{0, 0};
    require(setrlimit(RLIMIT_CORE, &no_core) == 0, "Cannot disable core dumps");
    const long landlock_abi = deny_filesystem_writes();

    auto sink = std::make_shared<ValidationLog>();
    auto logger = std::make_shared<spdlog::logger>("glim", sink);
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    // Native create_module_logger("global") returns this registered console sink.
    auto global_logger = std::make_shared<spdlog::logger>("global", sink);
    global_logger->set_level(spdlog::level::info);
    spdlog::register_logger(global_logger);

    glim::GlobalConfig::instance((dump / "config").string());
    glim::GlobalMappingParams params;
    require(!params.enable_gpu, "Dump selects GPU factors; CPU-only validation required");
    // Native dummy implementation stores deserialized factors/values without optimizing.
    params.enable_optimization = false;
    std::size_t submaps = 0, frames = 0, points = 0, factors = 0, values = 0;
    bool archives_observed = false, submap_update_observed = false;
    glim::GlobalMappingCallbacks::on_insert_submap.add([&](const glim::SubMap::ConstPtr& submap) {
      require(submap && submap->frame && submap->frame->has_points(), "Native submap/points missing");
      require(submap->frame->has_covs() && submap->frame->has_normals() &&
              submap->frame->has_intensities(), "Compact attributes missing");
      require(submap->frames.size() == submap->odom_frames.size(), "Frame counts disagree");
      double previous = -1.0;
      for (const auto& frame : submap->frames) {
        require(frame && std::isfinite(frame->stamp) && frame->stamp > previous &&
                frame->T_world_imu.matrix().allFinite() &&
                frame->T_world_lidar.matrix().allFinite(), "Loaded frame data invalid");
        previous = frame->stamp;
      }
      for (std::size_t i = 0; i < submap->frame->size(); ++i) {
        require(submap->frame->points[i].allFinite() &&
                submap->frame->covs[i].allFinite() &&
                submap->frame->normals[i].allFinite() &&
                std::isfinite(submap->frame->intensities[i]), "Loaded compact point data invalid");
      }
      ++submaps;
      frames += submap->frames.size();
      points += submap->frame->size();
    });
    glim::GlobalMappingCallbacks::on_smoother_update.add(
      [&](gtsam_points::ISAM2Ext& solver, gtsam::NonlinearFactorGraph& graph, gtsam::Values& loaded) {
        require(dynamic_cast<gtsam_points::ISAM2ExtDummy*>(&solver), "Unexpected optimizing backend");
        factors = graph.nrFactors();
        values = loaded.size();
        require(factors > 0 && values > 0, "Deserialized graph/values empty");
        for (const auto& factor : graph) {
          require(static_cast<bool>(factor), "Null deserialized factor");
          for (auto key : factor->keys()) require(loaded.exists(key), "Factor references missing value");
        }
        archives_observed = true;
      });
    glim::GlobalMappingCallbacks::on_update_submaps.add(
      [&](const std::vector<glim::SubMap::Ptr>& loaded) {
        require(loaded.size() == expected_submaps, "Post-load submap count mismatch");
        for (const auto& submap : loaded)
          require(submap->T_world_origin.matrix().allFinite(), "Loaded submap pose invalid");
        submap_update_observed = true;
      });

    glim::GlobalMapping mapping(params);  // Synchronous API; no ROS/viewer/async worker.
    require(mapping.load(dump.string()), "Native GlobalMapping::load returned false");
    require(sink->errors == 0 && !sink->recovery_or_corruption, "Native deserialization error/recovery detected");
    require(archives_observed && submap_update_observed, "Native archive/update callbacks missing");
    require(submaps == expected_submaps && frames == expected_frames && points == expected_points,
            "Native loaded counts differ from expected experiment");
    const auto merged = mapping.export_points();  // In-memory export only; no file output.
    require(merged && merged->size() == expected_points, "Native exported point count mismatch");
    require(sink->errors == 0 && !sink->recovery_or_corruption, "Native export reported error");
    std::cout << "{\"result\":\"PASS\",\"landlock_abi\":" << landlock_abi
      << ",\"optimization_enabled\":false,\"submaps\":" << submaps
      << ",\"frames\":" << frames << ",\"points\":" << points
      << ",\"graph_factors\":" << factors << ",\"values\":" << values
      << ",\"exported_points\":" << merged->size()
      << ",\"warnings\":" << sink->warnings << ",\"errors\":" << sink->errors
      << ",\"recovery_detected\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
