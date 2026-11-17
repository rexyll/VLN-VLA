#include <cstdint>
#include <atomic>
#include <chrono>
#include <functional>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "mca_drink_control/mca_arm_core.hpp"
#include "mca_drink_control/srv/solve_ik.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mca_drink_control {

class McaIkServer final : public rclcpp::Node {
 public:
  McaIkServer() : Node("mca_ik_server") {
    const auto share = ament_index_cpp::get_package_share_directory("mca_drink_control");
    load_model("R", share + "/models/drink_arm_R.urdf");
    load_model("L", share + "/models/drink_arm_L.urdf");
    service_ = create_service<srv::SolveIk>(
        "/mca_drink/solve_ik",
        std::bind(&McaIkServer::solve, this, std::placeholders::_1,
                  std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "MCA DLS IK service ready; models are generated from vla_follow.arm_kin");
  }

 private:
  void load_model(const std::string& side, const std::string& path) {
    std::ifstream stream(path);
    if (!stream.good()) throw std::runtime_error("cannot open arm model: " + path);
    const std::string urdf((std::istreambuf_iterator<char>(stream)),
                           std::istreambuf_iterator<char>());
    mca::kinematics::RobotModel model;
    std::string error;
    if (!mca::kinematics::UrdfModelAdapter::parse(urdf, model, &error))
      throw std::runtime_error("invalid " + side + " arm URDF: " + error);
    models_.emplace(side, std::move(model));
  }

  static mca::geometry::Pose3D to_mca_pose(
      const geometry_msgs::msg::Pose& pose) {
    mca::geometry::Pose3D out;
    out.position = {pose.position.x, pose.position.y, pose.position.z};
    out.rotation = {pose.orientation.w, pose.orientation.x,
                    pose.orientation.y, pose.orientation.z};
    return out;
  }

  void solve(const std::shared_ptr<srv::SolveIk::Request> request,
             std::shared_ptr<srv::SolveIk::Response> response) {
    const auto request_id = ++request_counter_;
    const auto started = std::chrono::steady_clock::now();
    const auto& target = request->target.position;
    RCLCPP_INFO(get_logger(),
                "MCA_IK_REQUEST id=%llu side=%s target_base_m=[%.6f,%.6f,%.6f] seed_count=%zu pos_tol_m=%.6f rot_tol_rad=%.6f max_iter=%u",
                static_cast<unsigned long long>(request_id),
                request->side.c_str(), target.x, target.y, target.z,
                request->seed.position.size(), request->position_tolerance_m,
                request->orientation_tolerance_rad, request->max_iterations);
    const auto model_it = models_.find(request->side);
    if (model_it == models_.end()) {
      response->message = "side must be R or L";
      RCLCPP_WARN(get_logger(), "MCA_IK_REJECT id=%llu reason=%s",
                  static_cast<unsigned long long>(request_id),
                  response->message.c_str());
      return;
    }
    if (request->seed.name.size() != 7U ||
        request->seed.position.size() != 7U) {
      response->message = "seed must contain seven named arm joint positions";
      RCLCPP_WARN(get_logger(), "MCA_IK_REJECT id=%llu reason=%s names=%zu positions=%zu",
                  static_cast<unsigned long long>(request_id),
                  response->message.c_str(), request->seed.name.size(),
                  request->seed.position.size());
      return;
    }
    std::vector<double> seed;
    seed.reserve(7U);
    std::size_t seed_index = 0;
    for (const auto& joint : model_it->second.joints) {
      if (joint.type == mca::kinematics::JointType::Fixed) continue;
      if (request->seed.name[seed_index] != joint.name) {
        response->message = "seed joint name/order mismatch at index " +
                            std::to_string(seed_index) + ": expected " +
                            joint.name + ", got " +
                            request->seed.name[seed_index];
        RCLCPP_WARN(get_logger(), "MCA_IK_REJECT id=%llu reason=%s",
                    static_cast<unsigned long long>(request_id),
                    response->message.c_str());
        return;
      }
      seed.push_back(request->seed.position[seed_index]);
      ++seed_index;
    }
    auto options = mca::kinematics::DampedLeastSquaresIkOptions{};
    if (request->position_tolerance_m > 0.0)
      options.position_tolerance = request->position_tolerance_m;
    if (request->orientation_tolerance_rad > 0.0)
      options.orientation_tolerance = request->orientation_tolerance_rad;
    if (request->max_iterations > 0U)
      options.max_iterations = request->max_iterations;

    const auto report = McaArmCore::solve_ik(
        model_it->second, to_mca_pose(request->target), seed, options);
    response->success = report.converged;
    seed_index = 0;
    for (const auto& joint : model_it->second.joints) {
      if (joint.type == mca::kinematics::JointType::Fixed) continue;
      response->solution.name.push_back(joint.name);
      if (seed_index < report.joint_positions.size())
        response->solution.position.push_back(report.joint_positions[seed_index]);
      ++seed_index;
    }
    response->solution.header.stamp = now();
    response->iterations = static_cast<uint32_t>(report.iterations);
    response->position_error_m = report.position_error_m;
    response->orientation_error_rad = report.orientation_error_rad;
    response->message = report.message;
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    RCLCPP_INFO(get_logger(),
                "MCA_IK_RESULT id=%llu side=%s success=%s iterations=%u position_error_m=%.8f orientation_error_rad=%.8f elapsed_ms=%.3f message=%s",
                static_cast<unsigned long long>(request_id),
                request->side.c_str(), response->success ? "true" : "false",
                response->iterations, response->position_error_m,
                response->orientation_error_rad, elapsed_ms,
                response->message.c_str());
  }

  std::atomic<std::uint64_t> request_counter_{0};
  std::unordered_map<std::string, mca::kinematics::RobotModel> models_;
  rclcpp::Service<srv::SolveIk>::SharedPtr service_;
};

}  // namespace mca_drink_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mca_drink_control::McaIkServer>());
  } catch (const std::exception& error) {
    std::cerr << "mca_ik_server fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
