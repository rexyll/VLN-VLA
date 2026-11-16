#include <fstream>
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
    const auto model_it = models_.find(request->side);
    if (model_it == models_.end()) {
      response->message = "side must be R or L";
      return;
    }
    if (request->seed.size() != 7U) {
      response->message = "seed must contain exactly seven arm joint positions";
      return;
    }
    auto options = mca::kinematics::DampedLeastSquaresIkOptions{};
    if (request->position_tolerance_m > 0.0)
      options.position_tolerance = request->position_tolerance_m;
    if (request->orientation_tolerance_rad > 0.0)
      options.orientation_tolerance = request->orientation_tolerance_rad;
    if (request->max_iterations > 0U)
      options.max_iterations = request->max_iterations;

    const auto report = McaArmCore::solve_ik(
        model_it->second, to_mca_pose(request->target), request->seed, options);
    response->success = report.converged;
    response->joint_positions = report.joint_positions;
    response->iterations = static_cast<uint32_t>(report.iterations);
    response->position_error_m = report.position_error_m;
    response->orientation_error_rad = report.orientation_error_rad;
    response->message = report.message;
  }

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
