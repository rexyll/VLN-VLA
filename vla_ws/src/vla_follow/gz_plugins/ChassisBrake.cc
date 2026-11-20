// ChassisBrake.cc — opt-in, testable wheel-joint position brake (P104-10.1)
//
// This plugin is intentionally NOT loaded by work_area.sdf yet. It is first
// exercised in an isolated Gazebo world. Enabling it installs JointForceCmd
// on both drive-wheel joints; Gazebo Sim 8 Physics gives JointForceCmd
// precedence over DiffDrive's per-tick JointVelocityCmd. Disabling removes
// JointForceCmd so DiffDrive velocity control can resume.
//
// LOCKED status means the force-command components were installed. It is not
// proof that the chassis is physically stationary; pose / wheel telemetry
// remains an independent acceptance gate.

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

namespace vla
{
class ChassisBrake
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[ChassisBrake] Must be attached to a model." << std::endl;
      return;
    }

    if (_sdf)
    {
      this->leftJointName = _sdf->Get<std::string>(
          "left_joint", "left_wheel_joint").first;
      this->rightJointName = _sdf->Get<std::string>(
          "right_joint", "right_wheel_joint").first;
      this->commandTopic = _sdf->Get<std::string>(
          "command_topic", "/chassis_brake/enable").first;
      this->statusTopic = _sdf->Get<std::string>(
          "status_topic", "/chassis_brake/status").first;
      this->kp = _sdf->Get<double>("position_kp", 80.0).first;
      this->kd = _sdf->Get<double>("velocity_kd", 2.0).first;
      this->maxTorque = _sdf->Get<double>("max_torque", 5.0).first;
    }

    if (this->leftJointName.empty() || this->rightJointName.empty() ||
        this->leftJointName == this->rightJointName ||
        this->kp < 0.0 || this->kd < 0.0 || this->maxTorque <= 0.0)
    {
      gzerr << "[ChassisBrake] Invalid joint names or gains/torque limit."
            << std::endl;
      return;
    }

    this->statusPub =
        this->node.Advertise<gz::msgs::StringMsg>(this->statusTopic);
    if (!this->node.Subscribe(this->commandTopic,
                              &ChassisBrake::OnCommand, this))
    {
      gzerr << "[ChassisBrake] Failed to subscribe " << this->commandTopic
            << std::endl;
      return;
    }

    this->configured = true;
    gzmsg << "[ChassisBrake] Ready; command=" << this->commandTopic
          << " status=" << this->statusTopic
          << " joints=" << this->leftJointName << ","
          << this->rightJointName << " max_torque=" << this->maxTorque
          << "Nm" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    if (this->leftJoint == gz::sim::kNullEntity)
      this->leftJoint = this->model.JointByName(_ecm, this->leftJointName);
    if (this->rightJoint == gz::sim::kNullEntity)
      this->rightJoint = this->model.JointByName(_ecm, this->rightJointName);
    if (this->leftJoint == gz::sim::kNullEntity ||
        this->rightJoint == gz::sim::kNullEntity)
      return;

    bool hasRequest = false;
    bool requested = false;
    bool newlyEnabled = false;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      hasRequest = this->hasPendingRequest;
      requested = this->requestedEnabled;
      this->hasPendingRequest = false;
    }

    if (hasRequest && !requested && this->enabled)
    {
      this->RemoveBrake(_ecm);
      this->enabled = false;
      this->PublishStatus("UNLOCKED force_components_removed");
    }

    if (hasRequest && requested && !this->enabled)
    {
      double leftPosition = 0.0;
      double rightPosition = 0.0;
      if (!this->ReadJointState(_ecm, this->leftJoint, leftPosition) ||
          !this->ReadJointState(_ecm, this->rightJoint, rightPosition))
      {
        // Keep the request pending; physics may not have populated the
        // position / velocity components on the first simulation iteration.
        std::lock_guard<std::mutex> guard(this->mutex);
        this->hasPendingRequest = true;
        this->requestedEnabled = true;
        return;
      }
      this->leftTarget = leftPosition;
      this->rightTarget = rightPosition;
      this->enabled = true;
      newlyEnabled = true;
    }

    if (!this->enabled)
      return;

    double leftPosition = 0.0;
    double rightPosition = 0.0;
    double leftVelocity = 0.0;
    double rightVelocity = 0.0;
    if (!this->ReadJointState(_ecm, this->leftJoint, leftPosition,
                              &leftVelocity) ||
        !this->ReadJointState(_ecm, this->rightJoint, rightPosition,
                              &rightVelocity))
    {
      this->RemoveBrake(_ecm);
      this->enabled = false;
      this->PublishStatus("FAULT joint_state_unavailable force_removed");
      return;
    }

    const double leftError = std::remainder(
        this->leftTarget - leftPosition, 6.28318530717958647692);
    const double rightError = std::remainder(
        this->rightTarget - rightPosition, 2.0 * M_PI);
    const double leftTorque = std::clamp(
        this->kp * leftError - this->kd * leftVelocity,
        -this->maxTorque, this->maxTorque);
    const double rightTorque = std::clamp(
        this->kp * rightError - this->kd * rightVelocity,
        -this->maxTorque, this->maxTorque);

    this->SetForce(_ecm, this->leftJoint, leftTorque);
    this->SetForce(_ecm, this->rightJoint, rightTorque);
    if (newlyEnabled)
      this->PublishStatus("LOCKED force_components_installed");
  }

private:
  void OnCommand(const gz::msgs::Boolean &_msg)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->requestedEnabled = _msg.data();
    this->hasPendingRequest = true;
  }

  bool ReadJointState(gz::sim::EntityComponentManager &_ecm,
                      gz::sim::Entity _joint, double &_position,
                      double *_velocity = nullptr)
  {
    const auto *position =
        _ecm.Component<gz::sim::components::JointPosition>(_joint);
    const auto *velocity =
        _ecm.Component<gz::sim::components::JointVelocity>(_joint);
    if (!position || position->Data().empty() ||
        (_velocity && (!velocity || velocity->Data().empty())))
      return false;
    _position = position->Data()[0];
    if (_velocity)
      *_velocity = velocity->Data()[0];
    return std::isfinite(_position) &&
        (!_velocity || std::isfinite(*_velocity));
  }

  void SetForce(gz::sim::EntityComponentManager &_ecm,
                gz::sim::Entity _joint, double _torque)
  {
    auto *force =
        _ecm.Component<gz::sim::components::JointForceCmd>(_joint);
    if (!force)
    {
      _ecm.CreateComponent(
          _joint, gz::sim::components::JointForceCmd({_torque}));
    }
    else
    {
      force->Data()[0] = _torque;
    }
  }

  void RemoveBrake(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->leftJoint != gz::sim::kNullEntity)
      _ecm.RemoveComponent<gz::sim::components::JointForceCmd>(
          this->leftJoint);
    if (this->rightJoint != gz::sim::kNullEntity)
      _ecm.RemoveComponent<gz::sim::components::JointForceCmd>(
          this->rightJoint);
  }

  void PublishStatus(const std::string &_status)
  {
    if (!this->statusPub.Valid())
      return;
    gz::msgs::StringMsg message;
    message.set_data(_status);
    this->statusPub.Publish(message);
    gzmsg << "[ChassisBrake] " << _status << std::endl;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity leftJoint{gz::sim::kNullEntity};
  gz::sim::Entity rightJoint{gz::sim::kNullEntity};
  std::string leftJointName{"left_wheel_joint"};
  std::string rightJointName{"right_wheel_joint"};
  std::string commandTopic{"/chassis_brake/enable"};
  std::string statusTopic{"/chassis_brake/status"};
  gz::transport::Node node;
  gz::transport::Node::Publisher statusPub;
  std::mutex mutex;
  bool configured{false};
  bool hasPendingRequest{false};
  bool requestedEnabled{false};
  bool enabled{false};
  double leftTarget{0.0};
  double rightTarget{0.0};
  double kp{80.0};
  double kd{2.0};
  double maxTorque{5.0};
};
}  // namespace vla

GZ_ADD_PLUGIN(vla::ChassisBrake, gz::sim::System,
              vla::ChassisBrake::ISystemConfigure,
              vla::ChassisBrake::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vla::ChassisBrake, "vla::ChassisBrake")
