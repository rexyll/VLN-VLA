// DynamicChassisLock.cc — experimental runtime world-to-chassis fixed joint.
// Diagnostic-only candidate for Gazebo Sim 8.15. This plugin creates/removes
// a fixed joint through SdfEntityCreator; it never changes production SDF or
// replaces the production lock.
//
// A published "attached" state means only that the joint entity was created.
// Physical hold is independently accepted by measured chassis pose under a
// nonzero DiffDrive challenge. A creation/removal fault never reports unlocked.

#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/transport/Node.hh>
#include <sdf/Joint.hh>

namespace vla
{
class DynamicChassisLock
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  enum class Request { NONE, ATTACH, DETACH };

public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override
  {
    this->modelEntity = _entity;
    this->attachTopic = "/world/follow_world/dynamic_chassis_lock/attach";
    this->detachTopic = "/world/follow_world/dynamic_chassis_lock/detach";
    this->stateTopic = "/world/follow_world/dynamic_chassis_lock/state";
    bool initiallyLocked = false;
    if (_sdf)
    {
      this->attachTopic = _sdf->Get<std::string>(
          "attach_topic", this->attachTopic).first;
      this->detachTopic = _sdf->Get<std::string>(
          "detach_topic", this->detachTopic).first;
      this->stateTopic = _sdf->Get<std::string>(
          "state_topic", this->stateTopic).first;
      initiallyLocked = _sdf->Get<bool>("initially_locked", false).first;
    }

    if (_entity == gz::sim::kNullEntity || !_ecm.HasEntity(_entity))
    {
      gzerr << "[DynamicChassisLock] invalid robot model entity" << std::endl;
      return;
    }

    this->creator = std::make_unique<gz::sim::SdfEntityCreator>(
        _ecm, _eventMgr);
    this->statePub =
        this->node.Advertise<gz::msgs::StringMsg>(this->stateTopic);
    if (!this->statePub.Valid() ||
        !this->node.Subscribe(this->attachTopic,
            &DynamicChassisLock::OnAttach, this) ||
        !this->node.Subscribe(this->detachTopic,
            &DynamicChassisLock::OnDetach, this))
    {
      gzerr << "[DynamicChassisLock] transport setup failed" << std::endl;
      this->creator.reset();
      return;
    }

    this->configured = true;
    if (initiallyLocked)
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      this->pending = Request::ATTACH;
    }
    else
    {
      this->Publish("detached");
    }
    gzmsg << "[DynamicChassisLock] ready; model=" << _entity
          << " attach=" << this->attachTopic
          << " detach=" << this->detachTopic << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    if (this->removalPending)
    {
      if (!this->creator || _ecm.HasEntity(this->jointEntity))
        return;
      this->jointEntity = gz::sim::kNullEntity;
      this->removalPending = false;
      this->Publish("detached");
    }

    Request request = Request::NONE;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      request = this->pending;
      this->pending = Request::NONE;
    }
    if (request == Request::NONE)
      return;

    if (request == Request::ATTACH)
    {
      if (this->removalPending)
      {
        this->Publish("fault attach_while_removal_pending");
        return;
      }
      if (this->jointEntity != gz::sim::kNullEntity &&
          _ecm.HasEntity(this->jointEntity))
      {
        this->Publish("attached");
        return;
      }

      sdf::Joint joint;
      joint.SetName("runtime_world_fixed_chassis_lock");
      joint.SetType(sdf::JointType::FIXED);
      // Runtime-created joint is outside the source SDF frame graph.
      // These names are deliberately passed as pre-resolved ECM names.
      joint.SetParentLinkName("world");
      joint.SetChildLinkName("chassis");

      const auto created = this->creator->CreateEntities(&joint, true);
      if (created == gz::sim::kNullEntity)
      {
        this->Publish("fault joint_entity_creation_failed");
        gzerr << "[DynamicChassisLock] failed to create fixed joint"
              << std::endl;
        return;
      }
      this->creator->SetParent(created, this->modelEntity);
      if (!_ecm.HasEntity(created) ||
          !_ecm.Component<gz::sim::components::Joint>(created))
      {
        this->creator->RequestRemoveEntity(created);
        this->Publish("fault joint_entity_incomplete");
        gzerr << "[DynamicChassisLock] created joint lacks Joint component"
              << std::endl;
        return;
      }

      // Gazebo Sim 8 Physics removes ordinary Joint entities from ECM without
      // removing the corresponding physics-engine constraint. Its supported
      // runtime detach path specifically handles removed DetachableJoint
      // components and calls DetachJointFeature. Mark this runtime joint so
      // removal follows that path instead of silently leaving a ghost lock.
      _ecm.CreateComponent(created,
          gz::sim::components::DetachableJoint());

      this->jointEntity = created;
      this->Publish("attached");
      gzmsg << "[DynamicChassisLock] fixed detachable joint entity created: "
            << created << " (physical hold still requires external test)"
            << std::endl;
      return;
    }

    if (request == Request::DETACH)
    {
      if (this->jointEntity == gz::sim::kNullEntity ||
          !_ecm.HasEntity(this->jointEntity))
      {
        this->jointEntity = gz::sim::kNullEntity;
        this->Publish("detached");
        return;
      }
      this->creator->RequestRemoveEntity(this->jointEntity);
      this->removalPending = true;
      gzmsg << "[DynamicChassisLock] detachable fixed-joint removal requested: "
            << this->jointEntity << std::endl;
    }
  }

private:
  void OnAttach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->pending = Request::ATTACH;
  }

  void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->pending = Request::DETACH;
  }

  void Publish(const std::string &_state)
  {
    if (!this->statePub.Valid())
      return;
    gz::msgs::StringMsg message;
    message.set_data(_state);
    this->statePub.Publish(message);
    gzmsg << "[DynamicChassisLock] state=" << _state << std::endl;
  }

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::unique_ptr<gz::sim::SdfEntityCreator> creator;
  gz::transport::Node node;
  gz::transport::Node::Publisher statePub;
  std::string attachTopic;
  std::string detachTopic;
  std::string stateTopic;
  std::mutex mutex;
  Request pending{Request::NONE};
  bool configured{false};
  bool removalPending{false};
};
}  // namespace vla

GZ_ADD_PLUGIN(vla::DynamicChassisLock, gz::sim::System,
              vla::DynamicChassisLock::ISystemConfigure,
              vla::DynamicChassisLock::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vla::DynamicChassisLock, "vla::DynamicChassisLock")
