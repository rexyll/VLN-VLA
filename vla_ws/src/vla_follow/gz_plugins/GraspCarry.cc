// GraspCarry.cc —— 世界内"位姿执行器"系统插件 (P86)
//
// 这个插件只做一件事: **把某个模型的世界位姿钉到上位机给的值上, 每 1ms 一次。**
// 它不读任何传感器、不算任何运动学。
//
// 为什么需要它 (走过的三条弯路)
// ============================
// 机器人抓壶/捧杯时, 物体必须在手中**刚性跟随**。本项目试过:
//
//  ① `gz-sim-detachable-joint-system`(官方做法): `child_model` 只在本插件
//     所在模型内部找嵌套模型, 世界级的 kettle/cup 找不到就报
//     `[Wrn] Child Model kettle could not be found.`; 更糟的是**即使从不
//     attach**, 这 4 个插件也会破坏机械臂的关节约束求解(P85 实测 R_pan 硬锁
//     0.175、速度反馈恒 0、14 个关节里 6 个不到位)。见 TaskPlan §6.4 #19。
//
//  ② 外部进程调 `/world/<w>/set_pose`: 位置能到 0.7mm, 但每次 teleport
//     不清零速度, 重力每帧往下拽(50ms 掉 6mm, 208ms 落回桌面); 而且
//     gz-transport 的 Python 请求/应答链路**会累计退化**, 约 400 次后成片
//     超时(实测第 83~99 拍连续全失败)并伴随 core dump。扛不住每帧调用。
//
//  ③ 本插件跑在**世界内部**, 1ms 一个 PreUpdate 直接写
//     `Model::SetWorldPoseCmd` —— 不经过请求/应答, 没有延迟、不会退化。
//
// 为什么位姿由上位机算, 而不是插件自己读掌心
// =========================================
// 第一版让插件用 `Link::WorldPose(ecm)` / `gz::sim::worldPose()` 自己读
// 掌心, 再乘一个 attach 时冻结的偏移。实测误差**只取决于手臂构型**:
// 手臂摆 0.45 rad 后, 外部量到 28.9mm / 0.76° 的固定偏差, 摆回原位变成
// 66.6mm / 1.63° —— 每摆一个来回累加一次。三方对照(话题掌心 / 关节角正解 /
// 插件读数)证实: **正解与话题完全一致(0.0mm), 而插件从 ECM 读到的 link
// 位姿是旧的**(x 差 7.3mm、姿态差 1.63°, 被 2.37m 力臂放大成 66.6mm)。
// gz 的 link `Pose` 组件对关节链的刷新不及时, 这个不可靠性不该由插件承担。
//
// 于是职责倒置: 掌心位姿由 `drink_task` 用 `arm_kin.fk`(P85 已验收
// 0.0mm/0.00°)从**实测关节角**算好发过来。插件只是执行器, 运动学仍然只有
// 一份真源(arm_kin.py)。
//
// 控制接口
// ========
// gz 话题 `/drink/carry`, 类型 `gz.msgs.StringMsg`, 由 ros_gz_bridge 从 ROS
// 侧单向桥过来, 与 37 路关节指令同一条链路(P85 已验证稳定)。指令:
//
//   grab <model> <x> <y> <z> <qx> <qy> <qz> <qw>   登记目标并回 ack
//   set  <model> <x> <y> <z> <qx> <qy> <qz> <qw>   更新目标(不回 ack, 高频)
//   place <model> <x> <y> <z> <qx> <qy> <qz> <qw>  一次性摆位(不持续钉住)
//   joint <model> <joint> <value>                  运动学驱动某个关节并回 ack
//   release <model>                                取消跟随并回 ack
//   release all                                    全部取消
//
// `joint` 是给壶/杯上的两个滑动副(jet / water)用的。它们是**速度模式**的
// JointPositionController, 一旦所属模型被本插件每 1ms teleport, 那些速度
// 指令就会被 teleport 的清零抢走 —— 实测同一份命令时而完全不动、时而只走
// 一部分(jet 0.124/0.16、water 0/0.012), 全是操作顺序的竞态。
// 改成每 tick 写 `JointPositionReset`: 关节位置直接置位, 与 teleport 同源
// (都是"运动学驱动"), 不存在抢速度的问题。
//
// 注意: 话题上的四元数顺序是 (qx qy qz qw), 与 ROS 一致; 转成
// `gz::math::Pose3d` 时必须重排成 (qw qx qy qz), 见 OnCmd 里的踩坑记录。
//
// `place` 是给测试/布景用的: 需要机器人先站到服务位才好做抓取验收, 但
// 正式流程里是 Nav2 开过去的。摆一次就撤, 不会把底盘钉死。
//
// 目标一旦登记, 每 tick 都会重发一次 —— 即使中间丢了几条消息, 物体也不会
// 掉队, 单向下发天然免疫丢包。
//
// 配合的 SDF 约定
// ==============
// 壶/杯的 body link 必须带 `<gravity>false</gravity>`: 本插件只负责位置,
// 不负责速度。若重力照常作用, 两次钉住之间物体会被加速, 松手瞬间的残余
// 速度会让它飞出去。关掉重力后它既不会被拽走, 也不会积累速度, 松手就停在
// 原地 —— 正是"放回桌面原位置"需要的语义。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace vla
{

class GraspCarry
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  /// \brief 一个被驱动的模型
  struct Target
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::math::Pose3d pose;          ///< 目标世界位姿
    bool active{false};
  };

public:
  void Configure(const gz::sim::Entity & /*_entity*/,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager & /*_ecm*/,
                 gz::sim::EventManager & /*_eventMgr*/) override
  {
    std::string topic = "/drink/carry";
    if (_sdf && _sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");
    if (_sdf && _sdf->HasElement("debug"))
      this->debug = _sdf->Get<bool>("debug");

    std::string ackTopic = topic + "/ack";
    if (_sdf && _sdf->HasElement("ack_topic"))
      ackTopic = _sdf->Get<std::string>("ack_topic");
    this->ackPub = this->node.Advertise<gz::msgs::StringMsg>(ackTopic);

    if (!this->node.Subscribe(topic, &GraspCarry::OnCmd, this))
    {
      gzerr << "[GraspCarry] 订阅 " << topic << " 失败" << std::endl;
      return;
    }
    gzmsg << "[GraspCarry] 就绪, 监听 " << topic << "  ack->" << ackTopic
          << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    // 1) 消化待处理指令(实体查找要 ecm, 只能在这里做)
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      if (this->hasPending)
      {
        this->Apply(this->pending, _ecm);
        this->hasPending = false;
      }
    }

    // 2) 每 tick 重发一次目标位姿
    for (auto &kv : this->targets)
    {
      auto &t = kv.second;
      if (!t.active || t.entity == gz::sim::kNullEntity)
        continue;
      gz::sim::Model(t.entity).SetWorldPoseCmd(_ecm, t.pose);
    }

    // 3) 每 tick 重发一次关节位置(运动学驱动, 与 teleport 同源)
    for (auto &kv : this->joints)
    {
      auto &jt = kv.second;
      if (jt.entity == gz::sim::kNullEntity)
        continue;
      _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
          jt.entity, std::vector<double>{jt.value});
    }

    if (this->debug && ++this->tick % 500 == 0)
    {
      for (auto &kv : this->targets)
      {
        if (kv.second.active)
          gzmsg << "[GraspCarry/dbg] " << kv.first << " -> ("
                << kv.second.pose.Pos().X() << " "
                << kv.second.pose.Pos().Y() << " "
                << kv.second.pose.Pos().Z() << ")" << std::endl;
      }
    }
  }

private:
  struct Cmd
  {
    std::string op;          // grab / set / place / joint / release
    std::string model;
    std::string joint;       // 仅 joint 指令用
    double value{0.0};       // 仅 joint 指令用
    gz::math::Pose3d pose;
  };

  struct JointTarget
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    double value{0.0};
  };

  void SendAck(const std::string &_text)
  {
    if (!this->ackPub.Valid())
      return;
    gz::msgs::StringMsg m;
    m.set_data(_text);
    this->ackPub.Publish(m);
  }

  /// \brief transport 线程: 只解析字符串
  void OnCmd(const gz::msgs::StringMsg &_msg)
  {
    std::istringstream iss(_msg.data());
    Cmd c;
    iss >> c.op >> c.model;
    if (c.op == "release")
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      this->pending = c;
      this->hasPending = true;
      return;
    }
    if (c.op == "joint")
    {
      if (!(iss >> c.joint >> c.value))
      {
        gzerr << "[GraspCarry] joint 参数不足: '" << _msg.data() << "'"
              << std::endl;
        return;
      }
      std::lock_guard<std::mutex> lk(this->mtx);
      this->pending = c;
      this->hasPending = true;
      return;
    }
    if (c.op != "grab" && c.op != "set" && c.op != "place")
    {
      gzerr << "[GraspCarry] 未知指令 '" << _msg.data() << "'" << std::endl;
      return;
    }
    double x, y, z, qx, qy, qz, qw;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw))
    {
      gzerr << "[GraspCarry] 位姿参数不足: '" << _msg.data() << "'"
            << std::endl;
      return;
    }
    // 踩坑记录: gz::math::Pose3d 的 7 参数构造是
    //   Pose3d(x, y, z, qw, qx, qy, qz)      <-- 四元数 **w 在最前**
    // 不是直觉上的 (qx, qy, qz, qw)。实测确认:
    //   Pose3d(1,2,3, 0.1,0.2,0.3,0.9274).rot() = (x,y,z,w)=(0.2,0.3,0.9274,0.1)
    // 写反的后果极隐蔽 —— 位置精确到 1e-15 而姿态整体错位一位:
    // 传 identity "0 0 0 1" 会被读成 qw=0,qz=1, 机器人恒转 180°;
    // 抓取跟随里掌心旋转全错, 表现为"位置零偏差但相对姿态差 120°"。
    // 这里显式构造, 避开参数顺序歧义。
    c.pose = gz::math::Pose3d(gz::math::Vector3d(x, y, z),
                              gz::math::Quaterniond(qw, qx, qy, qz));
    std::lock_guard<std::mutex> lk(this->mtx);
    this->pending = c;
    this->hasPending = true;
  }

  /// \brief 仿真线程: 执行一条指令
  void Apply(const Cmd &_c, gz::sim::EntityComponentManager &_ecm)
  {
    if (_c.op == "release")
    {
      if (_c.model == "all" || _c.model.empty())
      {
        for (auto &kv : this->targets)
          kv.second.active = false;
        gzmsg << "[GraspCarry] release all" << std::endl;
        this->SendAck("ok release all");
        return;
      }
      auto it = this->targets.find(_c.model);
      if (it != this->targets.end())
        it->second.active = false;
      this->SendAck("ok release " + _c.model);
      return;
    }

    if (_c.op == "joint")
    {
      // 关节实体是模型的子实体, 按模型 + 关节名精确定位(关节名全世界唯一,
      // 但走父子关系更稳, 也便于以后同名多实例的扩展)
      const auto me = _ecm.EntityByComponents(
          gz::sim::components::Model(),
          gz::sim::components::Name(_c.model));
      if (me == gz::sim::kNullEntity)
      {
        gzerr << "[GraspCarry] joint: 找不到模型 '" << _c.model << "'"
              << std::endl;
        this->SendAck("fail " + _c.joint + " no-model");
        return;
      }
      gz::sim::Entity je = gz::sim::kNullEntity;
      for (const auto e : _ecm.ChildrenByComponents(
               me, gz::sim::components::Joint()))
      {
        const auto nm = _ecm.Component<gz::sim::components::Name>(e);
        if (nm && nm->Data() == _c.joint)
        {
          je = e;
          break;
        }
      }
      if (je == gz::sim::kNullEntity)
      {
        gzerr << "[GraspCarry] joint: 模型 '" << _c.model << "' 下没有关节 '"
              << _c.joint << "'" << std::endl;
        this->SendAck("fail " + _c.joint + " no-joint");
        return;
      }
      if (!_ecm.Component<gz::sim::components::JointPositionReset>(je))
        _ecm.CreateComponent(
            je, gz::sim::components::JointPositionReset({_c.value}));
      JointTarget jt;
      jt.entity = je;
      jt.value = _c.value;
      // 用 model::joint 做键, 免得不同模型下的同名关节互相覆盖
      this->joints[_c.model + "::" + _c.joint] = jt;
      this->SendAck("ok joint " + _c.joint);
      return;
    }

    auto it = this->targets.find(_c.model);
    if (it == this->targets.end())
    {
      const auto e = _ecm.EntityByComponents(
          gz::sim::components::Model(), gz::sim::components::Name(_c.model));
      if (e == gz::sim::kNullEntity)
      {
        gzerr << "[GraspCarry] 找不到模型 '" << _c.model << "'" << std::endl;
        if (_c.op == "grab")
          this->SendAck("fail " + _c.model + " no-model");
        return;
      }
      Target t;
      t.entity = e;
      it = this->targets.emplace(_c.model, t).first;
    }
    it->second.pose = _c.pose;
    it->second.active = (_c.op != "place");   // place 只钉一帧

    if (_c.op == "place")
    {
      gz::sim::Model(it->second.entity).SetWorldPoseCmd(_ecm, _c.pose);
      this->SendAck("ok place " + _c.model);
      return;
    }
    if (_c.op == "grab")
    {
      gzmsg << "[GraspCarry] grab " << _c.model << " @ ("
            << _c.pose.Pos().X() << " " << _c.pose.Pos().Y() << " "
            << _c.pose.Pos().Z() << ")" << std::endl;
      this->SendAck("ok grab " + _c.model);
    }
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher ackPub;
  std::mutex mtx;
  Cmd pending;
  bool hasPending{false};
  std::map<std::string, Target> targets;
  // 踩坑记录(P87): jet/water 是**速度模式** JointPositionController, 所属模型
  // 被本插件每 1ms teleport 时, 速度指令会被 teleport 的清零抢走 —— 同一命令
  // 时而全动、时而全不动、时而只走一部分(jet 0.124/0.16, water 0/0.012)。
  // 所以这两个关节改由本插件用 JointPositionReset 直接写位置, 与 teleport 同源。
  std::map<std::string, JointTarget> joints;   // key = model::joint
  bool debug{false};
  uint64_t tick{0};
};

}  // namespace vla

GZ_ADD_PLUGIN(vla::GraspCarry, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vla::GraspCarry, "vla::GraspCarry")
