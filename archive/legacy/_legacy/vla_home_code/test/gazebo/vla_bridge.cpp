// =====================================================================
// vla_bridge.cpp —— Gazebo Harmonic (gz-sim 8) 系统插件：VLA 仿真桥
//
// 功能：把仿真世界封装成 TCP 行协议（默认 127.0.0.1:9101），供 Python 侧
//       (gz_env.py) 以与真机/合成环境相同的接口读写。单位：米（1px = 1cm）。
//
//   PING                        -> PONG vla_bridge 1.0
//   STATE                       -> STATE <t> <ax> <ay> <bx> <by> <rx> <ry> <yx> <yy> <contact>
//   VEL <vx> <vy>               -> 持续速度指令(m/s)，插件内积分位移，直到 STOP/新 VEL -> OK
//   STOP                        -> 速度清零                                            -> OK
//   SET_POSE agent|box|red|yellow <x> <y>  -> 传送模型                                  -> OK
//   RESET <ax> <ay> <bx> <by> <rx> <ry> <yx> <yy> -> 一次复位全部模型                  -> OK
//   GET_IMAGE                   -> IMG <w> <h> <nbytes>\n + RGB 原始字节（顶视相机最新帧）
//   IMG_STATUS                  -> IMG_STATUS <frames> <topic>
//   BYE                         -> 断开
//
// 世界内声明：<plugin filename="vla_bridge" name="vla_bridge::VLABridge"><port>9101</port></plugin>
// =====================================================================
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <sdf/Element.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace vla_bridge {

static std::mutex g_mtx;                 // 保护以下全部共享状态
static double g_vel_x = 0.0, g_vel_y = 0.0;
static bool g_vel_active = false;
static double g_set_x[4] = {-1, -1, -1, -1};   // 待传送目标: agent,box,red,yellow
static double g_set_y[4] = {-1, -1, -1, -1};
static double g_pos[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};  // PostUpdate 读回
static double g_sim_t = 0.0;
static int g_contact = 0;                // 接触代理：agent 与 box 距离 < 7.2cm
static std::vector<unsigned char> g_img;
static int g_img_w = 0, g_img_h = 0;
static long g_img_frames = 0;
static long g_topic_hits[8] = {0};

class VLABridge : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate,
                  public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_eventMgr*/) override {
    int port = 9101;
    if (_sdf->HasElement("port")) port = _sdf->Get<int>("port");
    if (_sdf->HasElement("camera_topic"))
      cam_topic_ = _sdf->Get<std::string>("camera_topic");
    // 模型实体在 PreUpdate 惰性解析（Configure 时 ECM 未必已就绪）

    // 订阅顶视相机（尝试多个候选 topic，哪个来数据用哪个）
    node_ = std::make_unique<gz::transport::Node>();
    std::vector<std::string> candidates;
    if (!cam_topic_.empty()) {
      candidates.push_back(cam_topic_);
      candidates.push_back(cam_topic_ + "/image");
    }
    candidates.push_back("/world/vla_arena/model/overhead_cam/link/link/sensor/cam/image");
    candidates.push_back("/vla_cam/image");
    for (size_t i = 0; i < candidates.size() && i < 8; ++i) {
      const std::string &t = candidates[i];
      node_->Subscribe<gz::msgs::Image>(
          t, [this, i](const gz::msgs::Image &_msg) { this->OnImage(i, _msg); });
      topics_.push_back(t);
    }

    tcp_thread_ = std::thread(&VLABridge::ServerLoop, this, port);
    gzmsg << "[vla_bridge] TCP listening on port " << port << "\n";
  }

  void OnImage(size_t _idx, const gz::msgs::Image &_msg) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_topic_hits[_idx]++;
    g_img_w = _msg.width();
    g_img_h = _msg.height();
    const std::string &d = _msg.data();
    g_img.assign(d.begin(), d.end());
    ++g_img_frames;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    if (_info.paused || !Resolve(_ecm)) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    std::lock_guard<std::mutex> lk(g_mtx);
    // 速度积分（agent 为静态模型：位移由插件按 v*dt 推进，与合成环境语义一致）
    if (g_vel_active && entities_[0] != gz::sim::kNullEntity) {
      if (auto comp = _ecm.Component<gz::sim::components::Pose>(entities_[0])) {
        auto p = comp->Data();
        p.Pos().X(std::clamp(p.Pos().X() + g_vel_x * dt, 0.02, 0.61));
        p.Pos().Y(std::clamp(p.Pos().Y() + g_vel_y * dt, 0.02, 0.61));
        comp->Data() = p;
      }
      // push 语义：接触（中心距<7.2cm）时箱子随智能体位移 —— 与 env_synth.env_step 一致。
      // （gz-sim 静态模型碰撞体不随 Pose 传送，物理推动不可靠，改用训练域同款运动学规则）
      if (auto ac = _ecm.Component<gz::sim::components::Pose>(entities_[0])) {
        if (auto bc = _ecm.Component<gz::sim::components::Pose>(entities_[1])) {
          auto ap = ac->Data().Pos();
          auto bp = bc->Data().Pos();
          const double dx = ap.X() - bp.X(), dy = ap.Y() - bp.Y();
          if (std::sqrt(dx * dx + dy * dy) < 0.072) {
            bp.X(std::clamp(bp.X() + g_vel_x * dt, 0.04, 0.59));
            bp.Y(std::clamp(bp.Y() + g_vel_y * dt, 0.04, 0.59));
            bc->Data().Pos() = bp;
          }
        }
      }
    }
    // 处理传送请求（-1 表示无）
    for (int i = 0; i < 4; ++i) {
      if (g_set_x[i] >= 0 && entities_[i] != gz::sim::kNullEntity) {
        if (auto comp = _ecm.Component<gz::sim::components::Pose>(entities_[i])) {
          auto p = comp->Data();
          p.Pos().X(g_set_x[i]);
          p.Pos().Y(g_set_y[i]);
          comp->Data() = p;
        }
        g_set_x[i] = -1;
      }
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override {
    if (!Resolve(_ecm)) return;
    std::lock_guard<std::mutex> lk(g_mtx);
    g_sim_t = std::chrono::duration<double>(_info.simTime).count();
    for (int i = 0; i < 4; ++i) {
      if (entities_[i] == gz::sim::kNullEntity) continue;
      if (auto comp = _ecm.Component<gz::sim::components::Pose>(entities_[i])) {
        g_pos[i][0] = comp->Data().Pos().X();
        g_pos[i][1] = comp->Data().Pos().Y();
      }
    }
    const double dx = g_pos[0][0] - g_pos[1][0];
    const double dy = g_pos[0][1] - g_pos[1][1];
    g_contact = (std::sqrt(dx * dx + dy * dy) < 0.072) ? 1 : 0;
  }

 private:
  // ---------------- 实体解析（惰性，按模型名查 Name 组件） ----------------
  bool Resolve(const gz::sim::EntityComponentManager &_ecm) {
    if (resolved_) return true;
    const char *names[4] = {"agent", "green_box", "red_cross", "yellow_cross"};
    for (int i = 0; i < 4; ++i) {
      auto e = _ecm.EntityByComponents(gz::sim::components::Name(names[i]));
      if (e == gz::sim::kNullEntity) return false;
      entities_[i] = e;
    }
    resolved_ = true;
    gzmsg << "[vla_bridge] models resolved\n";
    return true;
  }

  // ---------------- TCP 服务 ----------------
  void ServerLoop(int _port) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(_port));
    if (bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      gzwarn << "[vla_bridge] bind failed on port " << _port << "\n";
      return;
    }
    listen(srv, 4);
    while (true) {
      int fd = accept(srv, nullptr, nullptr);
      if (fd < 0) continue;
      // 每客户端独立线程：探测/监控连接不会被长连接阻塞
      std::thread([this, fd]() {
        HandleClient(fd);
        close(fd);
      }).detach();
    }
  }

  static bool WriteAll(int _fd, const char *_data, size_t _n) {
    size_t off = 0;
    while (off < _n) {
      ssize_t k = send(_fd, _data + off, _n - off, 0);
      if (k <= 0) return false;
      off += static_cast<size_t>(k);
    }
    return true;
  }

  static bool SendLine(int _fd, const std::string &_s) {
    return WriteAll(_fd, (_s + "\n").c_str(), _s.size() + 1);
  }

  void HandleClient(int _fd) {
    std::string line;
    while (ReadLine(_fd, &line)) {
      std::istringstream iss(line);
      std::string cmd;
      iss >> cmd;
      if (cmd == "PING") {
        SendLine(_fd, "PONG vla_bridge 1.0");
      } else if (cmd == "STATE") {
        std::lock_guard<std::mutex> lk(g_mtx);
        std::ostringstream os;
        os << "STATE " << g_sim_t;
        for (int i = 0; i < 4; ++i)
          os << " " << g_pos[i][0] << " " << g_pos[i][1];
        os << " " << g_contact;
        SendLine(_fd, os.str());
      } else if (cmd == "VEL") {
        double vx, vy;
        iss >> vx >> vy;
        std::lock_guard<std::mutex> lk(g_mtx);
        g_vel_x = vx;
        g_vel_y = vy;
        g_vel_active = true;
        SendLine(_fd, "OK");
      } else if (cmd == "STOP") {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_vel_active = false;
        SendLine(_fd, "OK");
      } else if (cmd == "SET_POSE") {
        std::string who;
        double x, y;
        iss >> who >> x >> y;
        const int idx = who == "agent" ? 0 : who == "box" ? 1
                                        : who == "red"   ? 2
                                                         : 3;
        std::lock_guard<std::mutex> lk(g_mtx);
        g_set_x[idx] = x;
        g_set_y[idx] = y;
        SendLine(_fd, "OK");
      } else if (cmd == "RESET") {
        double v[8];
        for (double &x : v) iss >> x;
        std::lock_guard<std::mutex> lk(g_mtx);
        for (int i = 0; i < 4; ++i) {
          g_set_x[i] = v[i * 2];
          g_set_y[i] = v[i * 2 + 1];
        }
        g_vel_active = false;
        SendLine(_fd, "OK");
      } else if (cmd == "GET_IMAGE") {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (g_img.empty()) {
          SendLine(_fd, "ERR no_image");
          continue;
        }
        std::ostringstream os;
        os << "IMG " << g_img_w << " " << g_img_h << " " << g_img.size();
        SendLine(_fd, os.str());
        WriteAll(_fd, reinterpret_cast<const char *>(g_img.data()), g_img.size());
      } else if (cmd == "IMG_STATUS") {
        std::lock_guard<std::mutex> lk(g_mtx);
        std::ostringstream os;
        os << "IMG_STATUS " << g_img_frames << " ";
        for (size_t i = 0; i < topics_.size(); ++i)
          os << topics_[i] << "=" << g_topic_hits[i] << (i + 1 < topics_.size() ? "," : "");
        SendLine(_fd, os.str());
      } else if (cmd == "BYE") {
        break;
      } else {
        SendLine(_fd, "ERR unknown");
      }
    }
  }

  static bool ReadLine(int _fd, std::string *_line) {
    static thread_local char buf[4096];
    static thread_local int pos = 0, len = 0;
    _line->clear();
    while (true) {
      if (pos >= len) {
        len = static_cast<int>(read(_fd, buf, sizeof(buf)));
        pos = 0;
        if (len <= 0) return false;
      }
      char ch = buf[pos++];
      if (ch == '\n') return true;
      if (ch != '\r') _line->push_back(ch);
      if (_line->size() > 512) return false;
    }
  }

  gz::sim::Entity entities_[4] = {gz::sim::kNullEntity, gz::sim::kNullEntity,
                                  gz::sim::kNullEntity, gz::sim::kNullEntity};
  bool resolved_ = false;
  std::unique_ptr<gz::transport::Node> node_;
  std::vector<std::string> topics_;
  std::string cam_topic_;
  std::thread tcp_thread_;
};

}  // namespace vla_bridge

GZ_ADD_PLUGIN(vla_bridge::VLABridge, gz::sim::System,
              vla_bridge::VLABridge::ISystemConfigure,
              vla_bridge::VLABridge::ISystemPreUpdate,
              vla_bridge::VLABridge::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(vla_bridge::VLABridge, "vla_bridge::VLABridge")
