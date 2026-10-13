#include <chrono>
#include <memory>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;
using std_msgs::msg::String;

// Owns the scene (objects, coloured containers, gray container). Loads it from a YAML file,
// validates every change against the geometry rules and the own IK/planner, and publishes it:
//
//   scene/config   (latched)  the scene as YAML, with a revision number; read by vision and sequencer
//   scene/state    (latched)  scene + destinations + reachability map + plan summary, for the GUI
//   scene/reply               result of each command: {id, ok, message}
//   scene/command  (input)    "ID|command" or "command", see README for the command list
//   sequencer/state (input)   edits are refused while the arm is moving
class SceneServer : public rclcpp::Node {
 public:
  SceneServer() : Node("scene_server") {
    declare_parameter("scene_file", "");
    declare_parameter("seed", -1);
    const int seed = static_cast<int>(get_parameter("seed").as_int());
    file_ = get_parameter("scene_file").as_string();
    model_ = std::make_unique<SceneModel>(seed >= 0 ? static_cast<unsigned>(seed) : std::random_device{}());

    message_ = "built-in default scene";
    if (!file_.empty()) {
      const auto r = model_->load(file_);
      if (r.ok) {
        message_ = r.message;
        RCLCPP_INFO(get_logger(), "%s: %zu objects, %zu containers", r.message.c_str(), model_->scene().objects.size(),
                    model_->scene().containers.size());
      } else {
        ok_ = false;
        message_ = "scene file '" + file_ + "' rejected: " + r.message + " - using the built-in default scene";
        RCLCPP_ERROR(get_logger(), "%s", message_.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Computing the reachability map with the own IK ...");
    model_->reachGrid();

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    config_pub_ = create_publisher<String>("scene/config", latched);
    state_pub_ = create_publisher<String>("scene/state", latched);
    reply_pub_ = create_publisher<String>("scene/reply", 10);
    cmd_sub_ = create_subscription<String>("scene/command", 10, [this](String::SharedPtr m) { onCommand(m->data); });
    seq_sub_ = create_subscription<String>("sequencer/state", 10, [this](String::SharedPtr m) {
      if (m->data == seq_) return;
      seq_ = m->data;
      publishState();
    });
    publishConfig();
    publishState();
    RCLCPP_INFO(get_logger(), "scene_server ready (revision %d)", model_->scene().revision);
  }

 private:
  void publishConfig() {
    String m;
    m.data = sceneToYaml(model_->scene(), true);
    config_pub_->publish(m);
  }
  void publishState() {
    String m;
    m.data = model_->stateYaml(seq_, ok_, message_);
    state_pub_->publish(m);
  }
  void publishReply(const std::string& id, const SceneModel::Reply& r) {
    YAML::Emitter e;
    e << YAML::Flow << YAML::BeginMap << YAML::Key << "id" << YAML::Value << id << YAML::Key << "ok" << YAML::Value
      << r.ok << YAML::Key << "message" << YAML::Value << r.message << YAML::EndMap;
    String m;
    m.data = e.c_str();
    reply_pub_->publish(m);
  }

  void onCommand(const std::string& raw) {
    std::string id, cmd = raw;
    const size_t bar = raw.find('|');
    if (bar != std::string::npos && bar > 0 && raw.find_first_not_of("0123456789", 0) == bar) {
      id = raw.substr(0, bar);
      cmd = raw.substr(bar + 1);
    }
    SceneModel::Reply r;
    const bool check_only = cmd.rfind("check_", 0) == 0 || cmd.rfind("save", 0) == 0;
    if (seq_ == "running" && !check_only) {
      r = {false, false, "the arm is moving: edit the scene when the cycle has finished"};
    } else if (cmd == "reset") {
      r = file_.empty() ? model_->setFromYaml(sceneToYaml(defaultScene())) : model_->load(file_);
      if (r.ok) r.message = "scene reset";
    } else {
      r = model_->execute(cmd);
    }
    if (!check_only) {
      ok_ = r.ok;
      message_ = r.message;
    }
    if (r.ok && r.changed) publishConfig();
    if (!check_only) {  // every edit is logged, so the launch terminal shows what happened
      if (r.ok)
        RCLCPP_INFO(get_logger(), "command '%s': OK, %s (scene revision %d)", cmd.c_str(), r.message.c_str(),
                    model_->scene().revision);
      else
        RCLCPP_WARN(get_logger(), "command '%s': REFUSED, %s", cmd.c_str(), r.message.c_str());
    }
    publishReply(id, r);
    if (!check_only) publishState();
  }

  std::unique_ptr<SceneModel> model_;
  std::string file_, seq_{"waiting"}, message_;
  bool ok_{true};
  rclcpp::Publisher<String>::SharedPtr config_pub_, state_pub_, reply_pub_;
  rclcpp::Subscription<String>::SharedPtr cmd_sub_, seq_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneServer>());
  rclcpp::shutdown();
  return 0;
}
