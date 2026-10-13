#pragma once
// Scene description (objects, containers, the gray catch-all container), its YAML form,
// validation, colour matching, random placement and the edit commands used by scene_server.
// ROS-free so it can be tested on its own.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sorting_cell_controller/pick_place.hpp"

namespace sorting_cell {

// ---------------------------------------------------------------- colours
inline const std::vector<std::string>& palette() {
  static const std::vector<std::string> p = {"red", "orange", "yellow", "green", "cyan", "blue", "purple"};
  return p;
}
inline bool isPaletteColor(const std::string& c) {
  const auto& p = palette();
  return std::find(p.begin(), p.end(), c) != p.end();
}
inline void colorRgb(const std::string& n, float& r, float& g, float& b) {
  r = g = b = 0.6f;
  if (n == "red") { r = 0.86f; g = 0.16f; b = 0.16f; }
  else if (n == "orange") { r = 0.96f; g = 0.55f; b = 0.12f; }
  else if (n == "yellow") { r = 0.90f; g = 0.86f; b = 0.16f; }
  else if (n == "green") { r = 0.24f; g = 0.75f; b = 0.24f; }
  else if (n == "cyan") { r = 0.16f; g = 0.78f; b = 0.86f; }
  else if (n == "blue") { r = 0.16f; g = 0.35f; b = 0.86f; }
  else if (n == "purple") { r = 0.59f; g = 0.20f; b = 0.74f; }
  else if (n == kGrayName) { r = g = b = 0.78f; }
}

// ---------------------------------------------------------------- geometry of the cell
constexpr double kCubeSizeM = 0.04;
constexpr double kBinSizeM = 0.14;                 // square footprint of every container
constexpr double kObjXMin = -0.77, kObjXMax = -0.23;  // inside the camera view with a cube-diagonal margin
constexpr double kObjYMin = -0.37, kObjYMax = 0.37;
constexpr double kBinXMin = -0.73, kBinXMax = -0.27;  // whole footprint on the table
constexpr double kBinYMin = -0.38, kBinYMax = 0.38;
constexpr double kTableXMin = -0.80, kTableXMax = -0.20, kTableYMin = -0.45, kTableYMax = 0.45;
constexpr double kObjObjClear = 0.09;              // centre distance between two objects
constexpr double kBinBinClear = kBinSizeM + 0.02;  // Chebyshev centre distance between containers
constexpr double kObjBinClear = kBinSizeM / 2 + kCubeSizeM / 2 + 0.005;  // Chebyshev, object to container
constexpr size_t kMaxObjects = 8, kMaxContainers = 7;

// ---------------------------------------------------------------- scene
struct SceneObject {
  std::string color;
  double x{0}, y{0}, yaw{0};
};
struct SceneContainer {
  std::string color;
  double x{0}, y{0};
};
struct Scene {
  std::string name{"scene"};
  std::vector<SceneObject> objects;
  std::vector<SceneContainer> containers;
  double gray_x{-0.72}, gray_y{0.0};
  int revision{0};
};

inline Scene defaultScene() {
  Scene s;
  s.name = "default";
  s.objects = {{"red", -0.40, -0.20, 0.0}, {"green", -0.45, 0.15, 0.5},
               {"blue", -0.58, -0.10, 1.0}, {"yellow", -0.62, 0.25, 0.3}};
  s.containers = {{"red", -0.30, -0.30}, {"green", -0.30, -0.10}, {"blue", -0.30, 0.10}, {"yellow", -0.30, 0.30}};
  return s;
}

inline std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
inline std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

inline double roundTo(double v, double q = 1e-6) { return std::round(v / q) * q + 0.0; }

// ---------------------------------------------------------------- colour matching
// The container an object goes to: the one with the same colour, otherwise the gray one.
// Returns the container index, or -1 for the gray container.
inline int matchContainer(const Scene& s, const std::string& color) {
  for (size_t i = 0; i < s.containers.size(); ++i)
    if (s.containers[i].color == color) return static_cast<int>(i);
  return -1;
}
inline std::vector<std::string> destinations(const Scene& s) {
  std::vector<std::string> d;
  for (const auto& o : s.objects) {
    const int c = matchContainer(s, o.color);
    d.push_back(c < 0 ? std::string(kGrayName) : s.containers[c].color);
  }
  return d;
}

inline PickPlaceConfig planConfig(const Scene& s) {
  PickPlaceConfig cfg;
  cfg.bins.clear();
  for (const auto& c : s.containers) cfg.bins[c.color] = Eigen::Vector3d(c.x, c.y, 0.02);
  cfg.has_gray_bin = true;
  cfg.gray_bin = Eigen::Vector3d(s.gray_x, s.gray_y, 0.02);
  return cfg;
}
inline std::vector<CubeTarget> cubeTargets(const Scene& s) {
  std::vector<CubeTarget> v;
  for (const auto& o : s.objects) v.push_back({o.color, Eigen::Vector3d(o.x, o.y, kCubeSizeM / 2), o.yaw});
  return v;
}

// ---------------------------------------------------------------- YAML in / out
inline std::string sceneToYaml(const Scene& s, bool with_revision = false) {
  YAML::Emitter out;
  out.SetDoublePrecision(6);
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 1;
  out << YAML::Key << "name" << YAML::Value << s.name;
  if (with_revision) out << YAML::Key << "revision" << YAML::Value << s.revision;
  out << YAML::Key << "objects" << YAML::Value << YAML::BeginSeq;
  for (const auto& o : s.objects) {
    out << YAML::Flow << YAML::BeginMap << YAML::Key << "color" << YAML::Value << o.color << YAML::Key << "x"
        << YAML::Value << roundTo(o.x) << YAML::Key << "y" << YAML::Value << roundTo(o.y) << YAML::Key << "yaw"
        << YAML::Value << roundTo(o.yaw) << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "containers" << YAML::Value << YAML::BeginSeq;
  for (const auto& c : s.containers) {
    out << YAML::Flow << YAML::BeginMap << YAML::Key << "color" << YAML::Value << c.color << YAML::Key << "x"
        << YAML::Value << roundTo(c.x) << YAML::Key << "y" << YAML::Value << roundTo(c.y) << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "gray_container" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "x"
      << YAML::Value << roundTo(s.gray_x) << YAML::Key << "y" << YAML::Value << roundTo(s.gray_y) << YAML::EndMap;
  out << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

namespace detail {
inline bool onlyKeys(const YAML::Node& n, const std::set<std::string>& allowed, const std::string& where,
                     std::string& err) {
  for (auto it = n.begin(); it != n.end(); ++it) {
    const std::string k = it->first.as<std::string>();
    if (!allowed.count(k)) {
      err = where + ": unknown key '" + k + "'";
      return false;
    }
  }
  return true;
}
inline bool number(const YAML::Node& n, const std::string& key, const std::string& where, double& v,
                   std::string& err, bool required = true, double dflt = 0.0) {
  if (!n[key]) {
    if (required) {
      err = where + ": missing '" + key + "'";
      return false;
    }
    v = dflt;
    return true;
  }
  try {
    v = n[key].as<double>();
  } catch (const YAML::Exception&) {
    err = where + ": '" + key + "' must be a number";
    return false;
  }
  if (!std::isfinite(v)) {
    err = where + ": '" + key + "' must be finite";
    return false;
  }
  return true;
}
}  // namespace detail

// Parses and checks the file format only (types, keys). Geometry is checked by validateScene().
inline bool sceneFromYaml(const std::string& text, Scene& out, std::string& err) {
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    err = std::string("not valid YAML: ") + e.what();
    return false;
  }
  if (!root.IsMap()) {
    err = "the scene file must be a YAML map";
    return false;
  }
  using detail::number;
  using detail::onlyKeys;
  if (!onlyKeys(root, {"version", "name", "revision", "objects", "containers", "gray_container"}, "scene", err))
    return false;
  if (!root["version"] || !root["version"].IsScalar() || root["version"].as<std::string>() != "1") {
    err = "scene: 'version' must be 1";
    return false;
  }
  Scene s;
  try {
    if (root["name"]) s.name = root["name"].as<std::string>();
    if (root["revision"]) s.revision = root["revision"].as<int>();
  } catch (const YAML::Exception&) {
    err = "scene: 'name' must be text and 'revision' an integer";
    return false;
  }
  if (!root["objects"] || !root["objects"].IsSequence()) {
    err = "scene: 'objects' must be a list";
    return false;
  }
  for (size_t i = 0; i < root["objects"].size(); ++i) {
    const YAML::Node n = root["objects"][i];
    const std::string where = "objects[" + std::to_string(i) + "]";
    if (!n.IsMap() || !onlyKeys(n, {"color", "x", "y", "yaw"}, where, err)) {
      if (err.empty()) err = where + ": must be a map with color, x, y and optional yaw";
      return false;
    }
    SceneObject o;
    if (!n["color"] || !n["color"].IsScalar()) {
      err = where + ": missing 'color'";
      return false;
    }
    o.color = n["color"].as<std::string>();
    if (!number(n, "x", where, o.x, err) || !number(n, "y", where, o.y, err) ||
        !number(n, "yaw", where, o.yaw, err, false, 0.0))
      return false;
    s.objects.push_back(o);
  }
  if (root["containers"] && !root["containers"].IsSequence()) {
    err = "scene: 'containers' must be a list";
    return false;
  }
  if (root["containers"]) {
    for (size_t i = 0; i < root["containers"].size(); ++i) {
      const YAML::Node n = root["containers"][i];
      const std::string where = "containers[" + std::to_string(i) + "]";
      if (!n.IsMap() || !onlyKeys(n, {"color", "x", "y"}, where, err)) {
        if (err.empty()) err = where + ": must be a map with color, x and y";
        return false;
      }
      SceneContainer c;
      if (!n["color"] || !n["color"].IsScalar()) {
        err = where + ": missing 'color'";
        return false;
      }
      c.color = n["color"].as<std::string>();
      if (!number(n, "x", where, c.x, err) || !number(n, "y", where, c.y, err)) return false;
      s.containers.push_back(c);
    }
  }
  if (root["gray_container"]) {
    const YAML::Node n = root["gray_container"];
    if (!n.IsMap() || !onlyKeys(n, {"x", "y"}, "gray_container", err)) {
      if (err.empty()) err = "gray_container: must be a map with x and y";
      return false;
    }
    if (!number(n, "x", "gray_container", s.gray_x, err) || !number(n, "y", "gray_container", s.gray_y, err))
      return false;
  }
  out = s;
  return true;
}

// ---------------------------------------------------------------- validation
struct BinRef {
  std::string name;
  double x, y;
};
inline std::vector<BinRef> allBins(const Scene& s) {
  std::vector<BinRef> b;
  for (const auto& c : s.containers) b.push_back({c.color, c.x, c.y});
  b.push_back({kGrayName, s.gray_x, s.gray_y});
  return b;
}

// Returns "" when the scene is well formed and has no overlaps, else the first problem found.
inline std::string validateScene(const Scene& s) {
  if (s.objects.empty()) return "the scene has no objects";
  if (s.objects.size() > kMaxObjects) return fmt("at most %zu objects are allowed", kMaxObjects);
  if (s.containers.size() > kMaxContainers) return fmt("at most %zu coloured containers are allowed", kMaxContainers);
  for (size_t i = 0; i < s.objects.size(); ++i) {
    const auto& o = s.objects[i];
    if (o.color == kGrayName) return fmt("object %zu: gray cannot be an object colour (it is the table colour)", i);
    if (!isPaletteColor(o.color)) return fmt("object %zu: unknown colour '%s'", i, o.color.c_str());
    if (!std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.yaw))
      return fmt("object %zu: position must be finite", i);
    if (o.x < kObjXMin || o.x > kObjXMax || o.y < kObjYMin || o.y > kObjYMax)
      return fmt("object %zu (%s) at (%.2f, %.2f) is outside the camera area (x %.2f..%.2f, y %.2f..%.2f)", i,
                 o.color.c_str(), o.x, o.y, kObjXMin, kObjXMax, kObjYMin, kObjYMax);
  }
  std::set<std::string> seen;
  for (size_t i = 0; i < s.containers.size(); ++i) {
    const auto& c = s.containers[i];
    if (c.color == kGrayName) return fmt("container %zu: the gray container is set with 'gray_container'", i);
    if (!isPaletteColor(c.color)) return fmt("container %zu: unknown colour '%s'", i, c.color.c_str());
    if (!seen.insert(c.color).second) return fmt("container %zu: there are two '%s' containers", i, c.color.c_str());
  }
  for (const auto& b : allBins(s)) {
    if (!std::isfinite(b.x) || !std::isfinite(b.y)) return "container positions must be finite";
    if (b.x < kBinXMin || b.x > kBinXMax || b.y < kBinYMin || b.y > kBinYMax)
      return fmt("%s container at (%.2f, %.2f) is not fully on the table (x %.2f..%.2f, y %.2f..%.2f)",
                 b.name.c_str(), b.x, b.y, kBinXMin, kBinXMax, kBinYMin, kBinYMax);
  }
  const auto bins = allBins(s);
  for (size_t i = 0; i < bins.size(); ++i)
    for (size_t j = i + 1; j < bins.size(); ++j)
      if (std::max(std::abs(bins[i].x - bins[j].x), std::abs(bins[i].y - bins[j].y)) < kBinBinClear - 1e-9)
        return fmt("the %s and %s containers overlap", bins[i].name.c_str(), bins[j].name.c_str());
  for (size_t i = 0; i < s.objects.size(); ++i) {
    for (size_t j = i + 1; j < s.objects.size(); ++j)
      if (std::hypot(s.objects[i].x - s.objects[j].x, s.objects[i].y - s.objects[j].y) < kObjObjClear - 1e-9)
        return fmt("objects %zu and %zu are too close together (min %.2f m)", i, j, kObjObjClear);
    for (const auto& b : bins)
      if (std::max(std::abs(s.objects[i].x - b.x), std::abs(s.objects[i].y - b.y)) < kObjBinClear - 1e-9)
        return fmt("object %zu (%s) lies on the %s container", i, s.objects[i].color.c_str(), b.name.c_str());
  }
  return "";
}

// ---------------------------------------------------------------- reachability with the own IK
// Can the tool go home -> hover -> grasp height -> hover over (x, y) with the given yaw?
inline bool pickReachable(double x, double y, double yaw = 0.0) {
  const PickPlaceConfig cfg;
  Vector6d q = homeJoints();
  const Eigen::Vector3d hover(x, y, cfg.hover_z), grasp(x, y, cfg.grasp_z);
  const Eigen::Matrix4d poses[3] = {toolDownPose(hover, yaw), toolDownPose(grasp, yaw), toolDownPose(hover, yaw)};
  for (int k = 0; k < 3; ++k) {
    const JointTrajectory tr = planCartesianMove(q, poses[k], k == 0 ? cfg.fast : cfg.slow, cfg.dt);
    if (!tr.ok) return false;
    q = tr.q.back();
  }
  return true;
}

// The complete sort planned by the real planner. "" when it works, else the planner's message.
inline std::string checkPlan(const Scene& s, double* seconds = nullptr) {
  const PickPlaceConfig cfg = planConfig(s);
  const PickPlacePlan plan = planPickPlace(cubeTargets(s), cfg);
  if (!plan.ok) return "cannot be sorted: " + plan.error;
  if (seconds) *seconds = static_cast<double>(plan.q.size()) * cfg.dt;
  return "";
}

// ---------------------------------------------------------------- model + edit commands
class SceneModel {
 public:
  struct Reply {
    bool ok{true};
    bool changed{false};
    std::string message;
  };

  explicit SceneModel(unsigned seed = std::random_device{}()) : rng_(seed) {
    scene_ = defaultScene();
    scene_.revision = 1;
    plan_error_ = checkPlan(scene_, &plan_seconds_);
  }

  const Scene& scene() const { return scene_; }
  double planSeconds() const { return plan_seconds_; }
  const std::string& planError() const { return plan_error_; }

  // Replaces the scene if the text parses, validates and can be planned.
  Reply setFromYaml(const std::string& text) {
    Scene s;
    std::string err;
    if (!sceneFromYaml(text, s, err)) return {false, false, err};
    return adopt(s, "scene loaded");
  }
  Reply load(const std::string& path) {
    std::ifstream f(expandUser(path));
    if (!f) return {false, false, "cannot read '" + path + "'"};
    std::stringstream ss;
    ss << f.rdbuf();
    Reply r = setFromYaml(ss.str());
    if (r.ok) r.message = "loaded '" + path + "'";
    return r;
  }
  Reply save(const std::string& path) const {
    std::ofstream f(expandUser(path));
    if (!f) return {false, false, "cannot write '" + path + "'"};
    f << sceneToYaml(scene_);
    if (!f) return {false, false, "write to '" + path + "' failed"};
    return {true, false, "saved '" + path + "'"};
  }

  // One text command; see the README for the list. Failed commands leave the scene untouched.
  Reply execute(const std::string& line) {
    std::istringstream is(line);
    std::vector<std::string> t;
    std::string w;
    while (is >> w) t.push_back(w);
    if (t.empty()) return {false, false, "empty command"};
    const std::string& c = t[0];
    std::string why;  // set when an index is out of range, more useful than the usage line
    auto idx = [&](const std::string& s, size_t n, size_t& i, const char* kind) {
      char* e = nullptr;
      const long v = std::strtol(s.c_str(), &e, 10);
      if (*e != 0 || v < 0 || static_cast<size_t>(v) >= n) {
        if (*e == 0 && !s.empty())
          why = n == 0 ? fmt("there is no %s %s (the scene has none)", kind, s.c_str())
                       : fmt("there is no %s %s (valid: 0 to %zu)", kind, s.c_str(), n - 1);
        return false;
      }
      i = static_cast<size_t>(v);
      return true;
    };
    auto num = [&](const std::string& s, double& v) {
      char* e = nullptr;
      v = std::strtod(s.c_str(), &e);
      return *e != 0 ? false : std::isfinite(v);
    };
    Scene s = scene_;
    size_t i = 0;
    double x = 0, y = 0, a = 0;

    if (c == "move_object" || c == "check_object") {
      if (t.size() != 4 || !idx(t[1], s.objects.size(), i, "object") || !num(t[2], x) || !num(t[3], y))
        return {false, false, why.empty() ? "usage: " + c + " INDEX X Y  (INDEX from 0)" : why};
      s.objects[i].x = x;
      s.objects[i].y = y;
      return finish(s, c == "check_object", "object " + std::to_string(i) + " moved");
    }
    if (c == "move_container" || c == "check_container") {
      if (t.size() != 4 || !idx(t[1], s.containers.size(), i, "container") || !num(t[2], x) || !num(t[3], y))
        return {false, false, why.empty() ? "usage: " + c + " INDEX X Y  (INDEX from 0)" : why};
      s.containers[i].x = x;
      s.containers[i].y = y;
      return finish(s, c == "check_container", "container " + std::to_string(i) + " moved");
    }
    if (c == "move_gray" || c == "check_gray") {
      if (t.size() != 3 || !num(t[1], x) || !num(t[2], y)) return {false, false, why.empty() ? "usage: " + c + " X Y" : why};
      s.gray_x = x;
      s.gray_y = y;
      return finish(s, c == "check_gray", "gray container moved");
    }
    if (c == "yaw_object") {
      if (t.size() != 3 || !idx(t[1], s.objects.size(), i, "object") || !num(t[2], a))
        return {false, false, why.empty() ? std::string("usage: yaw_object INDEX RADIANS") : why};
      s.objects[i].yaw = a;
      return finish(s, false, "object " + std::to_string(i) + " rotated");
    }
    if (c == "color_object") {
      if (t.size() != 3 || !idx(t[1], s.objects.size(), i, "object")) return {false, false, why.empty() ? std::string("usage: color_object INDEX COLOR") : why};
      s.objects[i].color = t[2];
      return finish(s, false, "object " + std::to_string(i) + " is now " + t[2]);
    }
    if (c == "color_container") {
      if (t.size() != 3 || !idx(t[1], s.containers.size(), i, "container"))
        return {false, false, why.empty() ? std::string("usage: color_container INDEX COLOR") : why};
      s.containers[i].color = t[2];
      return finish(s, false, "container " + std::to_string(i) + " is now " + t[2]);
    }
    if ((c == "add_object" || c == "add_container") && t.size() == 2) {  // no position: find a free one
      if (!placeNew(s, c == "add_container", t[1])) {
        const std::string v = validateScene(s);
        return {false, false, !v.empty() ? v : "no free reachable place found for a new " + t[1] + (c == "add_container" ? " container" : " object")};
      }
      return finish(s, false, t[1] + (c == "add_container" ? " container added" : " object added"));
    }
    if (c == "add_object") {
      if ((t.size() != 4 && t.size() != 5) || !num(t[2], x) || !num(t[3], y) || (t.size() == 5 && !num(t[4], a)))
        return {false, false, why.empty() ? std::string("usage: add_object COLOR X Y [YAW]") : why};
      s.objects.push_back({t[1], x, y, a});
      return finish(s, false, "object added");
    }
    if (c == "add_container") {
      if (t.size() != 4 || !num(t[2], x) || !num(t[3], y)) return {false, false, why.empty() ? std::string("usage: add_container COLOR X Y") : why};
      s.containers.push_back({t[1], x, y});
      return finish(s, false, "container added");
    }
    if (c == "remove_object") {
      if (t.size() != 2 || !idx(t[1], s.objects.size(), i, "object")) return {false, false, why.empty() ? std::string("usage: remove_object INDEX") : why};
      s.objects.erase(s.objects.begin() + static_cast<long>(i));
      return finish(s, false, "object removed");
    }
    if (c == "remove_container") {
      if (t.size() != 2 || !idx(t[1], s.containers.size(), i, "container")) return {false, false, why.empty() ? std::string("usage: remove_container INDEX") : why};
      s.containers.erase(s.containers.begin() + static_cast<long>(i));
      return finish(s, false, "container removed");
    }
    if (c == "random") {
      if (t.size() != 2 || (t[1] != "objects" && t[1] != "containers" && t[1] != "all"))
        return {false, false, why.empty() ? std::string("usage: random objects|containers|all") : why};
      const int what = t[1] == "objects" ? 1 : (t[1] == "containers" ? 2 : 3);
      if (!randomize(s, what)) return {false, false, "could not find a valid random arrangement, try again"};
      return finish(s, false, "random " + t[1] + " placed");
    }
    if (c == "load") {
      if (t.size() < 2) return {false, false, why.empty() ? std::string("usage: load PATH") : why};
      return load(rest(line, c));
    }
    if (c == "save") {
      if (t.size() < 2) return {false, false, why.empty() ? std::string("usage: save PATH") : why};
      return save(rest(line, c));
    }
    return {false, false, "unknown command '" + c + "'"};
  }

  // The state published for the GUI: scene, destinations, regions, reachability map, plan summary.
  std::string stateYaml(const std::string& seq, bool ok, const std::string& message) const {
    YAML::Emitter o;
    o.SetDoublePrecision(6);
    o << YAML::BeginMap;
    o << YAML::Key << "revision" << YAML::Value << scene_.revision;
    o << YAML::Key << "ok" << YAML::Value << ok;
    o << YAML::Key << "message" << YAML::Value << message;
    o << YAML::Key << "seq" << YAML::Value << seq;
    o << YAML::Key << "name" << YAML::Value << scene_.name;
    o << YAML::Key << "plan" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "ok" << YAML::Value
      << plan_error_.empty() << YAML::Key << "seconds" << YAML::Value << roundTo(plan_seconds_, 0.1) << YAML::Key
      << "error" << YAML::Value << plan_error_ << YAML::EndMap;
    auto region = [&](const char* k, double x0, double x1, double y0, double y1) {
      o << YAML::Key << k << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "xmin" << YAML::Value << x0
        << YAML::Key << "xmax" << YAML::Value << x1 << YAML::Key << "ymin" << YAML::Value << y0 << YAML::Key << "ymax"
        << YAML::Value << y1 << YAML::EndMap;
    };
    o << YAML::Key << "regions" << YAML::Value << YAML::BeginMap;
    region("table", kTableXMin, kTableXMax, kTableYMin, kTableYMax);
    region("object", kObjXMin, kObjXMax, kObjYMin, kObjYMax);
    region("container", kBinXMin, kBinXMax, kBinYMin, kBinYMax);
    o << YAML::EndMap;
    o << YAML::Key << "sizes" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "cube" << YAML::Value
      << kCubeSizeM << YAML::Key << "container" << YAML::Value << kBinSizeM << YAML::EndMap;
    o << YAML::Key << "palette" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& p : palette()) o << p;
    o << YAML::EndSeq;
    const auto dest = destinations(scene_);
    o << YAML::Key << "objects" << YAML::Value << YAML::BeginSeq;
    for (size_t i = 0; i < scene_.objects.size(); ++i) {
      const auto& ob = scene_.objects[i];
      o << YAML::Flow << YAML::BeginMap << YAML::Key << "color" << YAML::Value << ob.color << YAML::Key << "x"
        << YAML::Value << roundTo(ob.x) << YAML::Key << "y" << YAML::Value << roundTo(ob.y) << YAML::Key << "yaw"
        << YAML::Value << roundTo(ob.yaw) << YAML::Key << "dest" << YAML::Value << dest[i] << YAML::EndMap;
    }
    o << YAML::EndSeq;
    o << YAML::Key << "containers" << YAML::Value << YAML::BeginSeq;
    for (const auto& c : scene_.containers)
      o << YAML::Flow << YAML::BeginMap << YAML::Key << "color" << YAML::Value << c.color << YAML::Key << "x"
        << YAML::Value << roundTo(c.x) << YAML::Key << "y" << YAML::Value << roundTo(c.y) << YAML::EndMap;
    o << YAML::EndSeq;
    o << YAML::Key << "gray" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "x" << YAML::Value
      << roundTo(scene_.gray_x) << YAML::Key << "y" << YAML::Value << roundTo(scene_.gray_y) << YAML::EndMap;
    const auto& g = reachGrid();
    o << YAML::Key << "reach" << YAML::Value << YAML::BeginMap;
    o << YAML::Key << "x0" << YAML::Value << kReachXMax << YAML::Key << "y0" << YAML::Value << kReachYMin
      << YAML::Key << "step" << YAML::Value << kGridStep << YAML::Key << "nx" << YAML::Value << g.size()
      << YAML::Key << "ny" << YAML::Value << (g.empty() ? 0 : g[0].size()) << YAML::Key << "rows" << YAML::Value
      << YAML::BeginSeq;
    for (const auto& r : g) o << YAML::DoubleQuoted << r;  // quoted: an all-ones row must stay text
    o << YAML::EndSeq << YAML::EndMap;
    o << YAML::EndMap;
    return std::string(o.c_str()) + "\n";
  }

  // Pick reachability map (home -> hover -> grasp, tool yaw 0) around the whole arm, not only the
  // table. Row i is x = kReachXMax - i*step, column j is y = kReachYMin + j*step, '1' = reachable.
  const std::vector<std::string>& reachGrid() const {
    if (grid_.empty()) {
      const int nx = static_cast<int>(std::round((kReachXMax - kReachXMin) / kGridStep)) + 1;
      const int ny = static_cast<int>(std::round((kReachYMax - kReachYMin) / kGridStep)) + 1;
      for (int i = 0; i < nx; ++i) {
        std::string row;
        for (int j = 0; j < ny; ++j)
          row += pickReachable(kReachXMax - i * kGridStep, kReachYMin + j * kGridStep) ? '1' : '0';
        grid_.push_back(row);
      }
    }
    return grid_;
  }

  // The text after the command word, trimmed (paths may contain spaces).
  static std::string rest(const std::string& line, const std::string& cmd) {
    std::string r = line.substr(line.find(cmd) + cmd.size());
    const size_t a = r.find_first_not_of(" \t");
    const size_t b = r.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? "" : r.substr(a, b - a + 1);
  }

  static std::string expandUser(const std::string& p) {
    if (!p.empty() && p[0] == '~' && (p.size() == 1 || p[1] == '/')) {
      const char* h = std::getenv("HOME");
      if (h) return std::string(h) + p.substr(1);
    }
    return p;
  }

 private:
  static constexpr double kGridStep = 0.025;
  static constexpr double kReachXMin = -1.00, kReachXMax = 0.05, kReachYMin = -0.80, kReachYMax = 0.80;

  Reply adopt(Scene s, const std::string& what) {
    const std::string v = validateScene(s);
    if (!v.empty()) return {false, false, v};
    double secs = 0;
    const std::string p = checkPlan(s, &secs);
    if (!p.empty()) return {false, false, p};
    s.revision = scene_.revision + 1;
    scene_ = s;
    plan_seconds_ = secs;
    plan_error_.clear();
    return {true, true, what};
  }
  Reply finish(const Scene& s, bool check_only, const std::string& what) {
    if (!check_only) return adopt(s, what);
    const std::string v = validateScene(s);
    if (!v.empty()) return {false, false, v};
    const std::string p = checkPlan(s);
    if (!p.empty()) return {false, false, p};
    return {true, false, "position is valid"};
  }

  // Appends one object or container of the given colour at a random free, reachable place.
  // On failure s holds the scene with the item appended at its last tried place (for the message).
  bool placeNew(Scene& s, bool container, const std::string& color) {
    std::uniform_real_distribution<double> ox(kObjXMin, kObjXMax), oy(kObjYMin, kObjYMax);
    std::uniform_real_distribution<double> bx(kBinXMin, kBinXMax), by(kBinYMin, kBinYMax);
    std::uniform_real_distribution<double> uyaw(0.0, M_PI / 2);
    if (container) s.containers.push_back({color, 0.0, 0.0});
    else s.objects.push_back({color, 0.0, 0.0, 0.0});
    for (int tries = 0; tries < 400; ++tries) {
      if (container) {
        s.containers.back().x = bx(rng_);
        s.containers.back().y = by(rng_);
        if (!pickReachable(s.containers.back().x, s.containers.back().y)) continue;
      } else {
        auto& o = s.objects.back();
        o.x = ox(rng_);
        o.y = oy(rng_);
        o.yaw = uyaw(rng_);
        if (!pickReachable(o.x, o.y, foldCubeYaw(o.yaw))) continue;
      }
      const std::string v = validateScene(s);
      if (!v.empty()) {
        // colour, count and duplicate errors do not depend on the position: stop at once
        if (v.find("colour") != std::string::npos || v.find("at most") != std::string::npos ||
            v.find("two '") != std::string::npos)
          return false;
        continue;
      }
      if (checkPlan(s).empty()) return true;
    }
    return false;
  }

  bool randomize(Scene& s, int what) {
    std::uniform_real_distribution<double> ox(kObjXMin, kObjXMax), oy(kObjYMin, kObjYMax);
    std::uniform_real_distribution<double> bx(kBinXMin, kBinXMax), by(kBinYMin, kBinYMax);
    std::uniform_real_distribution<double> uyaw(0.0, M_PI / 2);
    for (int attempt = 0; attempt < 40; ++attempt) {
      Scene t = s;
      bool ok = true;
      if (what & 2) {  // containers (and the gray one), one at a time against the ones already placed
        std::vector<std::pair<double, double>> placed;
        const size_t n = t.containers.size() + 1;
        for (size_t k = 0; k < n && ok; ++k) {
          bool found = false;
          for (int tries = 0; tries < 300 && !found; ++tries) {
            const double x = bx(rng_), y = by(rng_);
            bool fine = pickReachable(x, y);
            for (const auto& p : placed)
              fine = fine && std::max(std::abs(p.first - x), std::abs(p.second - y)) >= kBinBinClear;
            if (!(what & 1))  // objects stay where they are: keep clear of them
              for (const auto& ob : t.objects)
                fine = fine && std::max(std::abs(ob.x - x), std::abs(ob.y - y)) >= kObjBinClear;
            if (!fine) continue;
            placed.push_back({x, y});
            if (k < t.containers.size()) {
              t.containers[k].x = x;
              t.containers[k].y = y;
            } else {
              t.gray_x = x;
              t.gray_y = y;
            }
            found = true;
          }
          ok = found;
        }
      }
      if (ok && (what & 1)) {
        const auto bins = allBins(t);
        std::vector<std::pair<double, double>> placed;
        for (auto& ob : t.objects) {
          bool found = false;
          for (int tries = 0; tries < 300 && !found; ++tries) {
            const double x = ox(rng_), y = oy(rng_), yaw = uyaw(rng_);
            bool fine = pickReachable(x, y, foldCubeYaw(yaw));
            for (const auto& p : placed) fine = fine && std::hypot(p.first - x, p.second - y) >= kObjObjClear;
            for (const auto& b : bins)
              fine = fine && std::max(std::abs(b.x - x), std::abs(b.y - y)) >= kObjBinClear;
            if (!fine) continue;
            placed.push_back({x, y});
            ob.x = x;
            ob.y = y;
            ob.yaw = yaw;
            found = true;
          }
          ok = ok && found;
          if (!ok) break;
        }
      }
      if (ok && validateScene(t).empty() && checkPlan(t).empty()) {
        s = t;
        return true;
      }
    }
    return false;
  }

  Scene scene_;
  double plan_seconds_{0.0};
  std::string plan_error_;
  mutable std::mt19937 rng_;
  mutable std::vector<std::string> grid_;
};

}  // namespace sorting_cell
