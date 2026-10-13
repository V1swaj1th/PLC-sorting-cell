#include <cstdio>
#include <fstream>
#include <functional>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;

static int passed = 0, failed = 0;
static void expect(const std::string& what, bool ok) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  (ok ? passed : failed)++;
}
static bool has(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

int main(int argc, char** argv) {
  std::printf("[1] default scene, file round trip\n");
  SceneModel m(1);
  expect("default scene is valid and plans", validateScene(m.scene()).empty() && m.planError().empty());
  expect("default plan takes about 34 s", m.planSeconds() > 30.0 && m.planSeconds() < 40.0);
  Scene back;
  std::string err;
  expect("YAML round trip", sceneFromYaml(sceneToYaml(m.scene()), back, err) && back.objects.size() == 4 &&
                                back.containers.size() == 4 && std::abs(back.gray_x - m.scene().gray_x) < 1e-9);
  expect("saved text starts with the version", sceneToYaml(m.scene()).rfind("version: 1", 0) == 0);
  if (argc > 1) {
    std::ofstream(argv[1]) << sceneToYaml(m.scene());
    std::printf("      wrote %s\n", argv[1]);
  }

  std::printf("[2] colour matching and the gray container\n");
  Scene s = defaultScene();
  expect("every default object has its own container", destinations(s) == std::vector<std::string>{"red", "green", "blue", "yellow"});
  s.containers.pop_back();  // no yellow container
  auto d = destinations(s);
  expect("an object without a matching container goes to gray", d[3] == "gray" && d[0] == "red");
  s.objects[0].color = "purple";
  s.objects[1].color = "orange";
  d = destinations(s);
  expect("colours with no container (purple, orange) go to gray", d[0] == "gray" && d[1] == "gray" && d[2] == "blue");
  PickPlacePlan plan = planPickPlace(cubeTargets(s), planConfig(s));
  expect("the plan succeeds", plan.ok);
  expect("plan destinations match the matching rule", plan.dest == d);
  const Eigen::Vector3d g(s.gray_x, s.gray_y, 0.02);
  expect("gray cubes are dropped at distinct slots inside the gray container",
         plan.ok && (plan.place_pos[0] - plan.place_pos[1]).norm() > 0.04 && (plan.place_pos[0] - g).head<2>().cwiseAbs().maxCoeff() < 0.05 + 1e-9 &&
             (plan.place_pos[3] - g).head<2>().cwiseAbs().maxCoeff() < 0.05 + 1e-9);
  expect("the red-less scene: blue goes to the blue container, not gray", (plan.place_pos[2] - Eigen::Vector3d(-0.30, 0.10, 0.02)).norm() < 1e-9);
  Scene none = defaultScene();
  none.containers.clear();
  plan = planPickPlace(cubeTargets(none), planConfig(none));
  expect("with no coloured container everything goes to gray (4 slots)", plan.ok && plan.dest == std::vector<std::string>(4, "gray"));
  Scene many = none;
  many.objects.clear();
  for (double x : {-0.40, -0.52})
    for (double y : {-0.30, -0.10, 0.10, 0.30}) many.objects.push_back({"red", x, y, 0.0});
  expect("eight cubes in the gray container validate and plan", validateScene(many).empty() ? checkPlan(many).empty() : false);
  PickPlaceConfig nogray = planConfig(s);
  nogray.has_gray_bin = false;
  expect("without a gray container an unmatched colour is an error", !planPickPlace(cubeTargets(s), nogray).ok);

  std::printf("[3] validation\n");
  auto bad = [&](const char* what, std::function<void(Scene&)> f, const char* msg) {
    Scene t = defaultScene();
    f(t);
    const std::string v = validateScene(t);
    expect(std::string(what) + " -> " + v, has(v, msg));
  };
  bad("unknown object colour", [](Scene& t) { t.objects[0].color = "pink"; }, "unknown colour");
  bad("gray object", [](Scene& t) { t.objects[0].color = "gray"; }, "gray cannot be an object");
  bad("duplicate container colour", [](Scene& t) { t.containers[1].color = "red"; }, "two 'red'");
  bad("object off the camera area", [](Scene& t) { t.objects[0].x = -0.9; }, "outside the camera area");
  bad("container off the table", [](Scene& t) { t.containers[0].x = -0.20; }, "not fully on the table");
  bad("containers overlap", [](Scene& t) { t.containers[1].y = -0.25; }, "overlap");
  bad("objects too close", [](Scene& t) { t.objects[1].x = -0.42; t.objects[1].y = -0.18; }, "too close");
  bad("object on a container", [](Scene& t) { t.objects[0].x = -0.32; t.objects[0].y = -0.28; }, "lies on the red container");
  bad("object on the gray container", [](Scene& t) { t.objects[0].x = -0.70; t.objects[0].y = 0.0; }, "lies on the gray container");
  bad("no objects", [](Scene& t) { t.objects.clear(); }, "no objects");
  bad("nine objects", [](Scene& t) { for (int i = 0; i < 5; ++i) t.objects.push_back({"red", -0.5, 0.0, 0}); }, "at most 8");
  const char* bad_yaml[][2] = {
      {"version: 2\nobjects: []\n", "'version' must be 1"},
      {"version: 1\nobject: []\n", "unknown key 'object'"},
      {"version: 1\nobjects: [{color: red, x: a, y: 0}]\n", "'x' must be a number"},
      {"version: 1\nobjects: [{color: red, x: -0.4}]\n", "missing 'y'"},
      {"version: 1\nobjects: [{color: red, x: -0.4, y: 0, size: 3}]\n", "unknown key 'size'"},
      {"version: 1\nobjects: 5\n", "'objects' must be a list"},
      {"version: 1\nobjects: []\ngray_container: {x: 1}\n", "missing 'y'"},
      {"- not a map\n", "must be a YAML map"},
      {"version: 1\nobjects: [\n", "not valid YAML"}};
  for (auto& b : bad_yaml) {
    Scene t;
    std::string e;
    const bool ok = sceneFromYaml(b[0], t, e);
    expect(std::string("YAML error: ") + e, !ok && has(e, b[1]));
  }

  std::printf("[4] edit commands\n");
  SceneModel e(2);
  int rev = e.scene().revision;
  auto r = e.execute("move_object 0 -0.50 0.0");
  expect("move_object accepted: " + r.message, r.ok && r.changed && e.scene().revision == rev + 1 && std::abs(e.scene().objects[0].x + 0.5) < 1e-9);
  r = e.execute("move_object 0 -0.50 0.60");
  expect("move_object off the table refused: " + r.message, !r.ok && std::abs(e.scene().objects[0].y - 0.0) < 1e-9);
  r = e.execute("move_object 0 -0.15 0.0");
  expect("move_object outside camera area refused: " + r.message, !r.ok);
  r = e.execute("move_object 9 0 0");
  expect("bad index refused: " + r.message, !r.ok && has(r.message, "there is no object 9"));
  r = e.execute("move_object 0 abc 0");
  expect("non-numeric refused", !r.ok);
  rev = e.scene().revision;
  r = e.execute("check_object 0 -0.55 0.02");
  expect("check_object does not change anything: " + r.message, r.ok && !r.changed && e.scene().revision == rev && std::abs(e.scene().objects[0].x + 0.5) < 1e-9);
  r = e.execute("check_object 0 -0.55 0.60");
  expect("check_object reports a bad position", !r.ok);
  r = e.execute("color_container 3 purple");
  expect("container recoloured: " + r.message, r.ok && e.scene().containers[3].color == "purple");
  expect("yellow object now goes to gray", destinations(e.scene())[3] == "gray");
  r = e.execute("color_container 0 green");
  expect("a second green container is refused: " + r.message, !r.ok && has(r.message, "two 'green'"));
  r = e.execute("color_object 1 pink");
  expect("unknown colour refused", !r.ok && e.scene().objects[1].color == "green");
  r = e.execute("color_object 1 orange");
  expect("object recoloured", r.ok && destinations(e.scene())[1] == "gray");
  r = e.execute("move_container 0 -0.30 -0.28");
  expect("move_container: " + r.message, r.ok || has(r.message, "overlap") || has(r.message, "lies on"));
  r = e.execute("move_gray -0.70 -0.30");
  expect("move_gray: " + r.message, r.ok);
  r = e.execute("add_object cyan -0.50 -0.28 0.3");
  expect("add_object: " + r.message, r.ok && e.scene().objects.size() == 5);
  r = e.execute("remove_object 4");
  expect("remove_object", r.ok && e.scene().objects.size() == 4);
  r = e.execute("remove_container 3");
  expect("remove_container", r.ok && e.scene().containers.size() == 3);
  r = e.execute("add_container yellow -0.30 0.30");
  expect("add_container", r.ok && e.scene().containers.size() == 4);
  r = e.execute("yaw_object 0 1.0");
  expect("yaw_object", r.ok && std::abs(e.scene().objects[0].yaw - 1.0) < 1e-9);
  r = e.execute("dance");
  expect("unknown command refused", !r.ok && has(r.message, "unknown command"));
  r = e.execute("save /tmp/scene_test_save.yaml");
  expect("save: " + r.message, r.ok);
  SceneModel e2(3);
  r = e2.execute("load /tmp/scene_test_save.yaml");
  expect("load restores it: " + r.message, r.ok && e2.scene().objects.size() == 4 && e2.scene().containers[2].color == e.scene().containers[2].color &&
                                                std::abs(e2.scene().gray_y - e.scene().gray_y) < 1e-9);
  r = e2.execute("load /tmp/does/not/exist.yaml");
  expect("load of a missing file leaves the scene alone", !r.ok && e2.scene().objects.size() == 4);
  {
    std::ofstream("/tmp/scene_test_bad.yaml") << "version: 1\nobjects: [{color: red, x: -0.4, y: -0.2}]\ncontainers: [{color: red, x: -0.4, y: -0.2}]\n";
    r = e2.execute("load /tmp/scene_test_bad.yaml");
    expect("loading an invalid scene is refused: " + r.message, !r.ok && e2.scene().objects.size() == 4);
  }
  setenv("HOME", "/tmp", 1);
  r = e2.execute("save ~/scene_home.yaml");
  expect("~ is expanded in paths", r.ok && std::ifstream("/tmp/scene_home.yaml").good());

  std::printf("[5] random placement\n");
  int fail = 0, gray = 0;
  double min_oo = 9, min_ob = 9;
  for (unsigned seed = 0; seed < 60; ++seed) {
    SceneModel rm(seed);
    for (const char* what : {"random all", "random objects", "random containers"}) {
      const SceneModel::Reply rr = rm.execute(what);
      if (!rr.ok) { ++fail; std::printf("      seed %u %s: %s\n", seed, what, rr.message.c_str()); continue; }
      const Scene& sc = rm.scene();
      if (!validateScene(sc).empty() || !checkPlan(sc).empty()) ++fail;
      for (size_t i = 0; i < sc.objects.size(); ++i)
        for (size_t j = i + 1; j < sc.objects.size(); ++j)
          min_oo = std::min(min_oo, std::hypot(sc.objects[i].x - sc.objects[j].x, sc.objects[i].y - sc.objects[j].y));
      for (const auto& o : sc.objects)
        for (const auto& b : allBins(sc)) min_ob = std::min(min_ob, std::max(std::abs(o.x - b.x), std::abs(o.y - b.y)));
    }
  }
  expect("180 random rearrangements (60 seeds x all/objects/containers) valid and plannable, " + std::to_string(fail) + " failed", fail == 0);
  expect("closest object pair " + std::to_string(min_oo).substr(0, 5) + " m, closest object to a container " + std::to_string(min_ob).substr(0, 5) + " m", min_oo >= kObjObjClear - 1e-9 && min_ob >= kObjBinClear - 1e-9);
  SceneModel a(5), b(5), c(6);
  a.execute("random all"); b.execute("random all"); c.execute("random all");
  expect("same seed gives the same arrangement, a different seed another", sceneToYaml(a.scene()) == sceneToYaml(b.scene()) && sceneToYaml(a.scene()) != sceneToYaml(c.scene()));
  (void)gray;

  std::printf("[6] state message\n");
  const std::string st = m.stateYaml("ready", true, "hello: world");
  YAML::Node n = YAML::Load(st);
  expect("state parses; has revision, seq, message", n["revision"].as<int>() == m.scene().revision && n["seq"].as<std::string>() == "ready" && n["message"].as<std::string>() == "hello: world");
  expect("state has 4 objects with a destination each", n["objects"].size() == 4 && n["objects"][0]["dest"].as<std::string>() == "red");
  expect("state has the palette and regions", n["palette"].size() == 7 && n["regions"]["object"]["xmax"].as<double>() == kObjXMax);
  const auto& rows = n["reach"]["rows"];
  int cells = 0, reach = 0;
  for (size_t i = 0; i < rows.size(); ++i)
    for (char ch : rows[i].as<std::string>()) { ++cells; reach += ch == '1'; }
  expect("reachability map is " + std::to_string(rows.size()) + " x " + std::to_string(cells / rows.size()) + ", " + std::to_string(reach) + " of " + std::to_string(cells) + " cells reachable", n["reach"]["nx"].as<int>() == 43 && n["reach"]["ny"].as<int>() == 65 && reach > cells / 2 && reach < cells);
  // the cells the demo scene uses must be reachable in the map
  bool demo_cells = true;
  for (const auto& o : m.scene().objects) {
    const double x0 = n["reach"]["x0"].as<double>(), y0 = n["reach"]["y0"].as<double>(), st = n["reach"]["step"].as<double>();
    const int i = static_cast<int>(std::round((x0 - o.x) / st)), j = static_cast<int>(std::round((o.y - y0) / st));
    demo_cells = demo_cells && rows[i].as<std::string>()[j] == '1';
  }
  expect("every default object lies in a reachable cell", demo_cells);
  {
    const double x0 = n["reach"]["x0"].as<double>(), y0 = n["reach"]["y0"].as<double>(), st = n["reach"]["step"].as<double>();
    auto cell = [&](double x, double y) { return rows[static_cast<int>(std::round((x0 - x) / st))].as<std::string>()[static_cast<int>(std::round((y - y0) / st))]; };
    expect("the map shows the reach limit: (-0.95, 0) and the base itself unreachable, (-0.50, 0) reachable",
           cell(-0.95, 0.0) == '0' && cell(0.0, 0.0) == '0' && cell(-0.50, 0.0) == '1');
  }
  if (argc > 2) { std::ofstream(argv[2]) << st; std::printf("      wrote %s\n", argv[2]); }

  std::printf("\n%d passed, %d failed\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
