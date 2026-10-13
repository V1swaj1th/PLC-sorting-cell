#include <cstdio>

#include "sorting_cell_controller/scene.hpp"

using namespace sorting_cell;

// Checks a scene YAML file with the same code the scene server uses: format, geometry, colour
// matching and a full plan with the own IK. Exit code 0 when the scene is usable.
int main(int argc, char** argv) {
  if (argc != 2) {
    std::printf("usage: scene_check SCENE.yaml\n");
    return 2;
  }
  SceneModel m(0);
  const auto r = m.load(argv[1]);
  if (!r.ok) {
    std::printf("REJECTED: %s\n", r.message.c_str());
    return 1;
  }
  const Scene& s = m.scene();
  const auto dest = destinations(s);
  std::printf("OK: '%s', %zu objects, %zu coloured containers + gray container at (%.2f, %.2f)\n", s.name.c_str(),
              s.objects.size(), s.containers.size(), s.gray_x, s.gray_y);
  int gray = 0;
  for (size_t i = 0; i < s.objects.size(); ++i) {
    std::printf("  object %zu: %-7s at (%6.3f, %6.3f)  ->  %s container\n", i, s.objects[i].color.c_str(), s.objects[i].x,
                s.objects[i].y, dest[i].c_str());
    gray += dest[i] == kGrayName;
  }
  std::printf("%d of %zu objects have no container of their colour and go to the gray one.\n", gray, s.objects.size());
  std::printf("Planned with the own IK: %.1f s for the whole cycle.\n", m.planSeconds());
  return 0;
}
