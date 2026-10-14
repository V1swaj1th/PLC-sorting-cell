#pragma once
#include <cstdlib>
#include <string>

namespace sorting_cell {

// "~" or "~/..." -> $HOME or $HOME/...; anything else is returned unchanged.
inline std::string expandUser(const std::string& p) {
  if (!p.empty() && p[0] == '~' && (p.size() == 1 || p[1] == '/')) {
    const char* h = std::getenv("HOME");
    if (h) return std::string(h) + p.substr(1);
  }
  return p;
}

}  // namespace sorting_cell
