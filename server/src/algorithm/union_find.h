#pragma once

#include <vector>

namespace vats5 {

struct UnionFind {
  std::vector<int> parent, rank;

  UnionFind(int n);
  int Find(int x);
  bool Unite(int x, int y);
};

}  // namespace vats5
