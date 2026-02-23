#include "algorithm/union_find.h"

#include <algorithm>
#include <numeric>

namespace vats5 {

UnionFind::UnionFind(int n) : parent(n), rank(n, 0) {
  std::iota(parent.begin(), parent.end(), 0);
}

int UnionFind::Find(int x) {
  if (parent[x] != x) parent[x] = Find(parent[x]);
  return parent[x];
}

bool UnionFind::Unite(int x, int y) {
  int px = Find(x), py = Find(y);
  if (px == py) return false;
  if (rank[px] < rank[py]) std::swap(px, py);
  parent[py] = px;
  if (rank[px] == rank[py]) rank[px]++;
  return true;
}

}  // namespace vats5
