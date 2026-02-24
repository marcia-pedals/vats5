#pragma once

#include <cassert>
#include <new>
#include <string>
#include <vector>

#include "solver/data.h"
#include "solver/steps_adjacency_list.h"
#include "solver/witness_types.h"

namespace vats5 {

// Runtime layer: tracks the full parent-child lineage between spaces.
// Checked once at witness-creation time.
struct SpaceRegistry {
  struct SpaceInfo {
    int parent_id;  // -1 for root spaces
    int num_stops;
    std::string debug_name;
  };
  std::vector<SpaceInfo> spaces;

  int NewRoot(int num_stops, std::string name = "") {
    int id = static_cast<int>(spaces.size());
    spaces.push_back({-1, num_stops, std::move(name)});
    return id;
  }

  int NewExtension(int parent_id, int num_stops, std::string name = "") {
    assert(num_stops >= spaces[parent_id].num_stops);
    int id = static_cast<int>(spaces.size());
    spaces.push_back({parent_id, num_stops, std::move(name)});
    return id;
  }

  int NewRemap(int num_stops, std::string name = "") {
    return NewRoot(num_stops, std::move(name));
  }

  // Walk the parent chain of target_space to check if stop_space is an
  // ancestor (or is target_space itself). O(depth of extension chain).
  bool IsCompatible(int stop_space, int target_space) const {
    int cur = target_space;
    while (cur != -1) {
      if (cur == stop_space) return true;
      cur = spaces[cur].parent_id;
    }
    return false;
  }
};

// Zero-cost witness cast. Since the templated subclass adds no fields,
// Container<Unwitnessed> and Container<NewTag> have identical layout.
template <typename NewTag, template <typename> typename Container>
const Container<NewTag>& WitnessAs(const Container<Unwitnessed>& c) {
  static_assert(
      sizeof(Container<NewTag>) == sizeof(Container<Unwitnessed>));
  static_assert(
      alignof(Container<NewTag>) == alignof(Container<Unwitnessed>));
  return *std::launder(
      reinterpret_cast<const Container<NewTag>*>(&c));
}

// Simple witness: tags an unwitnessed container.
template <typename Tag, template <typename> typename Container>
const Container<Tag>& Witness(Tag, const Container<Unwitnessed>& c) {
  return WitnessAs<Tag>(c);
}

// Extension witness result: provides both parent and child views.
template <typename PTag, typename CLocalTag,
          template <typename> typename Container>
struct WitnessedExtension {
  using ChildTag = ExtendedTag<PTag, CLocalTag>;
  const Container<PTag>& parent;
  const Container<ChildTag>& child;
};

// Extension witness factory: verifies lineage at runtime and returns
// both parent and child views.
template <typename PTag, typename CLocalTag,
          template <typename> typename Container>
WitnessedExtension<PTag, CLocalTag, Container> WitnessExtension(
    PTag, CLocalTag, const Container<Unwitnessed>& parent,
    const Container<Unwitnessed>& child, const SpaceRegistry& reg) {
  assert(reg.IsCompatible(parent.space_id, child.space_id));
  using ChildTag = ExtendedTag<PTag, CLocalTag>;
  return {WitnessAs<PTag>(parent), WitnessAs<ChildTag>(child)};
}

// Remap witness result: provides old/new views and bidirectional mapping.
template <typename OldTag, typename NewTag,
          template <typename> typename Container>
struct WitnessedRemap {
  const Container<OldTag>& old_space;
  const Container<NewTag>& new_space;
  const StopIdMapping& mapping;

  StopId<NewTag> ToNew(StopId<OldTag> old_id) const {
    return StopId<NewTag>{mapping.original_to_new[old_id.v].v};
  }
  StopId<OldTag> ToOld(StopId<NewTag> new_id) const {
    return StopId<OldTag>{mapping.new_to_original[new_id.v].v};
  }
};

// Remap witness factory: tags old and new containers with distinct tags.
template <typename OldTag, typename NewTag,
          template <typename> typename Container>
WitnessedRemap<OldTag, NewTag, Container> WitnessRemap(
    OldTag, NewTag, const Container<Unwitnessed>& old_container,
    const Container<Unwitnessed>& new_container,
    const StopIdMapping& mapping) {
  return {WitnessAs<OldTag>(old_container),
          WitnessAs<NewTag>(new_container), mapping};
}

}  // namespace vats5
