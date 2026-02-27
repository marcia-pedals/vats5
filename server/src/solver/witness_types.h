#pragma once

#include <concepts>
#include <type_traits>

namespace vats5 {

// Sentinel tag for un-witnessed containers and StopIds.
// When a container's tag is Unwitnessed, all StopId-based access methods
// added by the witness layer are disabled at compile time.
struct Unwitnessed {};

// Tag for an extension space that carries its parent tag.
// Extensions can be nested: the parent tag itself can be an ExtendedTag.
template <typename ParentTag, typename LocalTag>
struct ExtendedTag {
  using parent_tag = ParentTag;
};

// Helper trait: CompatibleWithImpl<S, T> is true_type if S == T, or
// T has a parent_tag and S is compatible with that parent.
// Concepts can't be recursive, so we use a struct template with specialization.
template <typename S, typename T, typename = void>
struct CompatibleWithImpl : std::false_type {};

// Base case: S == T
template <typename T>
struct CompatibleWithImpl<T, T, void> : std::true_type {};

// Recursive case: T has parent_tag and S is compatible with parent_tag.
// Only enabled when S != T (to avoid ambiguity with the base case).
template <typename S, typename T>
struct CompatibleWithImpl<
    S, T,
    std::enable_if_t<
        !std::is_same_v<S, T> &&
        requires { typename T::parent_tag; } &&
        CompatibleWithImpl<S, typename T::parent_tag>::value
    >
> : std::true_type {};

// A StopId<S> can be used in a container with tag T if:
// - S == T (exact match), or
// - T is an ExtendedTag<P, _> and S is compatible with P
//   (i.e., S is an ancestor of T in the extension chain)
template <typename S, typename T>
concept CompatibleWith = CompatibleWithImpl<S, T>::value;

}  // namespace vats5
