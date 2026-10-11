#pragma once

#include "../world.hpp"
#include "components.hpp"
#include "execution_policy.hpp"
#include "system_meta.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace talos::scheduler::system {

// ============================================================================
// System Base Interface
// ============================================================================

// Forward declaration — needed by SystemBase::as_external_compute()
class ExternalComputeSource;

class SystemBase {
public:
    virtual ~SystemBase() noexcept = default;
    // bind() is the only phase where channel handles may be acquired from World.
    virtual void bind(World& world) noexcept = 0;
    // run() returns true only if a declared output channel was written and must not dynamically
    // bind channels or mutate scheduler topology.
    virtual bool run(World& world) noexcept         = 0;
    virtual const SystemMeta& meta() const noexcept = 0;

    /// Returns this system as an ExternalComputeSource, or nullptr.
    /// Override in systems that implement ExternalComputeSource.
    /// Replaces dynamic_cast for RTTI-free dispatch.
    virtual ExternalComputeSource* as_external_compute() noexcept { return nullptr; }
};

/**
 * @brief Marker for compute systems that can be woken by external threads
 *
 * Systems implementing this interface act as build-time liveness sources.
 * The scheduler also binds them to the shared ready bitmask after build().
 *
 * ## Contract
 *
 * - Only compute systems may implement this interface
 * - External wakeups may only set the system's own ready bit
 * - Channel access still has to be acquired during bind(), not lazily in run()
 */
class ExternalComputeSource {
public:
    virtual ~ExternalComputeSource() noexcept = default;

    virtual void bind_external_ready_slot(
        std::atomic<std::uint64_t>* ready_systems, std::size_t system_index) noexcept = 0;
};

// ============================================================================
// Argument Construction
// ============================================================================

namespace detail {

// Compile-time counter: count how many local<T> types appear before a given index
template <typename ArgsTuple, std::size_t TargetIdx, std::size_t CurrentIdx = 0>
struct count_locals_before;

// Base case: when CurrentIdx reaches TargetIdx, stop counting
template <typename... Args, std::size_t TargetIdx>
struct count_locals_before<std::tuple<Args...>, TargetIdx, TargetIdx> {
    static constexpr std::size_t value = 0;
};

// Recursive case: check if current argument is local, then continue
template <typename... Args, std::size_t TargetIdx, std::size_t CurrentIdx>
requires(CurrentIdx < TargetIdx)
struct count_locals_before<std::tuple<Args...>, TargetIdx, CurrentIdx> {
    using CurrentArg                       = std::tuple_element_t<CurrentIdx, std::tuple<Args...>>;
    static constexpr bool is_current_local = is_local_type<CurrentArg>::value;
    static constexpr std::size_t value =
        (is_current_local ? 1 : 0)
        + count_locals_before<std::tuple<Args...>, TargetIdx, CurrentIdx + 1>::value;
};

// Extract types of all local<T> arguments
template <typename Tuple>
struct extract_local_types;

// Helper: wrap type in tuple if local, otherwise empty tuple
template <typename T, bool is_local>
struct local_to_tuple_impl;

template <typename T>
struct local_to_tuple_impl<T, true> {
    using type = std::tuple<inner_type_t<T>>;
};

template <typename T>
struct local_to_tuple_impl<T, false> {
    using type = std::tuple<>;
};

template <typename T>
using local_to_tuple_t = local_to_tuple_impl<T, is_local_type<T>::value>::type;

template <typename... Ts>
struct extract_local_types<std::tuple<Ts...>> {
    using type = decltype(std::tuple_cat(std::declval<local_to_tuple_t<Ts>>()...));
};

template <typename Tuple>
using extract_local_types_t = extract_local_types<Tuple>::type;

// Overload for local<T> - uses compile-time index
template <typename T, typename LocalStorage, std::size_t LocalIdx>
auto make_arg(
    World& world, LocalStorage& local_storage,
    std::integral_constant<std::size_t, LocalIdx>) noexcept {
    if constexpr (is_local_type<T>::value) {
        using Inner   = inner_type_t<T>;
        auto& storage = std::get<LocalIdx>(local_storage);
        return local<Inner>{&storage};
    } else {
        return world.get<T>();
    }
}

// Helper to build args_tuple with guaranteed left-to-right evaluation
// In C++17, braced initializer list elements are evaluated left-to-right (sequenced)
template <typename ArgsTuple, typename LocalStorage, std::size_t... Is>
auto make_args_sequenced(
    World& world, LocalStorage& local_storage, std::index_sequence<Is...>) noexcept {
    return ArgsTuple{make_arg<std::tuple_element_t<Is, ArgsTuple>>(
        world, local_storage,
        std::integral_constant<std::size_t, count_locals_before<ArgsTuple, Is>::value>{})...};
}

} // namespace detail

// ============================================================================
// Concrete System Implementation
// ============================================================================

template <typename F, typename Policy = default_policy>
class System : public SystemBase {
    using traits     = detail::function_traits<F>;
    using args_tuple = traits::args_tuple;

    F func_;
    SystemMeta meta_;
    std::optional<args_tuple> cached_args_;
    bool written_ = false;
    detail::extract_local_types_t<args_tuple> local_storage_{};

    // Recursively set written_flag_ for all writer arguments
    template <std::size_t I = 0>
    void bind_written_flags() noexcept {
        if constexpr (I < std::tuple_size_v<args_tuple>) {
            using ArgType = std::tuple_element_t<I, args_tuple>;
            if constexpr (detail::is_writer<ArgType>::value) {
                std::get<I>(*cached_args_).written_flag_ = &written_;
            }
            bind_written_flags<I + 1>();
        }
    }

public:
    System(std::string name, F func) noexcept
        : func_(std::move(func))
        , meta_(extract_system_meta<F, Policy>(std::move(name))) {}

    /// Pre-create all channel readers/writers (must be called single-threaded before run)
    void bind(World& world) noexcept override {
        if (cached_args_) {
            return; // Already bound
        }
        cached_args_ = detail::make_args_sequenced<args_tuple>(
            world, local_storage_, std::make_index_sequence<traits::arity>{});
        bind_written_flags();
    }

    bool run([[maybe_unused]] World& world) noexcept override {
        written_ = false;
        std::apply(func_, *cached_args_);
        return written_;
    }

    const SystemMeta& meta() const noexcept override { return meta_; }
};

template <typename F, typename Policy = default_policy>
[[nodiscard]] std::unique_ptr<SystemBase> make_system(std::string name, F&& func) noexcept {
    return std::make_unique<System<std::decay_t<F>, Policy>>(
        std::move(name), std::forward<F>(func));
}

} // namespace talos::scheduler::system
