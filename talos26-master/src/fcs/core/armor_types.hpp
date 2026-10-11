#pragma once

#include <cstdint>
#include <string_view>

#include <fmt/core.h>
#include <magic_enum.hpp>

namespace fcs {

// ============================================================================
// Armor Enums
// ============================================================================

enum class ArmorType : uint8_t {
    Small   = 0,
    Large   = 1,
    Invalid = 2,
};

enum class ArmorColor : uint8_t {
    Red     = 0,
    Blue    = 1,
    Neutral = 2,
    Purple  = 3,
};

enum class ArmorName : uint8_t {
    Sentry = 0,
    One,
    Two,
    Three,
    Four,
    Five,
    Outpost,
    Base,
    BaseLarge,
    Invalid
};

// ============================================================================
// AT Legacy cls/color Mapping (from NN output)
// ============================================================================

/// Determine ArmorType from armor name
/// Hero (One) and BaseLarge are Large armor types
[[nodiscard]] constexpr ArmorType cls_to_armor_type(ArmorName armor_name) noexcept {
    return (armor_name == ArmorName::One || armor_name == ArmorName::BaseLarge) ? ArmorType::Large
                                                                                : ArmorType::Small;
}

} // namespace fcs

// ============================================================================
// fmt::formatter specializations
// ============================================================================

namespace fmt {

template <>
struct formatter<fcs::ArmorType> : formatter<std::string_view> {
    auto format(const fcs::ArmorType c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

template <>
struct formatter<fcs::ArmorColor> : formatter<std::string_view> {
    auto format(const fcs::ArmorColor c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

template <>
struct formatter<fcs::ArmorName> : formatter<std::string_view> {
    auto format(const fcs::ArmorName c, format_context& ctx) const {
        return formatter<std::string_view>::format(magic_enum::enum_name(c), ctx);
    }
};

} // namespace fmt
