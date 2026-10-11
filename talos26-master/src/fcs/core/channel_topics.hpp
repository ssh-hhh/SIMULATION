#pragma once

namespace fcs {

// ============================================================================
// AT Legacy Armor Detection Pipeline Channel Topics
// ============================================================================

/// @brief Image frame channel (Camera → Detector)
/// Data type: ImageFrame
/// Mode: SPMC (single producer, multiple consumers)
struct ImageChannelTopic {};

/// @brief Detection result channel (Detector → PnPSolver)
/// Data type: ArmorDetectionBatch
/// Mode: SPMC (single producer, multiple consumers)
struct DetectionChannelTopic {};

/// @brief Measurement result channel (PnPSolver → Tracker)
/// Data type: ArmorMeasurementBatch
/// Mode: SPMC (single producer, multiple consumers)
struct MeasurementChannelTopic {};

struct LdmDetectionChannelTopic {};
struct LdmMeasurementChannelTopic {};

// ============================================================================
// L3 Estimation Pipeline Channel Topics
// ============================================================================

/// @brief Tracker output channel (Tracker → Solver)
/// Data type: fcs::L3::TrackerOutputs
/// Mode: SPMC (single producer, multiple consumers)
struct TrackerOutputChannelTopic {};

struct RuneObservationChannelTopic {};
struct RuneDebugFrameChannelTopic {};

struct EnergyMeterStateChannelTopic {};

// ============================================================================
// L4 Planning Pipeline Channel Topics
// ============================================================================

/// @brief L4 → L5 control intent channel (L4 Planner → L5 Weapon System)
/// Data type: fcs::L4::ControlIntent (std::variant<TrackCommand, ShotCommand, HoldCommand>)
/// Mode: SPMC (single producer, multiple consumers)
struct ControlIntentChannelTopic {};

/// @brief L4-selected target snapshot channel (Aimer -> Viz/Chiral/Capturer)
/// Data type: fcs::L4::SelectedTargetSnapshot
/// Mode: SPMC (single producer, multiple consumers)
struct SelectedTargetSnapshotChannelTopic {};

/// @brief Target-selection diagnostics channel (Aimer -> Viz/Capturer)
/// Data type: fcs::L4::TargetSelectionTrace
/// Mode: SPMC (single producer, multiple consumers)
struct TargetSelectionTraceChannelTopic {};

// ============================================================================
// L5 Weapon System Channel Topics
// ============================================================================

/// @brief Weapon command channel (Fire Control → Output Interface)
/// Data type: fcs::L5::WeaponCommand
/// Mode: SPMC (single producer, multiple consumers)
struct WeaponCommandChannelTopic {};

/// @brief Runtime control-resource snapshot channel (IMU/resource owner -> capturer)
/// Data type: fcs::core::ControlResourceSnapshot
/// Mode: SPMC (single producer, multiple consumers)
struct RuntimeControlStateChannelTopic {};

// ============================================================================
// Ground Truth (Daedalus IPC → Visualization)
// ============================================================================

/// @brief Ground truth batch channel (Daedalus IPC reader → Visualization)
/// Data type: ipc::GroundTruthBatch
/// Mode: SPMC (single producer, multiple consumers)
struct GroundTruthBatchChannelTopic {};

} // namespace fcs
