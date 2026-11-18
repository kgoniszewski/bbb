// Parameter descriptor table (plan §5.3): the single source of truth for the
// pm_param ids. Static data only; lookups are allocation-free and real-time
// safe.
#pragma once

#include <cstdint>

#include "pm_api/pm_api.h"

namespace pm {

// The descriptor of a parameter id, or nullptr for an unknown id.
const pm_param_info_t* paramDescriptor(std::int32_t id) noexcept;

// Validates a value for set_param against the descriptor: returns false for
// NaN, or out of range when the parameter rejects such values; otherwise
// clamps *value to [min, max] and returns true. Values in range are returned
// unchanged (bit for bit).
bool paramAccept(const pm_param_info_t& d, double* value) noexcept;

}  // namespace pm
