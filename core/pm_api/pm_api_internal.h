// Internal to the C API implementation (not part of the stable surface):
// the engine behind a pm_engine handle, for the host layer.
#pragma once

#include "pm_api/pm_api.h"
#include "pm_engine/engine.h"

pm::Engine* pmEngineOf(pm_engine* engine) noexcept;
