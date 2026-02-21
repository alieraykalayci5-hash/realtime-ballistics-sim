#pragma once

namespace sim {

struct Material {
    const char* name = "generic";
    double penetration_threshold_j = 200.0; // Joules
};

} // namespace sim