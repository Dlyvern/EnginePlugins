#pragma once

#include <glm/vec3.hpp>
#include <vector>

namespace elix::plugin::advancedanim
{

    class FABRIKSolver
    {
    public:
        struct Result
        {
            std::vector<glm::vec3> positions; // solved joint world positions (root → effector)
            bool converged{false};
        };

        static Result solve(const std::vector<glm::vec3> &inPositions,
                            const glm::vec3 &target,
                            const std::vector<float> &boneLengths,
                            uint32_t maxIterations = 10,
                            float tolerance = 0.01f);

        static void applyPoleConstraint(std::vector<glm::vec3> &positions,
                                        const glm::vec3 &poleTarget);
    };

} // namespace elix::plugin::advancedanim
