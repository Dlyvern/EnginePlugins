#include "AdvancedAnimationPlugin/IK/FABRIKSolver.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

namespace elix::plugin::advancedanim
{

    FABRIKSolver::Result FABRIKSolver::solve(const std::vector<glm::vec3> &inPositions,
                                             const glm::vec3 &target,
                                             const std::vector<float> &boneLengths,
                                             uint32_t maxIterations,
                                             float tolerance)
    {
        const size_t n = inPositions.size();
        Result result;
        result.positions = inPositions;

        if (n < 2 || boneLengths.size() != n - 1)
            return result;

        // Total chain length
        float totalLength = 0.f;
        for (float len : boneLengths)
            totalLength += len;

        const glm::vec3 root = inPositions[0];
        const float distToTarget = glm::length(target - root);

        // If target is unreachable, stretch the chain toward it
        if (distToTarget >= totalLength)
        {
            glm::vec3 dir = glm::normalize(target - root);
            for (size_t i = 1; i < n; ++i)
                result.positions[i] = result.positions[i - 1] + dir * boneLengths[i - 1];
            result.converged = false;
            return result;
        }

        for (uint32_t iter = 0; iter < maxIterations; ++iter)
        {
            // Forward pass — pull effector to target
            result.positions[n - 1] = target;
            for (int i = static_cast<int>(n) - 2; i >= 0; --i)
            {
                glm::vec3 dir = glm::normalize(result.positions[i] - result.positions[i + 1]);
                result.positions[i] = result.positions[i + 1] + dir * boneLengths[i];
            }

            // Backward pass — root back in place
            result.positions[0] = root;
            for (size_t i = 1; i < n; ++i)
            {
                glm::vec3 dir = glm::normalize(result.positions[i] - result.positions[i - 1]);
                result.positions[i] = result.positions[i - 1] + dir * boneLengths[i - 1];
            }

            if (glm::length2(result.positions[n - 1] - target) <= tolerance * tolerance)
            {
                result.converged = true;
                break;
            }
        }

        return result;
    }

    void FABRIKSolver::applyPoleConstraint(std::vector<glm::vec3> &positions,
                                           const glm::vec3 &poleTarget)
    {
        if (positions.size() != 3)
            return;

        const glm::vec3 &root = positions[0];
        const glm::vec3 &mid = positions[1];
        const glm::vec3 &tip = positions[2];

        // Project mid-joint and pole onto the plane perpendicular to root→tip
        glm::vec3 axis = glm::normalize(tip - root);

        auto projectOntoPlane = [&](const glm::vec3 &p, const glm::vec3 &origin)
        {
            glm::vec3 v = p - origin;
            return v - glm::dot(v, axis) * axis;
        };

        glm::vec3 midProj = projectOntoPlane(mid, root);
        glm::vec3 poleProj = projectOntoPlane(poleTarget, root);

        const float midLen = glm::length(midProj);
        const float poleLen = glm::length(poleProj);
        if (midLen < 1e-6f || poleLen < 1e-6f)
            return;

        midProj /= midLen;
        poleProj /= poleLen;

        float cosA = glm::clamp(glm::dot(midProj, poleProj), -1.f, 1.f);
        float angle = std::acos(cosA);
        if (glm::abs(angle) < 1e-6f)
            return;

        // Determine rotation direction
        glm::vec3 cross = glm::cross(midProj, poleProj);
        if (glm::dot(cross, axis) < 0.f)
            angle = -angle;

        // Rotate mid-joint about axis by angle
        glm::quat rot = glm::angleAxis(angle, axis);
        glm::vec3 midLocal = mid - root;
        positions[1] = root + glm::vec3(rot * glm::vec4(midLocal, 0.f));
    }

} // namespace elix::plugin::advancedanim
