#include "AdvancedAnimationPlugin/IK/IKComponent.hpp"
#include "AdvancedAnimationPlugin/IK/FABRIKSolver.hpp"

#include "Engine/Components/AnimatorComponent.hpp"
#include "Engine/Components/Transform3DComponent.hpp"
#include "Engine/Entity.hpp"
#include "Engine/Scene.hpp"
#include "Engine/Scripting/VelixAPI.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>

namespace elix::plugin::advancedanim
{

    void IKComponent::onOwnerAttached()
    {
        registerHook();
    }

    void IKComponent::onDetach()
    {
        unregisterHook();
    }

    void IKComponent::registerHook()
    {
        if (m_hookRegistered)
            return;
        auto *entity = getOwner<elix::engine::Entity>();
        if (!entity)
            return;
        auto *animator = entity->getComponent<elix::engine::AnimatorComponent>();
        if (!animator)
            return;
        animator->addPostAnimHook(this, [this](elix::engine::Skeleton &skel)
                                  { solveAllChains(skel); });
        m_hookRegistered = true;
    }

    void IKComponent::unregisterHook()
    {
        if (!m_hookRegistered)
            return;
        auto *entity = getOwner<elix::engine::Entity>();
        if (!entity)
            return;
        auto *animator = entity->getComponent<elix::engine::AnimatorComponent>();
        if (animator)
            animator->removePostAnimHook(this);
        m_hookRegistered = false;
    }

    void IKComponent::addChain(const IKChain &chain)
    {
        m_chains.push_back(chain);
    }

    void IKComponent::removeChain(const std::string &name)
    {
        m_chains.erase(
            std::remove_if(m_chains.begin(), m_chains.end(),
                           [&name](const IKChain &c)
                           { return c.name == name; }),
            m_chains.end());
    }

    IKChain *IKComponent::getChain(const std::string &name)
    {
        auto it = std::find_if(m_chains.begin(), m_chains.end(),
                               [&name](const IKChain &c)
                               { return c.name == name; });
        return it != m_chains.end() ? &(*it) : nullptr;
    }

    glm::vec3 IKComponent::resolveTarget(const IKChain &chain) const
    {
        if (chain.useEntityTarget)
        {
            auto *scene = elix::engine::scripting::getActiveScene();
            if (scene)
            {
                auto *e = scene->getEntityById(chain.targetEntityId);
                if (e)
                {
                    if (auto *t = e->getComponent<elix::engine::Transform3DComponent>())
                        return t->getWorldPosition();
                }
            }
        }
        return chain.fixedTargetWorld;
    }

    glm::vec3 IKComponent::resolvePole(const IKChain &chain) const
    {
        if (chain.useEntityPole)
        {
            auto *scene = elix::engine::scripting::getActiveScene();
            if (scene)
            {
                auto *e = scene->getEntityById(chain.poleEntityId);
                if (e)
                {
                    if (auto *t = e->getComponent<elix::engine::Transform3DComponent>())
                        return t->getWorldPosition();
                }
            }
        }
        return chain.fixedPoleWorld;
    }

    void IKComponent::propagateChildTransforms(elix::engine::Skeleton &skeleton,
                                               elix::engine::Skeleton::BoneInfo *bone,
                                               const glm::mat4 &delta,
                                               const std::vector<std::string> &chainBones)
    {
        for (auto *child : bone->childrenInfo)
        {
            // Skip bones that are part of the IK chain — they're solved explicitly
            bool inChain = std::find(chainBones.begin(), chainBones.end(), child->name) != chainBones.end();
            if (inChain)
                continue;

            child->finalTransformation = delta * child->finalTransformation;
            propagateChildTransforms(skeleton, child, delta, chainBones);
        }
    }

    void IKComponent::solveAllChains(elix::engine::Skeleton &skeleton)
    {
        auto *entity = getOwner<elix::engine::Entity>();
        if (!entity)
            return;

        auto *transform = entity->getComponent<elix::engine::Transform3DComponent>();
        const glm::mat4 entityWorld = transform ? transform->getMatrix() : glm::mat4(1.f);
        const glm::mat4 worldToModel = glm::inverse(entityWorld);

        for (auto &chain : m_chains)
        {
            if (chain.weight <= 0.f || chain.boneNames.size() < 2)
                continue;

            // Gather bone pointers
            std::vector<elix::engine::Skeleton::BoneInfo *> bones;
            bones.reserve(chain.boneNames.size());
            for (const auto &name : chain.boneNames)
            {
                auto *b = skeleton.getBone(name);
                if (!b)
                {
                    bones.clear();
                    break;
                }
                bones.push_back(b);
            }
            if (bones.size() < 2)
                continue;

            // Extract current model-space joint positions from BoneInfo::finalTransformation
            std::vector<glm::vec3> positions;
            positions.reserve(bones.size());
            for (auto *b : bones)
                positions.emplace_back(b->finalTransformation[3]);

            // Compute segment lengths
            std::vector<float> boneLengths;
            boneLengths.reserve(bones.size() - 1);
            for (size_t i = 0; i + 1 < bones.size(); ++i)
            {
                float len = glm::length(positions[i + 1] - positions[i]);
                boneLengths.push_back(len < 1e-6f ? 1e-6f : len);
            }

            // Convert world-space target to model space
            const glm::vec3 targetWorld = resolveTarget(chain);
            const glm::vec3 targetModel = glm::vec3(worldToModel * glm::vec4(targetWorld, 1.f));

            // FABRIK solve
            const std::vector<glm::vec3> preSolvePositions = positions;
            FABRIKSolver::Result result = FABRIKSolver::solve(
                positions, targetModel, boneLengths, chain.maxIterations, chain.tolerance);

            // Pole constraint for 2-bone chains
            if (chain.hasPoleTarget && result.positions.size() == 3)
            {
                const glm::vec3 poleWorld = resolvePole(chain);
                const glm::vec3 poleModel = glm::vec3(worldToModel * glm::vec4(poleWorld, 1.f));
                FABRIKSolver::applyPoleConstraint(result.positions, poleModel);
            }

            // Write solved positions back to bone transforms
            for (size_t i = 0; i + 1 < bones.size(); ++i)
            {
                auto *bone = bones[i];
                const glm::mat4 oldFinal = bone->finalTransformation;

                const glm::vec3 oldDir = preSolvePositions[i + 1] - preSolvePositions[i];
                const glm::vec3 newDir = result.positions[i + 1] - result.positions[i];

                const float oldLen = glm::length(oldDir);
                const float newLen = glm::length(newDir);

                glm::mat4 newFinal = oldFinal;

                if (oldLen > 1e-6f && newLen > 1e-6f)
                {
                    glm::quat q = glm::rotation(oldDir / oldLen, newDir / newLen);

                    // Blend with chain weight
                    if (chain.weight < 1.f)
                        q = glm::slerp(glm::quat(1.f, 0.f, 0.f, 0.f), q, chain.weight);

                    // Apply model-space rotation delta to the bone matrix
                    newFinal = glm::mat4_cast(q) * oldFinal;
                }

                // Blend the position
                const glm::vec3 blendedPos = glm::mix(preSolvePositions[i], result.positions[i], chain.weight);
                newFinal[3] = glm::vec4(blendedPos, 1.f);

                bone->finalTransformation = newFinal;

                // Propagate delta to non-IK-chain children
                const glm::mat4 delta = newFinal * glm::inverse(oldFinal);
                propagateChildTransforms(skeleton, bone, delta, chain.boneNames);
            }

            // Set effector bone position
            {
                auto *effector = bones.back();
                const glm::vec3 blendedPos = glm::mix(preSolvePositions.back(), result.positions.back(), chain.weight);
                effector->finalTransformation[3] = glm::vec4(blendedPos, 1.f);
            }
        }
    }

} // namespace elix::plugin::advancedanim
