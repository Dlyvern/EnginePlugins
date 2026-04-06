#pragma once

#include "Engine/Components/ECS.hpp"
#include "Engine/Skeleton.hpp"

#include <functional>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace elix::plugin::advancedanim
{

    struct IKChain
    {
        std::string name;                   // unique name for this chain
        std::vector<std::string> boneNames; // root → effector (both inclusive)

        // Target — either a scene entity or a fixed world position
        bool useEntityTarget{false};
        uint32_t targetEntityId{0};
        glm::vec3 fixedTargetWorld{0.f};

        // Optional pole vector for 2-bone IK (knee/elbow hint)
        bool hasPoleTarget{false};
        bool useEntityPole{false};
        uint32_t poleEntityId{0};
        glm::vec3 fixedPoleWorld{0.f};

        float weight{1.0f}; // blend factor [0,1]; 0 = no IK, 1 = full IK
        uint32_t maxIterations{10};
        float tolerance{0.01f}; // convergence threshold in metres
    };

    class IKComponent final : public elix::engine::ECS
    {
    public:
        void onDetach() override;

        void addChain(const IKChain &chain);
        void removeChain(const std::string &name);
        IKChain *getChain(const std::string &name);
        const std::vector<IKChain> &getChains() const { return m_chains; }

        void registerHook();
        void unregisterHook();

    protected:
        void onOwnerAttached() override;

    private:
        void solveAllChains(elix::engine::Skeleton &skeleton);
        glm::vec3 resolveTarget(const IKChain &chain) const;
        glm::vec3 resolvePole(const IKChain &chain) const;

        void propagateChildTransforms(elix::engine::Skeleton &skeleton,
                                      elix::engine::Skeleton::BoneInfo *bone,
                                      const glm::mat4 &rotDeltaMat,
                                      const std::vector<std::string> &chainBones);

        std::vector<IKChain> m_chains;
        bool m_hookRegistered{false};
    };

} // namespace elix::plugin::advancedanim
