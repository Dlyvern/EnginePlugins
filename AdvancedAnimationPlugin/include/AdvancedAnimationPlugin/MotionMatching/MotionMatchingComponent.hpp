#pragma once

#include "AdvancedAnimationPlugin/MotionMatching/MotionDatabase.hpp"

#include "Engine/Components/ECS.hpp"
#include "Engine/Components/AnimatorComponent.hpp"
#include "Engine/Skeleton.hpp"

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace elix::engine
{
    class SkeletalMeshComponent;
    class Transform3DComponent;
}

namespace elix::plugin::advancedanim
{

    // MotionMatchingComponent — replaces AnimatorComponent as the skeleton driver.
    //
    // Attach to an entity with a SkeletalMeshComponent (but NOT an AnimatorComponent).
    // Supply a pre-baked MotionDatabase and a list of animation clips.
    // Each frame, call setDesiredVelocity() from a script, and the component
    // automatically picks the best-matching clip+frame and blends into it.
    class MotionMatchingComponent final : public elix::engine::ECS
    {
    public:
        bool isSkeletonDriver() const override { return true; }

        void update(float deltaTime) override;

        // Must be set before update() is called
        void setDatabase(std::shared_ptr<MotionDatabase> db);
        void setBoneNameMap(const BoneNameMap &map);
        void setAnimations(const std::vector<elix::engine::Animation> &animations);

        // Input from player/AI — set every frame
        void setDesiredVelocity(const glm::vec3 &vel) { m_desiredVelocity = vel; }

        // Tuning
        void setBlendDuration(float seconds) { m_blendDuration = seconds; }
        void setMinMatchInterval(float seconds) { m_minMatchInterval = seconds; }
        void setFeatureWeights(const MotionFeatureWeights &w) { m_weights = w; }

        // Post-pose hooks (for layering IK on top)
        using PostPoseHook = std::function<void(elix::engine::Skeleton &)>;
        void addPostPoseHook(const void *ownerKey, PostPoseHook fn);
        void removePostPoseHook(const void *ownerKey);

        // Read-back
        glm::vec3 getCurrentAnimatedVelocity() const { return m_currentAnimatedVelocity; }
        const MotionFeature *getCurrentMatchedFeature() const { return m_currentFeature; }

    protected:
        void onOwnerAttached() override;

    private:
        void applyPoseFromFeature(const MotionFeature &feature, float blendAlpha,
                                  const MotionFeature *blendFrom);

        void firePostPoseHooks(elix::engine::Skeleton &skeleton);

        elix::engine::SkeletalMeshComponent *m_skeletalMesh{nullptr};
        elix::engine::Transform3DComponent *m_transform{nullptr};

        std::shared_ptr<MotionDatabase> m_database;
        std::vector<elix::engine::Animation> m_animations;
        BoneNameMap m_boneNames;
        MotionFeatureWeights m_weights;

        glm::vec3 m_desiredVelocity{0.f};
        glm::vec3 m_currentAnimatedVelocity{0.f};

        // Playback state
        const MotionFeature *m_currentFeature{nullptr};
        const MotionFeature *m_previousFeature{nullptr};
        float m_currentTimeInClip{0.f};
        float m_blendAlpha{1.f};
        float m_blendDuration{0.15f};
        float m_timeSinceLastMatch{0.f};
        float m_minMatchInterval{0.083f}; // ~5 frames at 60 Hz

        struct PostPoseHookEntry
        {
            const void *key;
            PostPoseHook fn;
        };
        std::vector<PostPoseHookEntry> m_postPoseHooks;
    };

} // namespace elix::plugin::advancedanim
