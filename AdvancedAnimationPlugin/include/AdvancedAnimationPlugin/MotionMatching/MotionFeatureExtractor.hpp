#pragma once

#include "AdvancedAnimationPlugin/MotionMatching/MotionDatabase.hpp"

#include "Engine/Components/AnimatorComponent.hpp"
#include "Engine/Skeleton.hpp"

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace elix::plugin::advancedanim
{

    // Utility class for extracting MotionFeature vectors from animations or
    // the current runtime skeleton state.
    class MotionFeatureExtractor
    {
    public:
        // Extract a MotionFeature from an animation clip at a given time (seconds).
        // Evaluates SQT keyframes, fills pose + trajectory fields.
        // dt is used for finite-difference velocity estimation.
        static MotionFeature extractAt(const elix::engine::Animation &anim,
                                       elix::engine::Skeleton *skeleton,
                                       const BoneNameMap &boneNames,
                                       float timeSeconds,
                                       uint32_t clipIndex,
                                       uint32_t frameIndex,
                                       float dt = 1.f / 30.f);

        // Build a runtime query feature from:
        //  - currentSkeleton: the current animated skeleton pose (model-space bone transforms)
        //  - currentVelocity: the character's current actual velocity (world/model space)
        //  - desiredVelocity: target velocity from player input / AI
        //  - characterRotation: current character facing quaternion (to convert to local space)
        static MotionFeature buildQueryFeature(elix::engine::Skeleton *skeleton,
                                               const BoneNameMap &boneNames,
                                               const glm::vec3 &currentVelocity,
                                               const glm::vec3 &desiredVelocity,
                                               const glm::quat &characterRotation);

        // Evaluate all bone transforms for an animation at timeSeconds into skeleton's
        // finalTransformation fields.  Returns false if the animation has no keyframes.
        static bool evaluatePoseAt(const elix::engine::Animation &anim,
                                   elix::engine::Skeleton *skeleton,
                                   float timeTicks);

    private:
        static void calculateBoneTransformForPose(
            elix::engine::Skeleton::BoneInfo *boneInfo,
            const glm::mat4 &parentTransform,
            const elix::engine::Animation &anim,
            float timeTicks);

        static glm::vec3 bonePosition(elix::engine::Skeleton *skeleton, const std::string &boneName);
    };

} // namespace elix::plugin::advancedanim
