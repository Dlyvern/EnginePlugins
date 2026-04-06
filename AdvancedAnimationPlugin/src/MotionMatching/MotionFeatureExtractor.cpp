#include "AdvancedAnimationPlugin/MotionMatching/MotionFeatureExtractor.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace elix::plugin::advancedanim
{

    static std::pair<const elix::engine::SQT *, const elix::engine::SQT *>
    findKeyframes(const std::vector<elix::engine::SQT> &kf, float t)
    {
        if (kf.empty())
            return {nullptr, nullptr};
        if (t <= kf.front().timeStamp)
            return {&kf.front(), &kf.front()};
        if (t >= kf.back().timeStamp)
            return {&kf.back(), &kf.back()};
        for (size_t i = 1; i < kf.size(); ++i)
            if (t < kf[i].timeStamp)
                return {&kf[i - 1], &kf[i]};
        return {nullptr, nullptr};
    }

    static glm::vec3 lerpVec3(const glm::vec3 &a, const glm::vec3 &b, float t)
    {
        return a + t * (b - a);
    }

    static glm::quat slerpQuat(const glm::quat &a, const glm::quat &b, float t)
    {
        return glm::slerp(a, b, t);
    }

    void MotionFeatureExtractor::calculateBoneTransformForPose(
        elix::engine::Skeleton::BoneInfo *boneInfo,
        const glm::mat4 &parentTransform,
        const elix::engine::Animation &anim,
        float timeTicks)
    {
        if (!boneInfo)
            return;

        glm::mat4 local = boneInfo->localBindTransform;

        if (const auto *track = anim.getAnimationTrack(boneInfo->name); track && !track->keyFrames.empty())
        {
            auto [s, e] = findKeyframes(track->keyFrames, timeTicks);
            if (s && e)
            {
                float dt = e->timeStamp - s->timeStamp;
                float t = (dt == 0.f) ? 0.f : glm::clamp((timeTicks - s->timeStamp) / dt, 0.f, 1.f);

                local = glm::translate(glm::mat4(1.f), lerpVec3(s->position, e->position, t)) *
                        glm::toMat4(glm::normalize(slerpQuat(s->rotation, e->rotation, t))) *
                        glm::scale(glm::mat4(1.f), lerpVec3(s->scale, e->scale, t));
            }
        }

        const glm::mat4 global = parentTransform * local;
        boneInfo->finalTransformation = global;

        // We don't have the skeleton pointer here, so we use childrenInfo pointers
        for (auto *child : boneInfo->childrenInfo)
            calculateBoneTransformForPose(child, global, anim, timeTicks);
    }

    bool MotionFeatureExtractor::evaluatePoseAt(const elix::engine::Animation &anim,
                                                elix::engine::Skeleton *skeleton,
                                                float timeTicks)
    {
        if (!skeleton || anim.boneAnimations.empty())
            return false;

        const glm::mat4 identity(1.f);
        for (size_t i = 0; i < skeleton->getBonesCount(); ++i)
        {
            auto *bone = skeleton->getBone(static_cast<int>(i));
            if (!bone || bone->parentId != -1)
                continue;
            calculateBoneTransformForPose(bone, identity, anim, timeTicks);
        }
        return true;
    }

    glm::vec3 MotionFeatureExtractor::bonePosition(elix::engine::Skeleton *skeleton,
                                                   const std::string &boneName)
    {
        if (!skeleton)
            return glm::vec3(0.f);
        auto *b = skeleton->getBone(boneName);
        return b ? glm::vec3(b->finalTransformation[3]) : glm::vec3(0.f);
    }

    MotionFeature MotionFeatureExtractor::extractAt(const elix::engine::Animation &anim,
                                                    elix::engine::Skeleton *skeleton,
                                                    const BoneNameMap &boneNames,
                                                    float timeSeconds,
                                                    uint32_t clipIndex,
                                                    uint32_t frameIndex,
                                                    float dt)
    {
        MotionFeature feat;
        feat.clipIndex = clipIndex;
        feat.frameIndex = frameIndex;
        feat.timeInClip = timeSeconds;

        if (!skeleton)
            return feat;

        const float tps = static_cast<float>(anim.ticksPerSecond > 0.0 ? anim.ticksPerSecond : 25.0);
        const float dur = static_cast<float>(anim.duration);

        auto clampTime = [&](float t) -> float
        {
            return std::fmod(std::max(0.f, t), std::max(dur, std::numeric_limits<float>::epsilon()));
        };

        const float ticks0 = clampTime(timeSeconds) * tps;
        evaluatePoseAt(anim, skeleton, ticks0);

        feat.leftFootPos = bonePosition(skeleton, boneNames.leftFoot);
        feat.rightFootPos = bonePosition(skeleton, boneNames.rightFoot);
        feat.leftHandPos = bonePosition(skeleton, boneNames.leftHand);
        feat.rightHandPos = bonePosition(skeleton, boneNames.rightHand);

        // Build future trajectory by sampling 0.1/0.2/0.3 s ahead (same clip, looped)
        auto samplePos = [&](float futureT)
        {
            const float ticksFuture = clampTime(timeSeconds + futureT) * tps;
            evaluatePoseAt(anim, skeleton, ticksFuture);
            return bonePosition(skeleton, boneNames.hip);
        };

        const glm::vec3 hipNow = bonePosition(skeleton, boneNames.hip);
        evaluatePoseAt(anim, skeleton, ticks0); // restore

        const glm::vec3 hip01 = samplePos(0.1f);
        const glm::vec3 hip02 = samplePos(0.2f);
        const glm::vec3 hip03 = samplePos(0.3f);

        feat.trajectoryPos0 = hip01 - hipNow;
        feat.trajectoryPos1 = hip02 - hipNow;
        feat.trajectoryPos2 = hip03 - hipNow;

        // Direction approximated from consecutive trajectory positions
        auto safeNorm = [](const glm::vec3 &v) -> glm::vec3
        {
            float len = glm::length(v);
            return len > 1e-6f ? v / len : glm::vec3(0.f, 0.f, 1.f);
        };

        feat.trajectoryDir0 = safeNorm(feat.trajectoryPos0);
        feat.trajectoryDir1 = safeNorm(feat.trajectoryPos1 - feat.trajectoryPos0);
        feat.trajectoryDir2 = safeNorm(feat.trajectoryPos2 - feat.trajectoryPos1);

        // Velocity: finite difference from (t - dt) to (t + dt)
        const float ticksPrev = clampTime(timeSeconds - dt) * tps;
        const float ticksNext = clampTime(timeSeconds + dt) * tps;

        evaluatePoseAt(anim, skeleton, ticksPrev);
        const glm::vec3 hipPrev = bonePosition(skeleton, boneNames.hip);
        const glm::vec3 leftFPrev = bonePosition(skeleton, boneNames.leftFoot);
        const glm::vec3 rightFPrev = bonePosition(skeleton, boneNames.rightFoot);

        evaluatePoseAt(anim, skeleton, ticksNext);
        const glm::vec3 hipNext = bonePosition(skeleton, boneNames.hip);
        const glm::vec3 leftFNext = bonePosition(skeleton, boneNames.leftFoot);
        const glm::vec3 rightFNext = bonePosition(skeleton, boneNames.rightFoot);

        const float invDt2 = 0.5f / std::max(dt, 1e-6f);
        feat.hipVelocity = (hipNext - hipPrev) * invDt2;
        feat.leftFootVelocity = (leftFNext - leftFPrev) * invDt2;
        feat.rightFootVelocity = (rightFNext - rightFPrev) * invDt2;

        // Restore pose at requested time
        evaluatePoseAt(anim, skeleton, ticks0);

        return feat;
    }

    MotionFeature MotionFeatureExtractor::buildQueryFeature(elix::engine::Skeleton *skeleton,
                                                            const BoneNameMap &boneNames,
                                                            const glm::vec3 &currentVelocity,
                                                            const glm::vec3 &desiredVelocity,
                                                            const glm::quat &characterRotation)
    {
        MotionFeature feat;
        feat.clipIndex = 0;
        feat.frameIndex = 0;
        feat.timeInClip = 0.f;

        if (!skeleton)
            return feat;

        // Current pose from the live skeleton (already filled by the component)
        feat.leftFootPos = bonePosition(skeleton, boneNames.leftFoot);
        feat.rightFootPos = bonePosition(skeleton, boneNames.rightFoot);
        feat.leftHandPos = bonePosition(skeleton, boneNames.leftHand);
        feat.rightHandPos = bonePosition(skeleton, boneNames.rightHand);

        feat.hipVelocity = currentVelocity;
        feat.leftFootVelocity = glm::vec3(0.f); // unknown at runtime without tracking history
        feat.rightFootVelocity = glm::vec3(0.f);

        // Predict trajectory: assume linear motion toward desiredVelocity
        // Rotate desiredVelocity to character-local space
        const glm::quat invRot = glm::inverse(characterRotation);
        const glm::vec3 localVel = invRot * desiredVelocity;

        feat.trajectoryPos0 = localVel * 0.1f;
        feat.trajectoryPos1 = localVel * 0.2f;
        feat.trajectoryPos2 = localVel * 0.3f;

        auto safeNorm = [](const glm::vec3 &v) -> glm::vec3
        {
            float len = glm::length(v);
            return len > 1e-6f ? v / len : glm::vec3(0.f, 0.f, 1.f);
        };

        feat.trajectoryDir0 = safeNorm(localVel);
        feat.trajectoryDir1 = feat.trajectoryDir0;
        feat.trajectoryDir2 = feat.trajectoryDir0;

        return feat;
    }

} // namespace elix::plugin::advancedanim
