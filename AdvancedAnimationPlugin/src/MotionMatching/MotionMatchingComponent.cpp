#include "AdvancedAnimationPlugin/MotionMatching/MotionMatchingComponent.hpp"
#include "AdvancedAnimationPlugin/MotionMatching/MotionFeatureExtractor.hpp"

#include "Engine/Components/SkeletalMeshComponent.hpp"
#include "Engine/Components/Transform3DComponent.hpp"
#include "Engine/Entity.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>

namespace elix::plugin::advancedanim
{

    void MotionMatchingComponent::onOwnerAttached()
    {
        auto *entity = getOwner<elix::engine::Entity>();
        if (!entity)
            return;

        m_skeletalMesh = entity->getComponent<elix::engine::SkeletalMeshComponent>();
        m_transform = entity->getComponent<elix::engine::Transform3DComponent>();
    }

    void MotionMatchingComponent::setDatabase(std::shared_ptr<MotionDatabase> db)
    {
        m_database = std::move(db);
        m_currentFeature = nullptr;
        m_previousFeature = nullptr;
        m_blendAlpha = 1.f;
        m_timeSinceLastMatch = 0.f;
    }

    void MotionMatchingComponent::setBoneNameMap(const BoneNameMap &map)
    {
        m_boneNames = map;
    }

    void MotionMatchingComponent::setAnimations(const std::vector<elix::engine::Animation> &animations)
    {
        m_animations = animations;
    }

    void MotionMatchingComponent::addPostPoseHook(const void *ownerKey, PostPoseHook fn)
    {
        m_postPoseHooks.push_back({ownerKey, std::move(fn)});
    }

    void MotionMatchingComponent::removePostPoseHook(const void *ownerKey)
    {
        m_postPoseHooks.erase(
            std::remove_if(m_postPoseHooks.begin(), m_postPoseHooks.end(),
                           [ownerKey](const PostPoseHookEntry &e)
                           { return e.key == ownerKey; }),
            m_postPoseHooks.end());
    }

    void MotionMatchingComponent::firePostPoseHooks(elix::engine::Skeleton &skeleton)
    {
        for (auto &h : m_postPoseHooks)
            h.fn(skeleton);
    }

    void MotionMatchingComponent::update(float deltaTime)
    {
        if (!m_skeletalMesh || !m_database || m_animations.empty() || deltaTime <= 0.f)
            return;

        auto &skeleton = m_skeletalMesh->getSkeleton();

        m_timeSinceLastMatch += deltaTime;
        m_currentTimeInClip += deltaTime;

        if (m_timeSinceLastMatch >= m_minMatchInterval)
        {
            // Build query from current skeleton pose + desired velocity
            const glm::quat charRot = m_transform ? m_transform->getWorldRotation() : glm::quat(1.f, 0.f, 0.f, 0.f);
            const MotionFeature query = MotionFeatureExtractor::buildQueryFeature(
                &skeleton, m_boneNames,
                m_currentAnimatedVelocity, m_desiredVelocity,
                charRot);

            const uint32_t currClip = m_currentFeature ? m_currentFeature->clipIndex : ~0u;
            const uint32_t currFrame = m_currentFeature ? m_currentFeature->frameIndex : ~0u;

            const MotionFeature *bestMatch = m_database->findBestMatch(
                query, m_weights, currClip, currFrame, m_minMatchInterval);

            if (bestMatch && bestMatch != m_currentFeature)
            {
                m_previousFeature = m_currentFeature;
                m_currentFeature = bestMatch;
                m_currentTimeInClip = bestMatch->timeInClip;
                m_blendAlpha = (m_previousFeature == nullptr) ? 1.f : 0.f;
                m_timeSinceLastMatch = 0.f;
            }
        }

        if (!m_currentFeature)
            return;

        if (m_blendDuration > 0.f)
            m_blendAlpha = std::min(m_blendAlpha + deltaTime / m_blendDuration, 1.f);
        else
            m_blendAlpha = 1.f;

        applyPoseFromFeature(*m_currentFeature, m_blendAlpha, m_previousFeature);

        // Update animated velocity for next frame's query
        {
            const auto &anim = m_animations[m_currentFeature->clipIndex];
            const float tps = static_cast<float>(anim.ticksPerSecond > 0.0 ? anim.ticksPerSecond : 25.0);
            const float dur = static_cast<float>(anim.duration);
            if (dur > 0.f && tps > 0.f)
            {
                const float t0 = std::fmod(m_currentFeature->timeInClip, dur / tps);
                const float t1 = std::fmod(t0 + deltaTime, dur / tps);
                // Approximate hip velocity from the database for this clip frame
                m_currentAnimatedVelocity = m_currentFeature->hipVelocity;
            }
        }

        firePostPoseHooks(skeleton);
    }

    void MotionMatchingComponent::applyPoseFromFeature(const MotionFeature &feature,
                                                       float blendAlpha,
                                                       const MotionFeature *blendFrom)
    {
        if (feature.clipIndex >= m_animations.size())
            return;

        auto &skeleton = m_skeletalMesh->getSkeleton();
        const auto &animCurr = m_animations[feature.clipIndex];
        const float tpsCurr = static_cast<float>(animCurr.ticksPerSecond > 0.0 ? animCurr.ticksPerSecond : 25.0);
        const float durCurr = static_cast<float>(animCurr.duration);

        // Clamp/wrap time
        float timeSec = m_currentTimeInClip;
        if (durCurr > 0.f && tpsCurr > 0.f)
        {
            const float durSec = durCurr / tpsCurr;
            timeSec = durSec > 0.f ? std::fmod(std::max(0.f, timeSec), durSec) : 0.f;
        }

        const float ticksCurr = timeSec * tpsCurr;

        if (blendFrom && blendAlpha < 1.f && blendFrom->clipIndex < m_animations.size())
        {
            // Blend from previous feature to current
            const auto &animPrev = m_animations[blendFrom->clipIndex];
            const float tpsPrev = static_cast<float>(animPrev.ticksPerSecond > 0.0 ? animPrev.ticksPerSecond : 25.0);
            const float durPrev = static_cast<float>(animPrev.duration);
            const float durSecPrev = (durPrev > 0.f && tpsPrev > 0.f) ? durPrev / tpsPrev : 0.f;
            const float timeSecPrev = durSecPrev > 0.f
                                          ? std::fmod(std::max(0.f, blendFrom->timeInClip), durSecPrev)
                                          : 0.f;
            const float ticksPrev = timeSecPrev * tpsPrev;

            // Evaluate both poses and blend bone-by-bone
            // We store the "previous" pose in a temporary copy of bone matrices, then lerp
            MotionFeatureExtractor::evaluatePoseAt(animPrev, &skeleton, ticksPrev);
            const size_t boneCount = skeleton.getBonesCount();
            std::vector<glm::mat4> prevPose(boneCount);
            for (size_t i = 0; i < boneCount; ++i)
            {
                auto *b = skeleton.getBone(static_cast<int>(i));
                if (b)
                    prevPose[i] = b->finalTransformation;
            }

            MotionFeatureExtractor::evaluatePoseAt(animCurr, &skeleton, ticksCurr);

            for (size_t i = 0; i < boneCount; ++i)
            {
                auto *b = skeleton.getBone(static_cast<int>(i));
                if (!b)
                    continue;

                // Blend position
                const glm::vec3 posA = glm::vec3(prevPose[i][3]);
                const glm::vec3 posB = glm::vec3(b->finalTransformation[3]);

                // Blend rotation (extract from matrix columns, slerp as quaternions)
                auto extractRot = [](const glm::mat4 &m) -> glm::quat
                {
                    glm::vec3 c0 = glm::normalize(glm::vec3(m[0]));
                    glm::vec3 c1 = glm::normalize(glm::vec3(m[1]));
                    glm::vec3 c2 = glm::normalize(glm::vec3(m[2]));
                    glm::mat3 rotM(c0, c1, c2);
                    return glm::normalize(glm::quat_cast(rotM));
                };

                const glm::quat qA = extractRot(prevPose[i]);
                const glm::quat qB = extractRot(b->finalTransformation);
                const glm::quat qBlend = glm::slerp(qA, qB, blendAlpha);
                const glm::vec3 posBlend = glm::mix(posA, posB, blendAlpha);

                glm::mat4 blended = glm::mat4_cast(qBlend);
                blended[3] = glm::vec4(posBlend, 1.f);
                b->finalTransformation = blended;
            }
        }
        else
        {
            MotionFeatureExtractor::evaluatePoseAt(animCurr, &skeleton, ticksCurr);
        }
    }

} // namespace elix::plugin::advancedanim
