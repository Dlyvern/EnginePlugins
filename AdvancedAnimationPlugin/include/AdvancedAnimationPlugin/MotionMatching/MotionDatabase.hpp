#pragma once

#include "Engine/Components/AnimatorComponent.hpp" // Animation, SQT
#include "Engine/Skeleton.hpp"

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

namespace elix::plugin::advancedanim
{

    struct BoneNameMap
    {
        std::string hip{"Hips"};
        std::string leftFoot{"LeftFoot"};
        std::string rightFoot{"RightFoot"};
        std::string leftHand{"LeftHand"};
        std::string rightHand{"RightHand"};
    };

    struct MotionFeature
    {
        // Pose features (model-space)
        glm::vec3 leftFootPos{0.f};
        glm::vec3 rightFootPos{0.f};
        glm::vec3 leftHandPos{0.f};
        glm::vec3 rightHandPos{0.f};
        glm::vec3 hipVelocity{0.f};
        glm::vec3 leftFootVelocity{0.f};
        glm::vec3 rightFootVelocity{0.f};

        // Future trajectory (character-local XZ plane)
        glm::vec3 trajectoryPos0{0.f}; // position offset at +0.1 s
        glm::vec3 trajectoryPos1{0.f}; // position offset at +0.2 s
        glm::vec3 trajectoryPos2{0.f}; // position offset at +0.3 s
        glm::vec3 trajectoryDir0{0.f}; // facing direction at +0.1 s
        glm::vec3 trajectoryDir1{0.f}; // facing direction at +0.2 s
        glm::vec3 trajectoryDir2{0.f}; // facing direction at +0.3 s

        // Source info (not part of distance calculation)
        uint32_t clipIndex{0};
        uint32_t frameIndex{0};
        float timeInClip{0.f};
    };

    // Weight multipliers applied per-group when computing feature distance.
    struct MotionFeatureWeights
    {
        float posePositionWeight{1.0f};  // foot/hand positions
        float poseVelocityWeight{0.5f};  // hip + foot velocities
        float trajectoryPosWeight{2.0f}; // future trajectory positions
        float trajectoryDirWeight{1.0f}; // future trajectory directions
    };

    // Pre-baked database of motion features extracted from a set of animation clips.
    // Build once (offline or on first load), save to disk, load at runtime.
    class MotionDatabase
    {
    public:
        struct ClipDescriptor
        {
            std::string assetPath; // for display/debug purposes
            int clipIndex{0};      // index into the animations array passed to buildFromClips
            bool loop{true};
        };

        // Build the database by sampling each animation at sampleRate Hz.
        bool buildFromClips(const std::vector<ClipDescriptor> &clips,
                            elix::engine::Skeleton *skeleton,
                            const std::vector<elix::engine::Animation> &animations,
                            const BoneNameMap &boneNames,
                            float sampleRate = 30.f);

        // Binary save/load (fast startup)
        bool saveToDisk(const std::string &path) const;
        bool loadFromDisk(const std::string &path);

        // Find the best matching frame for a query feature.
        // currentClipIndex / currentFrameIndex — used to enforce the minTimeSinceLastMatch
        // guard (prevents thrashing when current and best match are close in time).
        const MotionFeature *findBestMatch(const MotionFeature &query,
                                           const MotionFeatureWeights &weights,
                                           uint32_t currentClipIndex,
                                           uint32_t currentFrameIndex,
                                           float minTimeSinceLastMatch,
                                           float sampleRate = 30.f) const;

        size_t featureCount() const { return m_features.size(); }
        const std::vector<MotionFeature> &getFeatures() const { return m_features; }

    private:
        float computeDistance(const MotionFeature &a,
                              const MotionFeature &b,
                              const MotionFeatureWeights &w) const;

        std::vector<MotionFeature> m_features;
    };

} // namespace elix::plugin::advancedanim
