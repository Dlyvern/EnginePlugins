#include "AdvancedAnimationPlugin/MotionMatching/MotionDatabase.hpp"
#include "AdvancedAnimationPlugin/MotionMatching/MotionFeatureExtractor.hpp"

#include <cstring>
#include <fstream>
#include <cmath>

namespace elix::plugin::advancedanim
{

    bool MotionDatabase::buildFromClips(const std::vector<ClipDescriptor> &clips,
                                        elix::engine::Skeleton *skeleton,
                                        const std::vector<elix::engine::Animation> &animations,
                                        const BoneNameMap &boneNames,
                                        float sampleRate)
    {
        m_features.clear();

        if (!skeleton || sampleRate <= 0.f)
            return false;

        const float dt = 1.f / sampleRate;

        for (const auto &clip : clips)
        {
            if (clip.clipIndex < 0 || static_cast<size_t>(clip.clipIndex) >= animations.size())
                continue;

            const auto &anim = animations[static_cast<size_t>(clip.clipIndex)];
            if (anim.boneAnimations.empty() || anim.duration <= 0.0 || anim.ticksPerSecond <= 0.0)
                continue;

            const float durationSec = static_cast<float>(anim.duration / anim.ticksPerSecond);
            const int frameCount = static_cast<int>(std::ceil(durationSec * sampleRate));

            for (int f = 0; f < frameCount; ++f)
            {
                const float timeSec = static_cast<float>(f) * dt;
                MotionFeature feat = MotionFeatureExtractor::extractAt(
                    anim, skeleton, boneNames, timeSec,
                    static_cast<uint32_t>(clip.clipIndex),
                    static_cast<uint32_t>(f),
                    dt);

                m_features.push_back(feat);
            }
        }

        return !m_features.empty();
    }

    bool MotionDatabase::saveToDisk(const std::string &path) const
    {
        std::ofstream f(path, std::ios::binary);
        if (!f)
            return false;

        const uint32_t magic = 0x4D4D4442; // "MMDB"
        const uint32_t version = 1;
        const uint64_t count = m_features.size();

        f.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
        f.write(reinterpret_cast<const char *>(&version), sizeof(version));
        f.write(reinterpret_cast<const char *>(&count), sizeof(count));

        if (count > 0)
            f.write(reinterpret_cast<const char *>(m_features.data()),
                    static_cast<std::streamsize>(count * sizeof(MotionFeature)));

        return f.good();
    }

    bool MotionDatabase::loadFromDisk(const std::string &path)
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            return false;

        uint32_t magic{}, version{};
        uint64_t count{};
        f.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        f.read(reinterpret_cast<char *>(&version), sizeof(version));
        f.read(reinterpret_cast<char *>(&count), sizeof(count));

        if (magic != 0x4D4D4442 || version != 1 || !f)
            return false;

        m_features.resize(static_cast<size_t>(count));
        if (count > 0)
            f.read(reinterpret_cast<char *>(m_features.data()),
                   static_cast<std::streamsize>(count * sizeof(MotionFeature)));

        return f.good() || (f.eof() && count > 0);
    }

    float MotionDatabase::computeDistance(const MotionFeature &a,
                                          const MotionFeature &b,
                                          const MotionFeatureWeights &w) const
    {
        auto d3 = [](const glm::vec3 &x, const glm::vec3 &y) -> float
        {
            const glm::vec3 diff = x - y;
            return diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        };

        float dist = 0.f;

        // Pose positions
        dist += w.posePositionWeight * (d3(a.leftFootPos, b.leftFootPos) +
                                        d3(a.rightFootPos, b.rightFootPos) +
                                        d3(a.leftHandPos, b.leftHandPos) +
                                        d3(a.rightHandPos, b.rightHandPos));

        // Pose velocities
        dist += w.poseVelocityWeight * (d3(a.hipVelocity, b.hipVelocity) +
                                        d3(a.leftFootVelocity, b.leftFootVelocity) +
                                        d3(a.rightFootVelocity, b.rightFootVelocity));

        // Trajectory positions
        dist += w.trajectoryPosWeight * (d3(a.trajectoryPos0, b.trajectoryPos0) +
                                         d3(a.trajectoryPos1, b.trajectoryPos1) +
                                         d3(a.trajectoryPos2, b.trajectoryPos2));

        // Trajectory directions
        dist += w.trajectoryDirWeight * (d3(a.trajectoryDir0, b.trajectoryDir0) +
                                         d3(a.trajectoryDir1, b.trajectoryDir1) +
                                         d3(a.trajectoryDir2, b.trajectoryDir2));

        return dist;
    }

    const MotionFeature *MotionDatabase::findBestMatch(const MotionFeature &query,
                                                       const MotionFeatureWeights &weights,
                                                       uint32_t currentClipIndex,
                                                       uint32_t currentFrameIndex,
                                                       float minTimeSinceLastMatch,
                                                       float sampleRate) const
    {
        if (m_features.empty())
            return nullptr;

        const uint32_t minFrameGap = static_cast<uint32_t>(minTimeSinceLastMatch * sampleRate);

        const MotionFeature *best = nullptr;
        float bestD = std::numeric_limits<float>::max();

        for (const auto &feat : m_features)
        {
            // Skip frames too close to current position in the same clip
            if (feat.clipIndex == currentClipIndex)
            {
                uint32_t gap = (feat.frameIndex >= currentFrameIndex)
                                   ? (feat.frameIndex - currentFrameIndex)
                                   : (currentFrameIndex - feat.frameIndex);
                if (gap < minFrameGap)
                    continue;
            }

            const float d = computeDistance(query, feat, weights);
            if (d < bestD)
            {
                bestD = d;
                best = &feat;
            }
        }

        return best;
    }

} // namespace elix::plugin::advancedanim
