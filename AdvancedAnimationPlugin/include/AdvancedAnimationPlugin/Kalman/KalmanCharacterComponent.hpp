#pragma once

#include "AdvancedAnimationPlugin/Kalman/KalmanFilter6D.hpp"

#include "Engine/Components/ECS.hpp"

#include <glm/vec3.hpp>
#include <string>

namespace elix::engine
{
    class AnimatorComponent;
    class CharacterMovementComponent;
    class Transform3DComponent;
}

namespace elix::plugin::advancedanim
{

    class KalmanCharacterComponent final : public elix::engine::ECS
    {
    public:
        void update(float deltaTime) override;

        void setProcessNoise(float q);
        void setMeasurementNoise(float r);

        void setSpeedParameter(const std::string &paramName) { m_speedParam = paramName; }
        void setVerticalSpeedParameter(const std::string &paramName) { m_verticalSpeedParam = paramName; }
        void clearAutoParameters();

        glm::vec3 getSmoothedPosition() const;
        glm::vec3 getSmoothedVelocity() const;
        float getSmoothedSpeed() const;
        float getSmoothedHorizontalSpeed() const;

    protected:
        void onOwnerAttached() override;

    private:
        elix::engine::CharacterMovementComponent *m_movement{nullptr};
        elix::engine::AnimatorComponent *m_animator{nullptr};
        elix::engine::Transform3DComponent *m_transform{nullptr};

        KalmanFilter6D m_filter;
        bool m_filterInitialized{false};

        std::string m_speedParam;
        std::string m_verticalSpeedParam;
    };

} // namespace elix::plugin::advancedanim
