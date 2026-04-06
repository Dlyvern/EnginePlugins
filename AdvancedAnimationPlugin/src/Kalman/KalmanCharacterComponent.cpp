#include "AdvancedAnimationPlugin/Kalman/KalmanCharacterComponent.hpp"

#include "Engine/Components/AnimatorComponent.hpp"
#include "Engine/Components/CharacterMovementComponent.hpp"
#include "Engine/Components/Transform3DComponent.hpp"
#include "Engine/Entity.hpp"

namespace elix::plugin::advancedanim
{

    void KalmanCharacterComponent::onOwnerAttached()
    {
        auto *entity = getOwner<elix::engine::Entity>();
        if (!entity)
            return;

        m_movement = entity->getComponent<elix::engine::CharacterMovementComponent>();
        m_animator = entity->getComponent<elix::engine::AnimatorComponent>();
        m_transform = entity->getComponent<elix::engine::Transform3DComponent>();
    }

    void KalmanCharacterComponent::update(float deltaTime)
    {
        if (!m_transform || deltaTime <= 0.f)
            return;

        const glm::vec3 rawPos = m_transform->getWorldPosition();

        if (!m_filterInitialized)
        {
            m_filter.init(rawPos, 0.01f, 0.1f);
            m_filterInitialized = true;
            return;
        }

        m_filter.predict(deltaTime);
        m_filter.update(rawPos);

        if (!m_animator)
            return;

        if (!m_speedParam.empty())
            m_animator->setFloat(m_speedParam, m_filter.getHorizontalSpeed());

        if (!m_verticalSpeedParam.empty())
            m_animator->setFloat(m_verticalSpeedParam, m_filter.getVelocity().y);
    }

    void KalmanCharacterComponent::setProcessNoise(float q)
    {
        m_filter.setProcessNoise(q);
    }

    void KalmanCharacterComponent::setMeasurementNoise(float r)
    {
        m_filter.setMeasurementNoise(r);
    }

    void KalmanCharacterComponent::clearAutoParameters()
    {
        m_speedParam.clear();
        m_verticalSpeedParam.clear();
    }

    glm::vec3 KalmanCharacterComponent::getSmoothedPosition() const
    {
        return m_filter.getPosition();
    }

    glm::vec3 KalmanCharacterComponent::getSmoothedVelocity() const
    {
        return m_filter.getVelocity();
    }

    float KalmanCharacterComponent::getSmoothedSpeed() const
    {
        return m_filter.getSpeed();
    }

    float KalmanCharacterComponent::getSmoothedHorizontalSpeed() const
    {
        return m_filter.getHorizontalSpeed();
    }

} // namespace elix::plugin::advancedanim
