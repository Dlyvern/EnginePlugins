#include "AdvancedAnimationPlugin/Kalman/KalmanFilter6D.hpp"

#include <cmath>

namespace elix::plugin::advancedanim
{

    Mat6 Mat6::identity()
    {
        Mat6 m;
        for (int i = 0; i < 6; ++i)
            m.at(i, i) = 1.f;
        return m;
    }

    Mat6 Mat6::zeros()
    {
        return Mat6{};
    }

    Mat6 Mat6::operator+(const Mat6 &b) const
    {
        Mat6 r;
        for (int i = 0; i < 36; ++i)
            r.data[i] = data[i] + b.data[i];
        return r;
    }

    Mat6 Mat6::operator*(const Mat6 &b) const
    {
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
            {
                float s = 0.f;
                for (int k = 0; k < 6; ++k)
                    s += at(i, k) * b.at(k, j);
                r.at(i, j) = s;
            }
        return r;
    }

    Mat6 Mat6::transposed() const
    {
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                r.at(j, i) = at(i, j);
        return r;
    }

    Mat6 Mat6::operator*(float s) const
    {
        Mat6 r;
        for (int i = 0; i < 36; ++i)
            r.data[i] = data[i] * s;
        return r;
    }

    Mat3 Mat3::identity()
    {
        Mat3 m;
        m.at(0, 0) = m.at(1, 1) = m.at(2, 2) = 1.f;
        return m;
    }

    Mat3 Mat3::operator+(const Mat3 &b) const
    {
        Mat3 r;
        for (int i = 0; i < 9; ++i)
            r.data[i] = data[i] + b.data[i];
        return r;
    }

    // 3×3 matrix inverse via cofactor expansion (Cramer's rule)
    Mat3 Mat3::inverse3x3() const
    {
        float a = at(0, 0), b = at(0, 1), c = at(0, 2);
        float d = at(1, 0), e = at(1, 1), f = at(1, 2);
        float g = at(2, 0), h = at(2, 1), k = at(2, 2);

        float A = (e * k - f * h);
        float B = -(d * k - f * g);
        float C = (d * h - e * g);

        float det = a * A + b * B + c * C;
        if (std::abs(det) < 1e-12f)
            return Mat3::identity(); // fallback

        float inv = 1.f / det;

        Mat3 r;
        r.at(0, 0) = (e * k - f * h) * inv;
        r.at(0, 1) = -(b * k - c * h) * inv;
        r.at(0, 2) = (b * f - c * e) * inv;
        r.at(1, 0) = B * inv;
        r.at(1, 1) = (a * k - c * g) * inv;
        r.at(1, 2) = -(a * f - c * d) * inv;
        r.at(2, 0) = C * inv;
        r.at(2, 1) = -(a * h - b * g) * inv;
        r.at(2, 2) = (a * e - b * d) * inv;
        return r;
    }

    // Compute P * Hᵀ where H = [I₃ | 0₃] — result is 6×3
    static Mat6x3 multiplyPHt(const Mat6 &P)
    {
        Mat6x3 r;
        // H selects the first 3 columns, so PHᵀ = first 3 columns of P
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                r.at(i, j) = P.at(i, j);
        return r;
    }

    // Compute H * P * Hᵀ where H = [I₃ | 0₃] — result is 3×3 (top-left 3×3 of P)
    static Mat3 multiplyHPHt(const Mat6 &P)
    {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.at(i, j) = P.at(i, j);
        return r;
    }

    // Compute K * innovation where K is 6×3 and innovation is 3-vector → 6-vector
    static Vec6 multiplyKv(const Mat6x3 &K, float innov[3])
    {
        Vec6 r;
        for (int i = 0; i < 6; ++i)
        {
            float s = 0.f;
            for (int j = 0; j < 3; ++j)
                s += K.at(i, j) * innov[j];
            r[i] = s;
        }
        return r;
    }

    // Compute K * H * P (where H = [I₃|0₃]) — result is 6×6
    // K * H has rows = K's rows, cols = 6, with the 3 left cols from K and right 3 = 0
    static Mat6 multiplyKHP(const Mat6x3 &K, const Mat6 &P)
    {
        // KH = [K | 0] (6×6): left 3 cols are K, right 3 are zero
        // Then (KH) * P
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
            {
                float s = 0.f;
                for (int k = 0; k < 3; ++k) // only first 3 cols of KH are non-zero
                    s += K.at(i, k) * P.at(k, j);
                r.at(i, j) = s;
            }
        return r;
    }

    void KalmanFilter6D::init(const glm::vec3 &initialPosition, float processNoise, float measurementNoise)
    {
        m_q = processNoise;
        m_r = measurementNoise;

        // State: position = initialPosition, velocity = 0
        m_x[0] = initialPosition.x;
        m_x[1] = initialPosition.y;
        m_x[2] = initialPosition.z;
        m_x[3] = m_x[4] = m_x[5] = 0.f;

        // Initial covariance: large uncertainty
        m_P = Mat6::identity() * 1.0f;

        m_initialized = true;
    }

    void KalmanFilter6D::predict(float dt)
    {
        if (!m_initialized)
            return;

        // State transition: x_new = F * x
        //  px += vx*dt,  py += vy*dt,  pz += vz*dt
        m_x[0] += m_x[3] * dt;
        m_x[1] += m_x[4] * dt;
        m_x[2] += m_x[5] * dt;

        // P_new = F * P * Fᵀ + Q
        // F = I + dt*A where A has 1s at [0,3],[1,4],[2,5]
        // F*P*Fᵀ computed blockwise:
        //   [I  dt*I] [P00 P01] [I  0 ]   P00 + dt*(P10+P01) + dt²*P11    P01 + dt*P11
        //   [0   I  ] [P10 P11] [dt*I I] = P10 + dt*P11                    P11
        // where P is partitioned 3×3 blocks: P00=P[0:3,0:3], P01=P[0:3,3:6], etc.

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                // P00 += dt*(P10 + P01) + dt²*P11
                m_P.at(i, j) += dt * (m_P.at(i + 3, j) + m_P.at(i, j + 3)) + dt * dt * m_P.at(i + 3, j + 3);
                // P01 += dt*P11
                m_P.at(i, j + 3) += dt * m_P.at(i + 3, j + 3);
                // P10 += dt*P11
                m_P.at(i + 3, j) += dt * m_P.at(i + 3, j + 3);
            }

        // Add process noise Q = q * I₆
        for (int i = 0; i < 6; ++i)
            m_P.at(i, i) += m_q;
    }

    void KalmanFilter6D::update(const glm::vec3 &measuredPosition)
    {
        if (!m_initialized)
            return;

        // Innovation: y = z - H*x = measuredPos - estimated pos
        float innov[3] = {
            measuredPosition.x - m_x[0],
            measuredPosition.y - m_x[1],
            measuredPosition.z - m_x[2]};

        // S = H P Hᵀ + R  (3×3)
        Mat3 S = multiplyHPHt(m_P);
        for (int i = 0; i < 3; ++i)
            S.at(i, i) += m_r;

        // K = P Hᵀ S⁻¹  (6×3)
        Mat6x3 PHt = multiplyPHt(m_P);
        Mat3 Sinv = S.inverse3x3();
        Mat6x3 K;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
            {
                float s = 0.f;
                for (int k = 0; k < 3; ++k)
                    s += PHt.at(i, k) * Sinv.at(k, j);
                K.at(i, j) = s;
            }

        // x = x + K * y
        Vec6 Ky = multiplyKv(K, innov);
        for (int i = 0; i < 6; ++i)
            m_x[i] += Ky[i];

        // P = (I - K H) P
        Mat6 KHP = multiplyKHP(K, m_P);
        Mat6 I = Mat6::identity();
        Mat6 IminusKH;
        for (int i = 0; i < 36; ++i)
            IminusKH.data[i] = I.data[i] - KHP.data[i];

        m_P = IminusKH * m_P;
    }

    glm::vec3 KalmanFilter6D::getPosition() const
    {
        return {m_x[0], m_x[1], m_x[2]};
    }

    glm::vec3 KalmanFilter6D::getVelocity() const
    {
        return {m_x[3], m_x[4], m_x[5]};
    }

    float KalmanFilter6D::getSpeed() const
    {
        const glm::vec3 v = getVelocity();
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    float KalmanFilter6D::getHorizontalSpeed() const
    {
        return std::sqrt(m_x[3] * m_x[3] + m_x[5] * m_x[5]);
    }

} // namespace elix::plugin::advancedanim
