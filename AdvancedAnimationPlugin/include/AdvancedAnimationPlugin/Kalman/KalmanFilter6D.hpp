#pragma once

#include <glm/vec3.hpp>
#include <array>

namespace elix::plugin::advancedanim
{

    struct Mat6
    {
        float data[36]{};

        static Mat6 identity();
        static Mat6 zeros();

        float &at(int row, int col) { return data[row * 6 + col]; }
        float at(int row, int col) const { return data[row * 6 + col]; }

        Mat6 operator+(const Mat6 &b) const;
        Mat6 operator*(const Mat6 &b) const;
        Mat6 transposed() const;

        // Scale every element by a scalar
        Mat6 operator*(float s) const;
    };

    struct Vec6
    {
        float data[6]{};

        float &operator[](int i) { return data[i]; }
        float operator[](int i) const { return data[i]; }
    };

    struct Mat3x6
    {
        float data[18]{};
        float &at(int r, int c) { return data[r * 6 + c]; }
        float at(int r, int c) const { return data[r * 6 + c]; }
    };

    struct Mat3
    {
        float data[9]{};
        static Mat3 identity();

        float &at(int r, int c) { return data[r * 3 + c]; }
        float at(int r, int c) const { return data[r * 3 + c]; }

        Mat3 operator+(const Mat3 &b) const;
        Mat3 inverse3x3() const;
    };

    struct Mat6x3
    {
        float data[18]{};
        float &at(int r, int c) { return data[r * 3 + c]; }
        float at(int r, int c) const { return data[r * 3 + c]; }
    };

    class KalmanFilter6D
    {
    public:
        KalmanFilter6D() = default;

        void init(const glm::vec3 &initialPosition, float processNoise, float measurementNoise);

        // Predict step: advance state estimate by dt seconds
        void predict(float dt);

        // Update step: incorporate a position measurement from PhysX / transform
        void update(const glm::vec3 &measuredPosition);

        glm::vec3 getPosition() const;
        glm::vec3 getVelocity() const;
        float getSpeed() const;
        float getHorizontalSpeed() const; // magnitude of (vx, vz) — ignores gravity

        void setProcessNoise(float q) { m_q = q; }
        void setMeasurementNoise(float r) { m_r = r; }

    private:
        Vec6 m_x{}; // state estimate
        Mat6 m_P{}; // error covariance
        float m_q{0.01f};
        float m_r{0.1f};
        bool m_initialized{false};
    };

} // namespace elix::plugin::advancedanim
