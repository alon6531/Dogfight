//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_PLANE_H
#define DOGFIGHT_PLANE_H


#include <vector>

#include "raylib.h"

class Plane {
private:
    Model m_model{};

    Vector3 m_position;
    Vector3 m_velocity;
    Vector3 m_acceleration;
    //Quaternion rotation;

    // float FuelLevel;
    // int AmmoCount;
    // enum CurrentState;

    float m_speed = 50.0f;
    std::vector<Vector3> m_path;
    int m_targetPathIdx = 0;

public:
    Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration);

    void SetDestination(int targetNodeIdx, class NavigationGraph &graph);


    ~Plane() = default;

    void Update(float deltaTime);
    void Draw() const;


    [[nodiscard]] Vector3 position() const {
        return m_position;
    }

    void set_m_position(const Vector3 &m_position) {
        this->m_position = m_position;
    }

    [[nodiscard]] Vector3 velocity() const {
        return m_velocity;
    }

    void set_m_velocity(const Vector3 &m_velocity) {
        this->m_velocity = m_velocity;
    }

    [[nodiscard]] Vector3 acceleration() const {
        return m_acceleration;
    }

    void set_m_acceleration(const Vector3 &m_acceleration) {
        this->m_acceleration = m_acceleration;
    }
};


#endif //DOGFIGHT_PLANE_H