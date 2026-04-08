//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_PLANE_H
#define DOGFIGHT_PLANE_H


#include <memory>
#include <vector>

#include "raylib.h"
#include "FSM.h"

class DStarLite;
class NavigationGraph;
class MPCController;

class Plane {
private:
    Model m_model{};

    Vector3 m_position;
    Vector3 m_velocity;
    Vector3 m_acceleration;
    float m_rotation;

    // float FuelLevel;
    // int AmmoCount;
    AIState CurrentState;
    AIEvent CurrentEvent;

    float m_speed = 100.0f;
    std::vector<Vector3> m_path;

    std::unique_ptr<DStarLite> m_dstar;

    NavigationGraph &m_graph;
    float m_pathUpdateTimer = 0.0f;

    Vector3 m_currentSteeringTarget;
    bool m_hasTarget = false;

    Vector3 m_lastEnemyPos = { 0 };
    Vector3 m_smoothedEnemyVel = { 0 };

    void UpdatePatrol( float deltaTime, const MPCController &mpc);
    void UpdatePursuit(const Vector3 &enemyPos, float deltaTime, const MPCController &mpc);

    Vector3 PredictEnemyPos(const Vector3 &enemyPos, float deltaTime);

    const Vector3& PredictEnemyPos(const Vector3 &enemyPos, float deltaTime) const;


public:


    [[nodiscard]] AIEvent GetCurrentEvent() const {
        return CurrentEvent;
    }

    void SetCurrentEvent(AIEvent current_event) {
        CurrentEvent = current_event;
    }



    Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration, const Color& color, NavigationGraph &graph);

    void SetDestinationViaAStar(int targetNodeIdx);

    ~Plane() = default;

    void Update(const Vector3 &enemyPos, float deltaTime, const MPCController &mpc);

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

    void SetCurrentState(AIState current_state) {
        CurrentState = current_state;
    }

    void set_m_acceleration(const Vector3 &m_acceleration) {
        this->m_acceleration = m_acceleration;
    }

    [[nodiscard]] AIState GetCurrentState() const {
        return CurrentState;
    }

};


#endif //DOGFIGHT_PLANE_H