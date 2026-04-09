#ifndef DOGFIGHT_PLANE_H
#define DOGFIGHT_PLANE_H

#include <deque>
#include <memory>
#include <vector>

#include "raylib.h"
#include "FSM.h"

#define NORMAL_SPEED 100.0f
#define EVADE_SPEED 150.0f
#define EVADE_TURN_SPEED 120.0f

// Forward declarations
class DStarLite;
class NavigationGraph;
class MPCController;

class Plane {
public:
    Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration, const Color& color, NavigationGraph &graph);
    ~Plane();

    void Update(Plane &enemy, float deltaTime, const MPCController &mpc);
    void Draw(Camera3D camera) const;
    void SetDestinationViaAStar(int targetNodeIdx);

    // Getters & Setters
    [[nodiscard]] Vector3 GetPosition() const { return m_position; }
    void SetPosition(const Vector3 &position) { m_position = position; }

    [[nodiscard]] Vector3 GetVelocity() const { return m_velocity; }
    void SetVelocity(const Vector3 &velocity) { m_velocity = velocity; }

    [[nodiscard]] Vector3 GetAcceleration() const { return m_acceleration; }
    void SetAcceleration(const Vector3 &acceleration) { m_acceleration = acceleration; }

    [[nodiscard]] AIState GetCurrentState() const { return CurrentState; }
    void SetCurrentState(AIState state) { CurrentState = state; }

    [[nodiscard]] AIEvent GetCurrentEvent() const { return CurrentEvent; }
    void SetCurrentEvent(AIEvent event) { CurrentEvent = event; }

    [[nodiscard]] bool GetLocked() const {
        return m_locked;
    }

    void SetIsLocked(bool m_locked) {
        this->m_locked = m_locked;
    }

    void UpdateLockTimer(float dt, bool isBeingTargeted) {
        if (isBeingTargeted)
        {
            m_lockTimer += dt;
            m_locked = true;
        }
        else m_lockTimer = 0.0f; // איבוד נעילה מאפס את הטיימר


    }

private:
    static constexpr float UPDATE_PATH_DELAY = 0.02;

    Model m_model{};
    Matrix m_baseTransform{}; // שומר את היישור הראשוני של המודל

    Vector3 m_position;
    Vector3 m_velocity;
    Vector3 m_acceleration;

    float m_rotation = 0.0f;
    float m_tilt = 0.0f;
    float m_speed = 100.0f;
    float m_delay = 0.0f;

    AIState CurrentState;
    AIEvent CurrentEvent;

    std::deque<Vector3> m_path;
    std::unique_ptr<DStarLite> m_dstar;
    NavigationGraph &m_graph;
    Vector3 m_lastPathTarget = {0, 0, 0};

    bool m_locked = false;
    float m_lockTimer = 0.0f;

public:
    [[nodiscard]] float GetLockTimer() const {
        return m_lockTimer;
    }

private:
    void UpdatePatrol(float deltaTime, const MPCController &mpc);

    void ApplyWorldBounds(float deltaTime);

    void UpdatePursuit(Plane &enemy, float deltaTime, const MPCController &mpc, bool shouldUpdatePath);
    void UpdateEvade(const Vector3 &enemyPos, const Vector3 &enemyVel, float deltaTime, const MPCController &mpc, bool shouldUpdatePath);

    void UpdateRotationAndTilt(float deltaTime);
};

#endif // DOGFIGHT_PLANE_H
