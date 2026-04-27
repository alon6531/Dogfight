#ifndef DOGFIGHT_PLANE_H
#define DOGFIGHT_PLANE_H

#include <deque>
#include <memory>
#include <vector>
#include "raylib.h"
#include "AIState/Base/AIState.h"
#include "Control/MPCController.h"

#define MASS 5.0f
#define GRAVITY 10.0f

#define MAX_THRUST 170.0f
#define CLIMB_EXTRA_POWER 80.0f
#define SLOPE_EFFECT 120.0f
#define DRAG_COEFF 0.02f

#define MAX_FUEL 50000.0f
#define ESCAPE_FUEL (MAX_FUEL * 0.3)





class NavigationGraph;

struct TargetLock {
    float lockProgress = 0.0f;
    float lockTimer = 1.0f;
    bool isLocked = false;
    bool finalLock = false;
};


class Plane {
public:
    Plane(const Vector3 &position, const Vector3 &velocity, const Color &color, NavigationGraph& graph, const Vector3 &targetPos);

    virtual ~Plane();


    void Update(float deltaTime, NavigationGraph& graph, const Map& map, const std::vector<Obstacle>& obstacles);

    void SteerTowards(Vector3 target, float deltaTime);

    void Draw();

    void DrawForceVectors() const;

    void DrawHub(bool showDebug) const;
    void DrawLocked(Camera3D camera) const;

    // Getters & Setters
    [[nodiscard]] Vector3 GetPosition() const { return m_position; }
    [[nodiscard]] Vector3 GetVelocity() const { return m_velocity; }

    void SetEnemy(std::shared_ptr<Plane> &enemy) {
        m_enemy = enemy;
    }

    void SetPosition(const Vector3 &position) {
        this->m_position = position;
    }

    void SetVelocity(const Vector3 &velocity) {
        this->m_velocity = velocity;
    }

    [[nodiscard]] Vector3 GetTargetPos() const {
        return m_targetPos;
    }

    void SetTargetPos(const Vector3 &targetPos) {
        m_targetPos = targetPos;
    }

    [[nodiscard]] Vector3 GetBasePos() const {
        return m_basePos;
    }

    [[nodiscard]] Vector3 GetForward() const {
        return m_forward;
    }

    [[nodiscard]] AIStateType GetCurrentStateType() const {
        return m_currentStateType;
    }

    void SetThrust(float thrust) {
        this->m_thrust = thrust;
    }

    [[nodiscard]] float GetFuel() const {
        return m_fuel;
    }

    void SetFuel(float fuel) { m_fuel = fuel; }


    [[nodiscard]] float GetThrust() const {return this->m_thrust;};

    const TargetLock& GetTargetLock() const {return m_targetLock;};

    [[nodiscard]] Vector3 GetLiftPoint() const {
        return m_liftPoint;
    }

    void SetLiftPoint(const Vector3 &lift_point) {
        m_liftPoint = lift_point;
    }

private:
    Mesh m_mesh{};
    Model m_model{};

    Vector3 m_position = {};
    Vector3 m_velocity = {};
    Vector3 m_acceleration = {};


    Vector3 m_basePos = {};
    Vector3 m_targetPos = {};
    Vector3 m_liftPoint = {};

    float m_rotation = 0.0f;
    float m_thrust = 0.0f;
    float m_lift = 0.0f;
    float m_drug = 0.0f;
    float m_gravity = 0.0f;
    float m_normal = 0.0f;
    Vector3 m_forward = {};
    float m_bankAngle = 0.0f;
    float m_fuel = 0.0f;
    int m_emmoCount = 1.0f;


    std::unique_ptr<AIState> m_fsm = nullptr;
    AIStateType m_currentStateType = AIStateType::IDLE;
    std::shared_ptr<Plane> m_enemy = nullptr;
    std::shared_ptr<MPCController> m_mpc = nullptr;

    TargetLock m_targetLock;



    void ChangeAIState(AIStateType newState, NavigationGraph& graph);
    void UpdatePhysics(float deltaTime, const Map& map, const std::vector<Obstacle>& obstacles);

    void CheckGroundCollision(const Map &map, float deltaTime);

    void DrawPath() const;

    void UpdateLockSystem(float deltaTime);



};

#endif