#ifndef DOGFIGHT_PLANE_H
#define DOGFIGHT_PLANE_H

#include <deque>
#include <memory>
#include <vector>
#include "raylib.h"
#include "AIState/Base/AIState.h"

// --- Constructor ---
#define PLANE_MODEL_PATH                "Assets/plane.glb"
#define PLANE_FORWARD_DEFAULT           { 1, 0, 0 }
#define PLANE_MPC_HORIZON               0
#define PLANE_MPC_DT                    0

// --- Update ---
#define PLANE_ROTATION_SPEED            2.5f
#define PLANE_FUEL_LOW_WINDOW           30.0f

// --- fuel
#define MAX_FUEL 50000.0f
#define ESCAPE_FUEL (MAX_FUEL * 0.3)

// --- UpdatePhysics ---
#define PLANE_LIFT_COEFF                0.85f
#define MASS 5.0f
#define GRAVITY 10.0f
#define MAX_THRUST 170.0f
#define CLIMB_EXTRA_POWER 80.0f
#define SLOPE_EFFECT 120.0f
#define DRAG_COEFF 0.02f


// --- CheckGroundCollision ---
#define PLANE_GROUND_SAFE_DIST          5.0f
#define PLANE_GROUND_SNAP_OFFSET        0.0f
#define PLANE_GROUND_FRICTION_THRUST    10.0f
#define PLANE_GROUND_FRICTION_SCALE     0.05f

// --- SteerTowards ---
#define PLANE_STEER_ROTATION_SPEED      2.0f

// --- UpdateLockSystem ---
#define LOCK_MAX_DIST                   800.0f
#define LOCK_CONE                       0.98f
#define LOCK_SPEED                      0.25f
#define LOCK_HEIGHT_ADVANTAGE           30.0f
#define LOCK_PREDICTION_SCALE           0.002f
#define LOCK_DECAY_SPEED                0.8f

// --- Draw ---
#define PLANE_SCALE                     0.1f
#define PLANE_CORRECTION_ROT_Y         90.0f
#define PLANE_CORRECTION_ROT_X         90.0f

// --- DrawForceVectors ---
#define FORCE_VISUAL_SCALE              0.5f
#define FORCE_HEAD_SIZE                 0.5f
#define FORCE_MIN_MAGNITUDE             0.1f

// --- DrawLocked ---
#define LOCK_SCREEN_MARGIN              0.0f
#define LOCK_BOX_SIZE_LOCKED            40.0f
#define LOCK_BOX_SIZE_UNLOCKED          60.0f
#define LOCK_BOX_BORDER                 2.0f
#define LOCK_TEXT_FONT_SIZE             20
#define LOCK_TEXT_Y_OFFSET             -25
#define LOCK_DIST_FONT_SIZE             15
#define LOCK_DIST_Y_OFFSET               5
#define LOCK_ALT_TEXT_Y_OFFSET          15
#define LOCK_ALT_FONT_SIZE              10
#define LOCK_ALT_X_OFFSET              -45
#define LOCK_LOW_HEIGHT_THRESHOLD       30.0f
#define LOCK_PROGRESS_BAR_X_OFFSET     -30
#define LOCK_PROGRESS_BAR_Y_OFFSET      35
#define LOCK_PROGRESS_BAR_WIDTH         60
#define LOCK_PROGRESS_BAR_HEIGHT         4

// --- DrawHub ---
#define HUD_PANEL_X                     15
#define HUD_PANEL_Y                     15
#define HUD_PANEL_WIDTH                280
#define HUD_PANEL_HEIGHT               400
#define HUD_BG_ALPHA                    0.7f
#define HUD_BORDER_ALPHA                0.5f
#define HUD_BORDER_THICKNESS            2.0f
#define HUD_INNER_MARGIN                20
#define HUD_TITLE_FONT_SIZE             16
#define HUD_TITLE_LINE_HEIGHT           25
#define HUD_MODE_FONT_SIZE              22
#define HUD_MODE_LABEL_OFFSET           85
#define HUD_MODE_LINE_HEIGHT            45
#define HUD_SECTION_FONT_SIZE           14
#define HUD_SECTION_LINE_HEIGHT         30
#define HUD_STAT_FONT_SIZE              18
#define HUD_STAT_VALUE_OFFSET          120
#define HUD_STAT_LINE_HEIGHT            25
#define HUD_STAT_GAP                    15
#define HUD_SPEED_FONT_SIZE             22
#define HUD_SPEED_BAR_WIDTH            240
#define HUD_SPEED_BAR_HEIGHT            10
#define HUD_SPEED_BAR_SCALE              2.0f
#define HUD_SPEED_LINE_HEIGHT           30
#define HUD_SPEED_BAR_LINE_HEIGHT       35
#define HUD_FUEL_FONT_SIZE              18
#define HUD_FUEL_LINE_HEIGHT            25
#define HUD_FUEL_BAR_WIDTH             240
#define HUD_FUEL_BAR_HEIGHT             15
#define HUD_FUEL_BAR_BORDER_ALPHA       0.3f
#define HUD_FUEL_LOW_THRESHOLD          25.0f
#define HUD_FUEL_MED_THRESHOLD          50.0f
#define HUD_FUEL_BLINK_RATE              4
#define HUD_FUEL_STALL_FONT_SIZE        14
#define HUD_FUEL_STALL_X_OFFSET         40
#define HUD_FUEL_STALL_Y_OFFSET         20
#define HUD_SHADOW_OFFSET                2
#define HUD_BG_DARK_R                   30
#define HUD_BG_DARK_G                   30
#define HUD_BG_DARK_B                   30
#define HUD_BG_MED_R                    40
#define HUD_BG_MED_G                    40
#define HUD_BG_MED_B                    40








class NavigationGraph;
class MPCController;

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

    [[nodiscard]] float& Get_lockDistanceSq() {
        return m_lockDistanceSq;
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
    float m_lockDistanceSq = 490000.0f;

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