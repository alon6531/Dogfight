#include "Plane.h"
#include <iostream>

#include "raymath.h"
#include "rlImGui.h"
#include "../../World/Map.h"
#include "AIState/Type/EvasionState.h"
#include "AIState/Type/PatrolState.h"
#include "AIState/Type/PursuitState.h"
#include "AIState/Type/TakeOffState.h"
#include "../../Collision/GJK.h"
#include "AIState/Type/FuelState.h"
#include "Control/MPCController.h"



Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Color &color, NavigationGraph& graph,
             const Vector3 &targetPos)
    : m_position(position), m_velocity(velocity), m_targetPos(targetPos), m_basePos(position)
{
    m_forward = PLANE_FORWARD_DEFAULT;
    m_model = LoadModel(PLANE_MODEL_PATH);

    for (int i = 0; i < m_model.materialCount; i++) {
        auto &mat = m_model.materials[i];
        mat.maps[MATERIAL_MAP_ALBEDO].color = color;
    }

    m_fuel = MAX_FUEL;
    m_mpc = std::make_unique<MPCController>(PLANE_MPC_HORIZON, PLANE_MPC_DT);

    ChangeAIState(AIStateType::TAKEOFF, graph);
}

Plane::~Plane() {
    UnloadModel(m_model);
}

void Plane::Update(float deltaTime, NavigationGraph& graph, const Map& map, const std::vector<Obstacle>& obstacles) {
    UpdateLockSystem(deltaTime);

    if (m_mpc && m_currentStateType != AIStateType::IDLE) {
        Vector3 mpcTarget = m_fsm->GetCurrentTargetFromAI();
        Vector3 bestSteer = m_mpc->CalculateBestSteer(
            m_position,
            m_velocity,
            m_forward,
            mpcTarget,
            obstacles,
            m_enemy
        );

        m_forward = Vector3Normalize(Vector3Lerp(m_forward, bestSteer, PLANE_ROTATION_SPEED * deltaTime));
    }

    UpdatePhysics(deltaTime, map, obstacles);

    if (m_fsm) {
        AIStateType nextState = m_fsm->Update(deltaTime);
        if (nextState != m_currentStateType) {
            ChangeAIState(nextState, graph);
        }
    }

    if (m_currentStateType != AIStateType::FUEL &&
        (m_fuel <= ESCAPE_FUEL && m_fuel >= ESCAPE_FUEL - PLANE_FUEL_LOW_WINDOW)) {
        m_targetPos = m_liftPoint;
        ChangeAIState(AIStateType::PATROL, graph);
        std::cout << "Low Fuel! Returning to Lift-Off Point for approach." << std::endl;
    }
}

void Plane::ChangeAIState(AIStateType newState, NavigationGraph& graph) {
    m_currentStateType = newState;

    switch (newState) {
        case AIStateType::IDLE:
            break;
        case AIStateType::TAKEOFF:
            m_fsm = std::make_unique<TakeOffState>(*this, graph);
            break;
        case AIStateType::PATROL:
            m_fsm = std::make_unique<PatrolState>(*this, graph, m_targetPos);
            break;
        case AIStateType::PURSUIT:
            m_fsm = std::make_unique<PursuitState>(*this, graph, m_enemy);
            break;
        case AIStateType::EVASION:
            m_fsm = std::make_unique<EvasionState>(*this, graph, m_enemy);
            break;
        case AIStateType::FUEL:
            m_fsm = std::make_unique<FuelState>(*this, graph);
            break;
    }
}

void Plane::UpdatePhysics(float deltaTime, const Map& map, const std::vector<Obstacle>& obstacles) {
    if (m_fuel >= 0)
        m_fuel -= m_thrust * deltaTime;

    float vx = m_velocity.x, vy = m_velocity.y, vz = m_velocity.z;
    float speedSq = vx*vx + vy*vy + vz*vz;
    float currentSpeed = sqrtf(speedSq);

    bool hasPath = (m_fsm != nullptr);
    float throttle = hasPath ? 1.0f : 0.0f;

    m_thrust = throttle * (m_currentStateType == AIStateType::EVASION
                           ? MAX_THRUST_EVASION
                           : MAX_THRUST + (fmaxf(0.0f, m_forward.y) * CLIMB_EXTRA_POWER));
    m_drug = speedSq * DRAG_COEFF;
    m_lift = currentSpeed * PLANE_LIFT_COEFF;

    float totalAcc = (m_thrust - m_drug) + (-m_forward.y * SLOPE_EFFECT);
    float newSpeed = fmaxf(0.1f, currentSpeed + (totalAcc * deltaTime));

    m_velocity.x = m_forward.x * newSpeed;
    m_velocity.y = m_forward.y * newSpeed + (m_lift - (GRAVITY * MASS)) * deltaTime;
    m_velocity.z = m_forward.z * newSpeed;

    if (m_position.y < 0.0f) {
        CheckGroundCollision(map, deltaTime);
    }

    m_position.x += m_velocity.x * deltaTime;
    m_position.y += m_velocity.y * deltaTime;
    m_position.z += m_velocity.z * deltaTime;
}

void Plane::CheckGroundCollision(const Map& map, float deltaTime) {
    float groundHeight    = map.GetHeightAt(m_position.x, m_position.z);
    float distanceToGround = m_position.y - groundHeight;

    if (distanceToGround <= PLANE_GROUND_SAFE_DIST) {
        float proximityFactor = 1.0f - (distanceToGround / PLANE_GROUND_SAFE_DIST);
        proximityFactor = Clamp(proximityFactor, 0.0f, 1.0f);

        m_normal = proximityFactor * m_gravity;
        m_velocity.y += (m_normal / MASS) * deltaTime;

        if (m_position.y < groundHeight + PLANE_GROUND_SNAP_OFFSET) {
            m_position.y = groundHeight + PLANE_GROUND_SNAP_OFFSET;
            if (m_velocity.y < 0) m_velocity.y = 0;
        }

        if (m_thrust < PLANE_GROUND_FRICTION_THRUST) {
            m_velocity = Vector3Scale(m_velocity, 1.0f - (PLANE_GROUND_FRICTION_SCALE * proximityFactor));
        }
    } else {
        m_normal = 0;
    }
}

void Plane::SteerTowards(Vector3 target, float deltaTime) {
    Vector3 targetDir = Vector3Normalize(Vector3Subtract(target, m_position));
    m_forward = Vector3Normalize(Vector3Lerp(m_forward, targetDir, PLANE_STEER_ROTATION_SPEED * deltaTime));
}

void Plane::UpdateLockSystem(float deltaTime) {
    if (!m_enemy) {
        m_targetLock.isLocked     = false;
        m_targetLock.lockProgress = 0.0f;
        return;
    }

    if (m_targetLock.lockTimer <= 0) m_targetLock.finalLock = true;

    Vector3 selfPos  = m_position;
    Vector3 enemyPos = m_enemy->GetPosition();
    Vector3 enemyVel = m_enemy->GetVelocity();

    float distToEnemy    = Vector3Distance(selfPos, enemyPos);
    float predictionTime = distToEnemy * LOCK_PREDICTION_SCALE;
    Vector3 predictedPos = Vector3Add(enemyPos, Vector3Scale(enemyVel, predictionTime));

    Vector3 dirToTarget = Vector3Normalize(Vector3Subtract(predictedPos, selfPos));
    float alignment     = Vector3DotProduct(m_forward, dirToTarget);

    bool isHighEnough = (selfPos.y > enemyPos.y + LOCK_HEIGHT_ADVANTAGE);
    bool canLock = (distToEnemy < LOCK_MAX_DIST) &&
                   (alignment > LOCK_CONE) &&
                   isHighEnough &&
                   (m_currentStateType == AIStateType::PURSUIT);

    if (canLock) {
        m_targetLock.lockProgress += LOCK_SPEED * deltaTime;
        if (m_targetLock.lockProgress >= 1.0f) {
            m_targetLock.lockProgress = 1.0f;
            m_targetLock.isLocked     = true;
        }
    } else {
        m_targetLock.lockProgress -= deltaTime * LOCK_DECAY_SPEED;
        if (m_targetLock.lockProgress <= 0.0f) {
            m_targetLock.lockProgress = 0.0f;
            m_targetLock.isLocked     = false;
        }
    }

    if (m_targetLock.isLocked) {
        m_targetLock.lockTimer -= deltaTime;
    }
}

void Plane::Draw() {
    Vector3 up = { 0, 1, 0 };

    Matrix worldMat      = MatrixLookAt(m_position, Vector3Add(m_position, m_forward), up);
    worldMat             = MatrixInvert(worldMat);

    Matrix modelCorrection = MatrixRotateY(DEG2RAD * PLANE_CORRECTION_ROT_Y);
    modelCorrection        = MatrixMultiply(MatrixRotateX(DEG2RAD * PLANE_CORRECTION_ROT_X), modelCorrection);

    Matrix tiltMat = MatrixRotateZ(DEG2RAD * m_bankAngle);

    Matrix finalTransform = MatrixScale(PLANE_SCALE, PLANE_SCALE, PLANE_SCALE);
    finalTransform = MatrixMultiply(finalTransform, modelCorrection);
    finalTransform = MatrixMultiply(finalTransform, tiltMat);
    finalTransform = MatrixMultiply(finalTransform, worldMat);

    m_model.transform = finalTransform;
    DrawModel(m_model, { 0, 0, 0 }, 1.0f, WHITE);

    DrawPath();
    DrawForceVectors();
}

void Plane::DrawForceVectors() const {
    auto DrawVector = [&](Vector3 direction, float magnitude, Color col) {
        if (magnitude <= FORCE_MIN_MAGNITUDE) return;
        Vector3 endPos = Vector3Add(m_position, Vector3Scale(direction, magnitude * FORCE_VISUAL_SCALE));
        DrawLine3D(m_position, endPos, col);
        DrawSphere(endPos, FORCE_HEAD_SIZE, col);
    };

    DrawVector(m_forward,              m_thrust,  ORANGE);
    DrawVector(Vector3Negate(m_forward), m_drug,  RED);
    DrawVector({0,  1, 0},             m_lift,    SKYBLUE);
    DrawVector({0, -1, 0},             m_gravity, WHITE);

    Vector3 velDir = Vector3Normalize(m_velocity);
    float   velMag = Vector3Length(m_velocity);
    DrawVector(velDir, velMag, GOLD);
}

void Plane::DrawPath() const {
    if (!m_fsm) return;
    const auto& path = m_fsm->GetPath();
    if (path.empty()) return;

    DrawLine3D(m_position, path[0], LIME);

    for (size_t i = 0; i < path.size(); i++) {
        if (i < path.size() - 1) {
            DrawLine3D(path[i], path[i + 1], GREEN);
        }
    }
}

void Plane::DrawLocked(Camera3D camera) const {
    if (!m_enemy) return;

    Vector3 selfPos  = m_position;
    Vector3 enemyPos = m_enemy->GetPosition();

    Vector3 dirToEnemy = Vector3Normalize(Vector3Subtract(enemyPos, selfPos));
    float alignment    = Vector3DotProduct(m_forward, dirToEnemy);

    if (alignment < 0) return;

    Vector2 screenPos = GetWorldToScreen(enemyPos, camera);

    if (screenPos.x > LOCK_SCREEN_MARGIN && screenPos.x < GetScreenWidth()  &&
        screenPos.y > LOCK_SCREEN_MARGIN && screenPos.y < GetScreenHeight())
    {
        bool lowHeight = selfPos.y < enemyPos.y + LOCK_LOW_HEIGHT_THRESHOLD;
        Color lockColor = m_targetLock.isLocked ? RED : LIME;

        if (lowHeight && !m_targetLock.isLocked) lockColor = ORANGE;

        float boxSize = m_targetLock.isLocked ? LOCK_BOX_SIZE_LOCKED : LOCK_BOX_SIZE_UNLOCKED;

        DrawRectangleLinesEx(
            Rectangle{ screenPos.x - boxSize / 2, screenPos.y - boxSize / 2, boxSize, boxSize },
            LOCK_BOX_BORDER, lockColor);

        if (m_targetLock.isLocked) {
            int textWidth = MeasureText("LOCK", LOCK_TEXT_FONT_SIZE);
            DrawText("LOCK",
                     screenPos.x - textWidth / 2,
                     screenPos.y - boxSize / 2 + LOCK_TEXT_Y_OFFSET,
                     LOCK_TEXT_FONT_SIZE, RED);

            float dist = Vector3Distance(selfPos, enemyPos);
            DrawText(TextFormat("%.0fm", dist),
                     screenPos.x - HUD_INNER_MARGIN,
                     screenPos.y + boxSize / 2 + LOCK_DIST_Y_OFFSET,
                     LOCK_DIST_FONT_SIZE, RED);
        }

        if (lowHeight && !m_targetLock.isLocked) {
            DrawText("GAIN ALTITUDE",
                     screenPos.x + LOCK_ALT_X_OFFSET,
                     screenPos.y + (boxSize / 2 + LOCK_ALT_TEXT_Y_OFFSET),
                     LOCK_ALT_FONT_SIZE, ORANGE);
        }

        if (!m_targetLock.isLocked && m_targetLock.lockProgress > 0) {
            DrawRectangle(
                screenPos.x + LOCK_PROGRESS_BAR_X_OFFSET,
                screenPos.y + LOCK_PROGRESS_BAR_Y_OFFSET,
                LOCK_PROGRESS_BAR_WIDTH * m_targetLock.lockProgress,
                LOCK_PROGRESS_BAR_HEIGHT, LIME);
        }
    }
}

void Plane::DrawHub(bool showDebug) const {
    int panelX = HUD_PANEL_X, panelY = HUD_PANEL_Y;

    auto DrawTextWithShadow = [](const char* text, int posX, int posY, int size, Color col) {
        DrawText(text, posX + HUD_SHADOW_OFFSET, posY + HUD_SHADOW_OFFSET, size, BLACK);
        DrawText(text, posX, posY, size, col);
    };

    DrawRectangle(panelX, panelY, HUD_PANEL_WIDTH, HUD_PANEL_HEIGHT, Fade(BLACK, HUD_BG_ALPHA));
    DrawRectangleLinesEx(
        Rectangle{ (float)panelX, (float)panelY, (float)HUD_PANEL_WIDTH, (float)HUD_PANEL_HEIGHT },
        HUD_BORDER_THICKNESS, Fade(SKYBLUE, HUD_BORDER_ALPHA));

    int x = panelX + HUD_INNER_MARGIN;
    int y = panelY + HUD_INNER_MARGIN;

    DrawTextWithShadow("FLIGHT COMPUTER", x, y, HUD_TITLE_FONT_SIZE, GRAY);
    y += HUD_TITLE_LINE_HEIGHT;

    const char* stateName = "UNKNOWN";
    Color stateColor = WHITE;

    switch (m_currentStateType) {
        case AIStateType::TAKEOFF:  stateName = "TAKEOFF";  stateColor = ORANGE;    break;
        case AIStateType::PATROL:   stateName = "PATROL";   stateColor = LIME;      break;
        case AIStateType::PURSUIT:  stateName = "PURSUIT";  stateColor = RED;       break;
        case AIStateType::IDLE:     stateName = "IDLE";     stateColor = LIGHTGRAY; break;
        case AIStateType::EVASION:  stateName = "EVASION";  stateColor = WHITE;     break;
        case AIStateType::FUEL:     stateName = "FUEL";     stateColor = LIGHTGRAY; break;
        default:                    stateName = "OTHER";    stateColor = WHITE;     break;
    }

    DrawTextWithShadow("MODE:", x, y, HUD_MODE_FONT_SIZE, WHITE);
    DrawTextWithShadow(stateName, x + HUD_MODE_LABEL_OFFSET, y, HUD_MODE_FONT_SIZE, stateColor);
    y += HUD_MODE_LINE_HEIGHT;

    DrawTextWithShadow("LIVE FORCE VECTORS", x, y, HUD_SECTION_FONT_SIZE, SKYBLUE);
    y += HUD_SECTION_LINE_HEIGHT;

    auto DrawStat = [&](const char* label, float value, Color col) {
        DrawTextWithShadow(label, x, y, HUD_STAT_FONT_SIZE, col);
        DrawTextWithShadow(TextFormat("%7.1f", value), x + HUD_STAT_VALUE_OFFSET, y, HUD_STAT_FONT_SIZE, WHITE);
        y += HUD_STAT_LINE_HEIGHT;
    };

    DrawStat("THRUST [T]:", m_thrust,    ORANGE);
    DrawStat("DRAG   [D]:", m_drug,      RED);
    DrawStat("LIFT   [L]:", m_lift,      SKYBLUE);
    DrawStat("GRAVITY[G]:", m_gravity,   WHITE);
    DrawStat("NET Y  [N]:", m_normal,    MAGENTA);
    DrawStat("Ammo: ",      m_emmoCount, YELLOW);

    y += HUD_STAT_GAP;

    float speed = Vector3Length(m_velocity);
    DrawTextWithShadow(TextFormat("AIRSPEED: %.1f m/s", speed), x, y, HUD_SPEED_FONT_SIZE, GOLD);

    y += HUD_SPEED_LINE_HEIGHT;
    DrawRectangle(x, y, HUD_SPEED_BAR_WIDTH, HUD_SPEED_BAR_HEIGHT,
                  Color{ HUD_BG_DARK_R, HUD_BG_DARK_G, HUD_BG_DARK_B, 255 });
    float speedWidth = Clamp(speed * HUD_SPEED_BAR_SCALE, 0, HUD_SPEED_BAR_WIDTH);
    DrawRectangle(x, y, (int)speedWidth, HUD_SPEED_BAR_HEIGHT, GOLD);

    y += HUD_SPEED_BAR_LINE_HEIGHT;

    float fuelPerc = (m_fuel / MAX_FUEL) * 100.0f;

    Color fuelColor = LIME;
    if (fuelPerc < HUD_FUEL_LOW_THRESHOLD) {
        fuelColor = ((int)(GetTime() * HUD_FUEL_BLINK_RATE) % 2 == 0) ? RED : MAROON;
    } else if (fuelPerc < HUD_FUEL_MED_THRESHOLD) {
        fuelColor = ORANGE;
    }

    DrawTextWithShadow(TextFormat("FUEL: %.1f%%", fuelPerc), x, y, HUD_FUEL_FONT_SIZE, fuelColor);
    y += HUD_FUEL_LINE_HEIGHT;

    DrawRectangle(x, y, HUD_FUEL_BAR_WIDTH, HUD_FUEL_BAR_HEIGHT,
                  Color{ HUD_BG_MED_R, HUD_BG_MED_G, HUD_BG_MED_B, 255 });
    float fuelBarWidth = Clamp((fuelPerc / 100.0f) * HUD_FUEL_BAR_WIDTH, 0, HUD_FUEL_BAR_WIDTH);
    DrawRectangle(x, y, (int)fuelBarWidth, HUD_FUEL_BAR_HEIGHT, fuelColor);
    DrawRectangleLines(x, y, HUD_FUEL_BAR_WIDTH, HUD_FUEL_BAR_HEIGHT, Fade(WHITE, HUD_FUEL_BAR_BORDER_ALPHA));

    if (fuelPerc <= 0.0f) {
        DrawTextWithShadow("ENGINE STALL - NO FUEL",
                           x + HUD_FUEL_STALL_X_OFFSET,
                           y + HUD_FUEL_STALL_Y_OFFSET,
                           HUD_FUEL_STALL_FONT_SIZE, RED);
    }

    if (showDebug) {
        if (m_fsm) m_fsm->DrawDebugUI();
    }
}