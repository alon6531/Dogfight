#include "Plane.h"
#include <iostream>

#include "raymath.h"
#include "../../World/Map.h"
#include "AIState/Type/EvasionState.h"
#include "AIState/Type/PatrolState.h"
#include "AIState/Type/PursuitState.h"
#include "AIState/Type/TakeOffState.h"
#include "../../Collision/GJK.h"
#include "AIState/Type/FuelState.h"


Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Color &color, NavigationGraph& graph,
             const Vector3 &targetPos)
    : m_position(position), m_velocity(velocity), m_targetPos(targetPos), m_basePos(position)
{
    m_forward = { 1, 0, 0 };
    // m_mesh = GenMeshCube(200, 20, 50);
    // m_model = LoadModelFromMesh(m_mesh);
    // m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = color;
    m_model = LoadModel("Assets/plane.glb");

    for (int i = 0; i < m_model.materialCount; i++) {
        auto &mat = m_model.materials[i];
        mat.maps[MATERIAL_MAP_ALBEDO].color = color;
    }

    m_fuel = MAX_FUEL;

    m_mpc = std::make_unique<MPCController>(10, 3);

   ChangeAIState(AIStateType::TAKEOFF, graph);

}

Plane::~Plane() {
    UnloadModel(m_model);
}

void Plane::Update(float deltaTime, NavigationGraph& graph, Map& map, std::vector<Obstacle>& obstacles) {
    if (m_fsm) {
        AIStateType nextState = m_fsm->Update(deltaTime);

        static float stateChangeTimer = 0.0f;
        stateChangeTimer += deltaTime;

        if (m_currentStateType == AIStateType::PURSUIT) {
            Vector3 dirToMe = Vector3Normalize(Vector3Subtract(m_position, m_enemy->GetPosition()));
            float enemyAlignment = Vector3DotProduct(m_enemy->GetForward(), dirToMe);
            float distToEnemy = Vector3Distance(m_position, m_enemy->GetPosition());

            if (enemyAlignment > 0.9f && distToEnemy < 500.0f && stateChangeTimer > 3.0f) {
                nextState = AIStateType::EVASION;
            }
        }

        if (nextState != m_currentStateType) {
            ChangeAIState(nextState, graph);
            stateChangeTimer = 0.0f;
        }
    }

    if (m_fuel <= ESCAPE_FUEL && m_fuel >= ESCAPE_FUEL - 30) {
        m_targetPos = m_basePos;
        ChangeAIState(AIStateType::PATROL, graph);
    }

    UpdateLockSystem(deltaTime);

    UpdatePhysics(deltaTime, map, obstacles);
}

void Plane::ChangeAIState(AIStateType newState, NavigationGraph& graph) {
    m_currentStateType = newState;

    switch (newState) {
        case AIStateType::IDLE:
            // m_fsm = std::make_unique<IdleState>(this, m_enemy);
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

void Plane::UpdatePhysics(float deltaTime, Map& map, std::vector<Obstacle>& obstacles) {
    m_gravity = GRAVITY * MASS;
    float currentSpeed = Vector3Length(m_velocity);
    float throttle = (m_fsm && !m_fsm->GetPath().empty()) ? 1.0f : 0.0f;

    float speedMultiplier = 1.0f;
    float dragReducer = 1.0f;
    if (m_currentStateType == AIStateType::EVASION) {
        speedMultiplier = 1.5f;
        dragReducer = 0.5f;
    }
    float climbBoost = fmaxf(0.0f, m_forward.y) * CLIMB_EXTRA_POWER;
    m_thrust = throttle * (MAX_THRUST + climbBoost) * speedMultiplier;
    m_drug = (currentSpeed * currentSpeed) * DRAG_COEFF * dragReducer;
    m_lift = currentSpeed * 0.85f;


    Vector3 avoidanceDir = { 0, 0, 0 };
    for (const auto& obs : obstacles) {
        float dist = Vector3Distance(m_position, obs.pos);
        float safeZone = obs.radius + 50.0f;

        if (dist < safeZone) {
            Vector3 push = Vector3Normalize(Vector3Subtract(m_position, obs.pos));
            float weight = (1.0f - (dist / safeZone));
            avoidanceDir = Vector3Add(avoidanceDir, Vector3Scale(push, weight));
        }
    }
    if (Vector3LengthSqr(avoidanceDir) > 0.001f) {
        Vector3 newTargetDir = Vector3Normalize(Vector3Add(m_forward, avoidanceDir));
        float avoidanceAgility = 3.0f;
        m_forward = Vector3Normalize(Vector3Lerp(m_forward, newTargetDir, avoidanceAgility * deltaTime));
    }


    float rotationAgility = 2.0f;


    Vector3 targetDir = Vector3Normalize(Vector3Subtract(m_targetPos, m_position));
    Vector3 right = Vector3CrossProduct(m_forward, {0, 1, 0});
    float turnAmount = Vector3DotProduct(targetDir, right);


    float targetBank = turnAmount * 45.0f;


    m_bankAngle = Lerp(m_bankAngle, targetBank, rotationAgility * deltaTime);

    const Vector3 currentTarget = m_fsm->GetCurrentTargetFromAI();


    const Vector3 mpcSteerDir = m_mpc->CalculateBestSteer(m_position, m_velocity, m_forward, currentTarget, obstacles, m_enemy);


    const float agility = 2.0f;
    m_forward = Vector3Normalize(Vector3Lerp(m_forward, mpcSteerDir, agility * deltaTime));


    float pitchInertia = -m_forward.y * SLOPE_EFFECT;
    float totalAcceleration = (m_thrust - m_drug) + pitchInertia;
    float newForwardSpeed = currentSpeed + (totalAcceleration * deltaTime);
    if (newForwardSpeed < 0.1f) newForwardSpeed = 0.1f;

    float verticalNetForce = m_lift - m_gravity;
    m_velocity = Vector3Scale(m_forward, newForwardSpeed);
    m_velocity.y += verticalNetForce * deltaTime;


    float groundHeight = map.GetHeightAt(m_position.x, m_position.z);
    float safeDistance = 5.0f;
    float distanceToGround = m_position.y - groundHeight;

    if (distanceToGround <= safeDistance) {
        float proximityFactor = 1.0f - (distanceToGround / safeDistance);
        proximityFactor = Clamp(proximityFactor, 0.0f, 1.0f);

        m_normal = proximityFactor * m_gravity;
        m_velocity.y += (m_normal / MASS) * deltaTime;

        if (m_position.y < groundHeight + 0.0f) {
            m_position.y = groundHeight + 0.0f;
            if (m_velocity.y < 0) m_velocity.y = 0;
        }

        if (m_thrust < 10) {
            m_velocity = Vector3Scale(m_velocity, 1.0f - (0.05f * proximityFactor));
        }
    }
    else {
        m_normal = 0;
    }

    m_fuel -= m_thrust * 0.01;

    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));
}

void Plane::SteerTowards(Vector3 target, float deltaTime) {
    Vector3 targetDir = Vector3Normalize(Vector3Subtract(target, m_position));


    float rotationSpeed = 2.0f;

    m_forward = Vector3Normalize(Vector3Lerp(m_forward, targetDir, rotationSpeed * deltaTime));
}

void Plane::UpdateLockSystem(float deltaTime) {

    if (!m_enemy) {
        m_targetLock.isLocked = false;
        m_targetLock.lockProgress = 0.0f;
        return;
    }

    if (m_targetLock.lockTimer <= 0)
        m_targetLock.finalLock = true;


    Vector3 selfPos = m_position;
    Vector3 enemyPos = m_enemy->GetPosition();
    Vector3 dirToEnemy = Vector3Normalize(Vector3Subtract(enemyPos, selfPos));


    float alignment = Vector3DotProduct(m_forward, dirToEnemy);
    float distToEnemy = Vector3Distance(selfPos, enemyPos);


    const float MAX_LOCK_DIST = 600.0f;
    const float LOCK_CONE = 0.97f;
    const float LOCK_SPEED = 0.3f;


    if (distToEnemy < MAX_LOCK_DIST && alignment > LOCK_CONE) {

        m_targetLock.lockProgress += LOCK_SPEED * deltaTime;

        if (m_targetLock.lockProgress >= 1.0f) {
            m_targetLock.lockProgress = 1.0f;
            m_targetLock.isLocked = true;
        }
    } else {

        m_targetLock.lockProgress -= deltaTime * 1.5f;
        if (m_targetLock.lockProgress <= 0.0f) {
            m_targetLock.lockProgress = 0.0f;
            m_targetLock.isLocked = false;
        }
    }
    if (m_targetLock.isLocked) {
        m_targetLock.lockTimer -= deltaTime;
    }
}


void Plane::Draw() {
    Vector3 up = { 0, 1, 0 };


    Matrix worldMat = MatrixLookAt(m_position, Vector3Add(m_position, m_forward), up);
    worldMat = MatrixInvert(worldMat);


    Matrix modelCorrection = MatrixRotateY(DEG2RAD * 90);
    modelCorrection = MatrixMultiply(MatrixRotateX(DEG2RAD * 90), modelCorrection);


    Matrix tiltMat = MatrixRotateZ(DEG2RAD * m_bankAngle);


    Matrix finalTransform = MatrixScale(0.1f, 0.1f, 0.1f);
    finalTransform = MatrixMultiply(finalTransform, modelCorrection);
    finalTransform = MatrixMultiply(finalTransform, tiltMat);
    finalTransform = MatrixMultiply(finalTransform, worldMat);

    m_model.transform = finalTransform;
    DrawModel(m_model, { 0, 0, 0 }, 1.0f, WHITE);

    DrawPath();
    DrawForceVectors();
}


void Plane::DrawForceVectors() const {
    float visualScale = 0.5f;
    float headSize = 0.5f;


    auto DrawVector = [&](Vector3 direction, float magnitude, Color col) {
        if (magnitude <= 0.1f) return;
        Vector3 endPos = Vector3Add(m_position, Vector3Scale(direction, magnitude * visualScale));
        DrawLine3D(m_position, endPos, col);
        DrawSphere(endPos, headSize, col);
    };




    DrawVector(m_forward, m_thrust, ORANGE);


    DrawVector(Vector3Negate(m_forward), m_drug, RED);


    DrawVector({0, 1, 0}, m_lift, SKYBLUE);


    DrawVector({0, -1, 0}, m_gravity, WHITE);


    Vector3 velDir = Vector3Normalize(m_velocity);
    float velMag = Vector3Length(m_velocity);
    DrawVector(velDir, velMag, GOLD);
}




void Plane::DrawPath() const {
    if (!m_fsm) return;
    const auto& path = m_fsm->GetPath();
    if (path.empty()) return;


    DrawLine3D(m_position, path[0], LIME);

    for (size_t i = 0; i < path.size(); i++) {



        if (i < path.size() - 1) {
            DrawLine3D(path[i], path[i+1], GREEN);
        }
    }
}



void Plane::DrawLocked(Camera3D camera) const {
    if (m_enemy) {

        Vector2 screenPos = GetWorldToScreen(m_enemy->GetPosition(), camera);


        if (screenPos.x > 0 && screenPos.y > 0) {


            Color lockColor = m_targetLock.isLocked ? RED : Fade(LIME, 0.5f);
            float boxSize = m_targetLock.isLocked ? 40.0f : 60.0f; // הריבוע מתכווץ כשננעל


            DrawRectangleLinesEx(Rectangle{
                screenPos.x - boxSize/2,
                screenPos.y - boxSize/2,
                boxSize,
                boxSize
            }, 2, lockColor);


            if (m_targetLock.isLocked) {
                float dist = Vector3Distance(m_position, m_enemy->GetPosition());
                DrawText("LOCK", screenPos.x - 15, screenPos.y - (boxSize/2 + 15), 10, RED);
                DrawText(TextFormat("%.0fm", dist), screenPos.x - 15, screenPos.y + (boxSize/2 + 5), 10, RED);
            }


            if (!m_targetLock.isLocked && m_targetLock.lockProgress > 0) {
                DrawRectangle(screenPos.x - 30, screenPos.y + 35, 60 * m_targetLock.lockProgress, 4, LIME);
            }
        }
    }
}


void Plane::DrawHub() const {
    int panelX = 15, panelY = 15;
    int panelWidth = 280, panelHeight = 350;

    auto DrawTextWithShadow = [](const char* text, int posX, int posY, int size, Color col) {
        DrawText(text, posX + 2, posY + 2, size, BLACK);
        DrawText(text, posX, posY, size, col);
    };

    DrawRectangle(panelX, panelY, panelWidth, panelHeight, Fade(BLACK, 0.7f));
    DrawRectangleLinesEx(Rectangle{(float)panelX, (float)panelY, (float)panelWidth, (float)panelHeight}, 2, Fade(SKYBLUE, 0.5f));

    int x = panelX + 20, y = panelY + 20;
    DrawTextWithShadow("FLIGHT COMPUTER", x, y, 16, GRAY);
    y += 25;


    const char* stateName = "UNKNOWN";
    Color stateColor = WHITE;

    switch (m_currentStateType) {
        case AIStateType::TAKEOFF:
            stateName = "TAKEOFF";
            stateColor = ORANGE;
            break;
        case AIStateType::PATROL:
            stateName = "PATROL";
            stateColor = LIME;
            break;
        case AIStateType::PURSUIT:
            stateName = "PURSUIT";
            stateColor = RED;
            break;
        case AIStateType::IDLE:
            stateName = "IDLE";
            stateColor = LIGHTGRAY;
            break;
        case AIStateType::EVASION:
            stateName = "EVASION";
            stateColor = WHITE;
            break;
        case AIStateType::FUEL:
            stateName = "FUEL";
            stateColor = LIGHTGRAY;
            break;
        default:
            stateName = "OTHER";
            stateColor = WHITE;
            break;
    }
    DrawTextWithShadow("MODE:", x, y, 22, WHITE);
    DrawTextWithShadow(stateName, x + 85, y, 22, stateColor);
    y += 45;

    DrawTextWithShadow("LIVE FORCE VECTORS", x, y, 14, SKYBLUE);
    y += 30;

    auto DrawStat = [&](const char* label, float value, Color col) {
        DrawTextWithShadow(label, x, y, 18, col); // הכיתוב בצבע של הוקטור
        DrawTextWithShadow(TextFormat("%7.1f", value), x + 120, y, 18, WHITE);
        y += 25;
    };


    DrawStat("THRUST [T]:", m_thrust, ORANGE);
    DrawStat("DRAG   [D]:", m_drug, RED);
    DrawStat("LIFT   [L]:", m_lift, SKYBLUE);
    DrawStat("GRAVITY[G]:", m_gravity, WHITE);
    DrawStat("NET Y  [N]:", m_normal, MAGENTA);

    y += 15;
    float speed = Vector3Length(m_velocity);
    DrawTextWithShadow(TextFormat("AIRSPEED: %.1f m/s", speed), x, y, 22, GOLD);


    y += 30;
    DrawRectangle(x, y, 240, 10, Color{ 30, 30, 30, 255 });
    float speedWidth = Clamp(speed * 2.0f, 0, 240);
    DrawRectangle(x, y, (int)speedWidth, 10, GOLD);

    y += 35;
    float fuelPerc = (m_fuel / MAX_FUEL) * 100.0f;


    Color fuelColor = LIME;
    if (fuelPerc < 25.0f) {

        fuelColor = ((int)(GetTime() * 4) % 2 == 0) ? RED : MAROON;
    } else if (fuelPerc < 50.0f) {
        fuelColor = ORANGE;
    }

    DrawTextWithShadow(TextFormat("FUEL: %.1f%%", fuelPerc), x, y, 18, fuelColor);

    y += 25;

    DrawRectangle(x, y, 240, 15, Color{ 40, 40, 40, 255 });

    float fuelBarWidth = Clamp((fuelPerc / 100.0f) * 240.0f, 0, 240);
    DrawRectangle(x, y, (int)fuelBarWidth, 15, fuelColor);

    DrawRectangleLines(x, y, 240, 15, Fade(WHITE, 0.3f));

    if (fuelPerc <= 0.0f) {
        DrawTextWithShadow("ENGINE STALL - NO FUEL", x + 40, y + 20, 14, RED);
    }

}
