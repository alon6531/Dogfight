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

    m_mpc = std::make_unique<MPCController>(0, 0);

   ChangeAIState(AIStateType::TAKEOFF, graph);

}

Plane::~Plane() {
    UnloadModel(m_model);
}

void Plane::Update(float deltaTime, NavigationGraph& graph, const Map& map, const std::vector<Obstacle>& obstacles) {
    UpdateLockSystem(deltaTime);

    // --- 1. MPC STEERING CALCULATION ---
    if (m_mpc && m_currentStateType != AIStateType::IDLE) {
        // We calculate the best steering direction based on our current state
        // Target is determined by the FSM (the next point in the path)
        Vector3 mpcTarget = m_fsm->GetCurrentTargetFromAI();
        // Get the optimized direction from MPC
        Vector3 bestSteer = m_mpc->CalculateBestSteer(
            m_position,
            m_velocity,
            m_forward,
            mpcTarget,
            obstacles,
            m_enemy
        );

        // Smoothly rotate m_forward towards the best steer direction
        float rotationSpeed = 2.5f; // Adjust for "snappiness"
        m_forward = Vector3Normalize(Vector3Lerp(m_forward, bestSteer, rotationSpeed * deltaTime));
    }

    // Now physics will use the updated m_forward
    UpdatePhysics(deltaTime, map, obstacles);

    // --- 2. FSM UPDATE ---
    if (m_fsm) {
        AIStateType nextState = m_fsm->Update(deltaTime);

        if (nextState != m_currentStateType) {
            ChangeAIState(nextState, graph);
        }
    }

    // --- 3. FUEL LOGIC ---
    if (m_currentStateType != AIStateType::FUEL && (m_fuel <= ESCAPE_FUEL && m_fuel >= ESCAPE_FUEL - 30.0f)) {
        m_targetPos = m_liftPoint;
        ChangeAIState(AIStateType::PATROL, graph);
        std::cout << "Low Fuel! Returning to Lift-Off Point for approach." << std::endl;
    }
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

void Plane::UpdatePhysics(float deltaTime, const Map& map, const std::vector<Obstacle>& obstacles) {

    if (m_fuel >= 0)
        m_fuel -= m_thrust * deltaTime;

    float vx = m_velocity.x, vy = m_velocity.y, vz = m_velocity.z;
    float speedSq = vx*vx + vy*vy + vz*vz;
    float currentSpeed = sqrtf(speedSq);

    bool hasPath = (m_fsm != nullptr);
    float throttle = hasPath ? 1.0f : 0.0f;

    m_thrust =  throttle * (m_currentStateType == AIStateType::EVASION ? MAX_THRUST_EVASION :  MAX_THRUST + (fmaxf(0.0f, m_forward.y) * CLIMB_EXTRA_POWER));
    m_drug = speedSq * DRAG_COEFF;
    m_lift = currentSpeed * 0.85f;


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

    if (m_targetLock.lockTimer <= 0) m_targetLock.finalLock = true;

    Vector3 selfPos = m_position;
    Vector3 enemyPos = m_enemy->GetPosition();
    Vector3 enemyVel = m_enemy->GetVelocity();


    float distToEnemy = Vector3Distance(selfPos, enemyPos);
    float predictionTime = distToEnemy * 0.002f;
    Vector3 predictedPos = Vector3Add(enemyPos, Vector3Scale(enemyVel, predictionTime));


    Vector3 dirToTarget = Vector3Normalize(Vector3Subtract(predictedPos, selfPos));
    float alignment = Vector3DotProduct(m_forward, dirToTarget);


    const float MAX_LOCK_DIST = 800.0f;
    const float LOCK_CONE = 0.98f;
    const float LOCK_SPEED = 0.25f;
    const float HEIGHT_ADVANTAGE_THRESHOLD = 30.0f;


    bool isHighEnough = (selfPos.y > enemyPos.y + HEIGHT_ADVANTAGE_THRESHOLD);


    bool canLock = (distToEnemy < MAX_LOCK_DIST) &&
                   (alignment > LOCK_CONE) &&
                   isHighEnough &&
                   (m_currentStateType == AIStateType::PURSUIT);

    if (canLock) {
        m_targetLock.lockProgress += LOCK_SPEED * deltaTime;
        if (m_targetLock.lockProgress >= 1.0f) {
            m_targetLock.lockProgress = 1.0f;
            m_targetLock.isLocked = true;
        }
    } else {

        m_targetLock.lockProgress -= deltaTime * 0.8f;
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
    if (!m_enemy) return;

    Vector3 selfPos = m_position;
    Vector3 enemyPos = m_enemy->GetPosition();

    Vector3 dirToEnemy = Vector3Normalize(Vector3Subtract(enemyPos, selfPos));
    float alignment = Vector3DotProduct(m_forward, dirToEnemy);

    if (alignment < 0) return;

    Vector2 screenPos = GetWorldToScreen(enemyPos, camera);

    if (screenPos.x > 0 && screenPos.x < GetScreenWidth() &&
        screenPos.y > 0 && screenPos.y < GetScreenHeight())
    {
        bool lowHeight = selfPos.y < enemyPos.y + 30.0f;
        Color lockColor = m_targetLock.isLocked ? RED : LIME;

        if (lowHeight && !m_targetLock.isLocked) lockColor = ORANGE;

        float boxSize = m_targetLock.isLocked ? 40.0f : 60.0f;

        DrawRectangleLinesEx(Rectangle{ screenPos.x - boxSize/2, screenPos.y - boxSize/2, boxSize, boxSize }, 2, lockColor);

        if (m_targetLock.isLocked) {

            int fontSize = 20;
            int textWidth = MeasureText("LOCK", fontSize);
            DrawText("LOCK", screenPos.x - textWidth / 2, screenPos.y - boxSize / 2 - 25, fontSize, RED);

            float dist = Vector3Distance(selfPos, enemyPos);
            DrawText(TextFormat("%.0fm", dist), screenPos.x - 20, screenPos.y + boxSize / 2 + 5, 15, RED);
        }

        if (lowHeight && !m_targetLock.isLocked) {
            DrawText("GAIN ALTITUDE", screenPos.x - 45, screenPos.y + (boxSize / 2 + 15), 10, ORANGE);
        }

        if (!m_targetLock.isLocked && m_targetLock.lockProgress > 0) {
            DrawRectangle(screenPos.x - 30, screenPos.y + 35, 60 * m_targetLock.lockProgress, 4, LIME);
        }
    }
}


void Plane::DrawHub(bool showDebug) const {


    int panelX = 15, panelY = 15;
    int panelWidth = 280, panelHeight = 400;

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
    DrawStat("Ammo: ", m_emmoCount, YELLOW);


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


    if (showDebug) {
        if (m_fsm) m_fsm->DrawDebugUI();

    }


}
