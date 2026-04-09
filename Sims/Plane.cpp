#include "Plane.h"
#include <iostream>
#include "DStarLite.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include "MPCController.h"



static const float WORLD_BOUND_LIMIT = 850.0f;
static const float EMERGENCY_RETURN_STRENGTH = 8.0f;

Plane::Plane(const Vector3 &pos, const Vector3 &vel, const Vector3 &accel, const Color& color, NavigationGraph &graph)
    : m_position(pos), m_velocity(vel), m_acceleration(accel), m_graph(graph),
      CurrentState(AIState::IDLE), CurrentEvent(AIEvent::NONE) {

    m_model = LoadModel("plane.glb");
    for (int i = 0; i < m_model.materialCount; i++) {
        m_model.materials[i].maps[MATERIAL_MAP_ALBEDO].color = color;
    }

    m_baseTransform = MatrixMultiply(MatrixRotateX(90.0f * DEG2RAD), MatrixRotateY(-90.0f * DEG2RAD));
    m_dstar = std::make_unique<DStarLite>(m_graph);
}

Plane::~Plane() {
    UnloadModel(m_model);
}

void Plane::SetDestinationViaAStar(int targetNodeIdx) {
    int startNodeIdx = m_graph.GetClosestNode(m_position);
    m_velocity = {0, 0, 0};

    if (startNodeIdx != -1) {
        auto newPath = m_graph.FindPathViaAStar(startNodeIdx, targetNodeIdx);
        m_path.assign(newPath.begin(), newPath.end());
        m_rotation = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
    }
}

void Plane::Update(Plane& enemy, float deltaTime, const MPCController &mpc) {
    deltaTime = fminf(deltaTime, 0.05f);
    m_delay += deltaTime;
    bool shouldUpdate = (m_delay >= UPDATE_PATH_DELAY);

    switch (CurrentState) {
        case AIState::PATROL:  UpdatePatrol(deltaTime); break;
        case AIState::PURSUIT: UpdatePursuit(enemy, deltaTime, shouldUpdate); break;
        case AIState::EVADE:   UpdateEvade(enemy.GetPosition(), enemy.GetVelocity(), deltaTime, shouldUpdate); break;
        default: break;
    }

    if (shouldUpdate) m_delay = 0.0f;
}

void Plane::UpdatePatrol(float deltaTime) {
    if (m_path.empty()) return;

    while (m_path.size() > 1) {

        if (Vector3Distance(m_path[0], m_position) < 35.0f) {
            m_path.pop_front();
        } else {
            break;
        }
    }

    if (m_path.size() == 1 && Vector3Distance(m_path[0], m_position) < 40.0f) {
        m_path.clear();
        return;
    }

    Vector3 steerDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));
    m_speed = CalculateEnergyVelocity(steerDir, deltaTime);

    m_velocity = Vector3Lerp(m_velocity, Vector3Scale(Vector3Normalize(steerDir), m_speed), 4.0f * deltaTime);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));
    UpdateRotationAndTilt(deltaTime);
}

void Plane::ApplyWorldBounds(float deltaTime) {
    if (fabsf(m_position.x) > WORLD_BOUND_LIMIT || fabsf(m_position.z) > WORLD_BOUND_LIMIT) {
        Vector3 toCenter = Vector3Normalize(Vector3Subtract({0, m_position.y, 0}, m_position));

        m_velocity = Vector3Lerp(m_velocity, Vector3Scale(toCenter, m_speed), EMERGENCY_RETURN_STRENGTH * deltaTime);
        m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));
        UpdateRotationAndTilt(deltaTime);
    }
}

void Plane::UpdatePursuit(Plane &enemy, float deltaTime, bool shouldUpdatePath) {


    if (m_path.empty() || shouldUpdatePath) {

        auto newPath = m_dstar->PlanPath(m_position, enemy.GetPosition());
        if (!newPath.empty()) {
            m_path.clear();
            for (const auto& p : newPath) m_path.push_back(p);
        } else if (m_path.empty()) {
            m_path.push_back(enemy.GetPosition());
        }

    }

    while (m_path.size() > 1) {

        if (Vector3Distance(m_path[0], m_position) < 35.0f) {
            m_path.pop_front();
        } else {
            break;
        }
    }

    Vector3 steerDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));
    m_speed = CalculateEnergyVelocity(steerDir, deltaTime);

    m_velocity = Vector3Lerp(m_velocity, Vector3Scale(Vector3Normalize(steerDir), m_speed), 4.0f * deltaTime);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    ApplyWorldBounds(deltaTime);
    UpdateRotationAndTilt(deltaTime);
}

void Plane::UpdateEvade(const Vector3& enemyPos, const Vector3& enemyVel, float deltaTime, bool shouldUpdatePath) {
    float dist = Vector3Distance(m_position, enemyPos);
    bool isTurning = (!m_path.empty() && Vector3Distance(m_path.back(), enemyPos) < 300.0f);

    if (shouldUpdatePath || m_path.empty() || (isTurning && dist < 150.0f) || (!isTurning && dist > 550.0f)) {
        Vector3 target;
        float altitudeVariation = (float)GetRandomValue(-100, 100);

        if ((dist < 550.0f && !isTurning) || dist < 150.0f) {
            Vector3 forward = (Vector3LengthSqr(m_velocity) > 0.1f) ? Vector3Normalize(m_velocity) : Vector3{0, 0, 1};
            Vector3 away = Vector3Normalize(Vector3Subtract(m_position, enemyPos));
            target = Vector3Add(m_position, Vector3Scale(Vector3Normalize(Vector3Add(Vector3Scale(forward, 0.4f), Vector3Scale(away, 0.6f))), 700.0f));
            target.y = Clamp(m_position.y + altitudeVariation, 150.0f, 600.0f);
        } else {
            Vector3 enemyDir = (Vector3LengthSqr(enemyVel) > 0.1f) ? Vector3Normalize(enemyVel) : Vector3{0, 0, 1};
            target = Vector3Subtract(enemyPos, Vector3Scale(enemyDir, 300.0f));
            float targetAlt = (enemyPos.y > 400.0f) ? enemyPos.y - 150.0f : enemyPos.y + 150.0f;
            target.y = targetAlt + (altitudeVariation * 0.5f);
        }

        target.y = Clamp(target.y, 100.0f, 700.0f);

        auto newNodes = m_dstar->PlanPath(m_position, target);
        if (!newNodes.empty()) {
            m_path.clear();
            for (const auto& n : newNodes) m_path.push_back(n);
        }

    }

    float pruningDist = 40.0f;
    while (m_path.size() > 1 && Vector3Distance(m_path[0], m_position) < pruningDist) m_path.pop_front();

    Vector3 targetDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));


    if (!isTurning) {

        float wave = sinf(GetTime() * 1.5f) * 0.15f;

        float sideWave = cosf(GetTime() * 1.0f) * 0.1f;

        targetDir.y += wave;
        targetDir.x += sideWave;
        targetDir = Vector3Normalize(targetDir);
    }

    m_speed = CalculateEnergyVelocity(targetDir, deltaTime);

    Vector3 currentDir = (Vector3LengthSqr(m_velocity) > 0.01f) ? Vector3Normalize(m_velocity) : Vector3{0, 0, 1};
    float steeringSensitivity = isTurning ? 1.8f : 1.0f;
    Vector3 nextDir = Vector3Normalize(Vector3Lerp(currentDir, targetDir, steeringSensitivity * deltaTime));

    m_velocity = Vector3Scale(nextDir, m_speed);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    ApplyWorldBounds(deltaTime);
    UpdateRotationAndTilt(deltaTime);
}

float Plane::CalculateEnergyVelocity(const Vector3& targetDir, float deltaTime) {
    float baseSpeed = (CurrentState == AIState::EVADE) ? EVADE_SPEED : NORMAL_SPEED;

    Vector3 currentDir = (Vector3LengthSqr(m_velocity) > 0.01f) ? Vector3Normalize(m_velocity) : Vector3{0, 0, 1};
    float dot = Vector3DotProduct(currentDir, targetDir);

    // האטה מתונה יותר בפניות כדי לשמור על תנע (Momentum)
    float turnFactor = Remap(dot, 1.0f, -1.0f, 1.0f, 0.85f);
    float pitchFactor = 1.0f - (targetDir.y * 0.25f);

    float targetSpeed = baseSpeed * turnFactor * pitchFactor;

    // מקדם 0.5f הופך את התאוצה והתאטה לאיטיות וריאליסטיות
    return Lerp(m_speed, targetSpeed, 0.5f * deltaTime);
}

void Plane::UpdateRotationAndTilt(float deltaTime) {
    float speedSqr = Vector3LengthSqr(m_velocity);
    if (speedSqr > 0.25f) {
        // --- 1. חישוב Yaw (סיבוב אופקי) ---
        float targetRot = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
        float deltaAngle = targetRot - m_rotation;

        while (deltaAngle > 180.0f) deltaAngle -= 360.0f;
        while (deltaAngle < -180.0f) deltaAngle += 360.0f;

        // מהירות סיבוב האף - הורדנו ל-3.0f לתחושת כבדות
        m_rotation += deltaAngle * 3.0f * deltaTime;

        // --- 2. חישוב Pitch (נטייה אנכית) ---
        float speed = sqrtf(speedSqr);
        float targetPitch = asinf(m_velocity.y / speed) * RAD2DEG;

        // החלקת ה-Pitch - הורדנו ל-4.0f
        m_pitch = Lerp(m_pitch, targetPitch, 4.0f * deltaTime);

        // --- 3. חישוב Tilt (גלגול - Banking) ---
        // הקטנו את המכפיל מ-4.5 ל-2.0 כדי שההטיה לא תהיה קיצונית מדי מיד
        float targetTilt = Clamp(deltaAngle * 2.0f, -45.0f, 45.0f);

        // מקדם ה-Lerp ירד מ-5.0 ל-1.2 לתנועה איטית וכבדה מאוד
        // זה גורם למטוס "להתגלגל" לצד לאט לאט כמו מטוס מטען או מטוס קרב עמוס
        m_tilt = Lerp(m_tilt, targetTilt, 1.2f * deltaTime);
    }
}

void Plane::Draw(Camera3D camera) const {

    Matrix rollMat = MatrixRotateZ(-m_tilt * DEG2RAD);
    Matrix pitchMat = MatrixRotateX(-m_pitch * DEG2RAD);
    Matrix yawMat = MatrixRotateY(m_rotation * DEG2RAD);

    Matrix dynamicRotation = MatrixMultiply(rollMat, MatrixMultiply(pitchMat, yawMat));
    Matrix finalRotation = MatrixMultiply(m_baseTransform, dynamicRotation);

    Quaternion q = QuaternionFromMatrix(finalRotation);
    Vector3 axis;
    float angle;
    QuaternionToAxisAngle(q, &axis, &angle);

    DrawModelEx(m_model, m_position, axis, angle * RAD2DEG, {0.1f, 0.1f, 0.1f}, WHITE);


    if (!m_path.empty()) {
        Color pathColor = (CurrentState == AIState::PURSUIT) ? RED : LIME;
        DrawLine3D(m_position, m_path[0], pathColor);
        for (size_t i = 0; i < m_path.size() - 1; i++) {
            DrawLine3D(m_path[i], m_path[i + 1], pathColor);
            DrawSphere(m_path[i], 0.6f, pathColor);
        }
        DrawSphere(m_path.back(), 1.5f, GOLD);
    }

    if (m_locked) {

        EndMode3D();


        Vector2 screenPos = GetWorldToScreen(m_position, camera);


        float size = 50.0f;


        float dist = Vector3Distance(camera.position, m_position);
        size = Clamp(2000.0f / dist, 20.0f, 150.0f);


        Rectangle rect = { screenPos.x - size/2, screenPos.y - size/2, size, size };
        DrawRectangleLinesEx(rect, 2.0f, RED);

        float l = size * 0.2f;
        DrawLine(rect.x, rect.y, rect.x + l, rect.y, RED);
        DrawLine(rect.x, rect.y, rect.x, rect.y + l, RED);
        DrawLine(rect.x + size, rect.y, rect.x + size - l, rect.y, RED);
        DrawLine(rect.x + size, rect.y, rect.x + size, rect.y + l, RED);
        DrawLine(rect.x, rect.y + size, rect.x + l, rect.y + size, RED);
        DrawLine(rect.x, rect.y + size, rect.x, rect.y + size - l, RED);
        DrawLine(rect.x + size, rect.y + size, rect.x + size - l, rect.y + size, RED);
        DrawLine(rect.x + size, rect.y + size, rect.x + size, rect.y + size - l, RED);

        BeginMode3D(camera);
    }
}