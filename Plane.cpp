//
// Created by User on 03/04/2026.
//

#include "Plane.h"

#include <iostream>
#include <memory>
#include <ostream>

#include "DStarLite.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include "MPCController.h"

Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration, const Color& color, NavigationGraph &graph)
    : m_graph(graph),
      m_position(position),
        m_velocity({0, 0, 0}),
      m_acceleration(acceleration) {
    const Mesh mesh = GenMeshCube(5.0f, 2.5f, 10.0f);
    m_model = LoadModel("plane.glb");
    for (int i = 0; i < m_model.materialCount; i++) {
        m_model.materials[i].maps[MATERIAL_MAP_ALBEDO].color = color;
    }
    m_acceleration = Vector3();
    CurrentState = AIState::IDLE;
    CurrentEvent = AIEvent::NONE;

    m_dstar = std::make_unique<DStarLite>(m_graph);

    Matrix rotX = MatrixRotateX(90.0f * DEG2RAD);

    // 2. צור מטריצת סיבוב לציר Y (כדי לסובב אותו מהצד לחזית)
    // אם הוא מסתכל שמאלה, נסה 90 או -90 כדי ליישר אותו קדימה
    Matrix rotY = MatrixRotateY(-90.0f * DEG2RAD);

    // 3. הכפל אותן יחד (הסדר חשוב!) ושמור ב-transform
    m_model.transform = MatrixMultiply(rotX, rotY);

}

void Plane::SetDestinationViaAStar(int targetNodeIdx) {
    int startNodeIdx = m_graph.GetClosestNode(m_position);
    m_velocity = {0, 0, 0};

    if (startNodeIdx != -1) {
        m_path = m_graph.FindPathViaAStar(startNodeIdx, targetNodeIdx);

        std::cout << "=== PATH ===" << std::endl;
        for (int i = 0; i < (int)m_path.size(); i++) {
            std::cout << "  [" << i << "] " << m_path[i].x << ", " << m_path[i].y << ", " << m_path[i].z << std::endl;
        }

        m_rotation = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
    }
}

Plane::~Plane() {
    UnloadModel(m_model);
}

void Plane::Update(Plane& enemy,float deltaTime, const MPCController &mpc) {

    m_delay += deltaTime;

    if (GetCurrentState() == AIState::PATROL) {
        UpdatePatrol(deltaTime, mpc);

    }

    if (GetCurrentState() == AIState::PURSUIT) {
        UpdatePursuit(enemy, deltaTime, mpc);
    }
    if (GetCurrentState() == AIState::EVADE) {
        UpdateEvade(enemy.position(), enemy.velocity(), deltaTime, mpc);
    }



    if (m_delay >  DELAY_TIME)
        m_delay = 0;

}

void Plane::UpdatePatrol(float deltaTime, const MPCController &mpc) {
    deltaTime = fminf(deltaTime, 0.05f);

    if (m_path.empty()) {

        return;
    }

    // Prune nodes we've passed
    while (m_path.size() > 1) {
        float dist = Vector3Length(Vector3Subtract(m_path[0], m_position));
        if (dist < 40.0f) {
            m_path.erase(m_path.begin());
        } else {
            break;
        }
    }

    // Final node arrival
    if (m_path.size() == 1) {
        float dist = Vector3Length(Vector3Subtract(m_path[0], m_position));
        if (dist < 40.0f) {
            m_path.clear();
            return;
        }
    }

    if (m_path.empty()) return;

    Vector3 toTarget = Vector3Subtract(m_path[0], m_position);
    Vector3 steerDir = Vector3Normalize(toTarget);
    if (steerDir.y > 0.4f)  steerDir.y =  0.4f;
    if (steerDir.y < -0.4f) steerDir.y = -0.4f;
    steerDir = Vector3Normalize(steerDir);

    m_velocity = Vector3Lerp(m_velocity, Vector3Scale(steerDir, m_speed), 4.0f * deltaTime);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    UpdateRotationAndTilt(deltaTime);
}

void Plane::UpdatePursuit(Plane &enemy, float deltaTime, const MPCController &mpc) {

    if (m_path.size() <= 1) {
        // אם אין נתיב, טוס ישר קדימה לאט וחפש נתיב חדש
        Vector3 forward = (Vector3LengthSqr(m_velocity) > 0.1f) ? Vector3Normalize(m_velocity) : Vector3{0, 0, 1};
        m_velocity = Vector3Lerp(m_velocity, Vector3Scale(forward, m_speed * 0.5f), deltaTime);
        m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));
    }
        if (m_delay>  DELAY_TIME)
            m_path = m_dstar->PlanPath(m_position, PredictEnemyPos(enemy, deltaTime));


        if (m_path.size() > 1) {
            Vector3 forward = Vector3Normalize(m_velocity);

            while (m_path.size() > 2) {
                Vector3 toNode = Vector3Subtract(m_path[0], m_position);
                float dist = Vector3Length(toNode);


                float dot = Vector3DotProduct(forward, Vector3Normalize(toNode));

                if (dot < 0.1f || dist < 35.0f) {
                    m_path.erase(m_path.begin());
                } else {
                    break;
                }
            }


            //static MPCController mpc({12, 0.15f, 50.0f, 2.5f, 1.0f});
            Vector3 steerDir = mpc.CalculateSteering(m_position, m_velocity, m_path);


            if (steerDir.y > 0.4f) steerDir.y = 0.4f;
            steerDir = Vector3Normalize(steerDir);


            Vector3 targetVelocity = Vector3Scale(steerDir, m_speed);
            m_velocity = Vector3Lerp(m_velocity, targetVelocity, 6.0f * deltaTime);

            m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));


            UpdateRotationAndTilt(deltaTime);
        }
}


Vector3 Plane::PredictEnemyPos(const Plane& enemy, float deltaTime) {
    Vector3 targetPos = enemy.position();
    Vector3 enemyVel = enemy.velocity();
    float dist = Vector3Distance(m_position, targetPos);

    // חישוב זמן הגעה משוער (זמן = מרחק / מהירות)
    float timeToReach = dist / (m_speed + 1.0f); // הגנה מחילוק ב-0

    // הגבלה של החיזוי (לא לחזות יותר מ-4 שניות קדימה כי זה לא אמין)
    timeToReach = fminf(timeToReach, 4.0f);

    // הנקודה החזויה: מיקום האויב + המהירות שלו כפול הזמן שיקח לי להגיע
    Vector3 predictedPos = Vector3Add(targetPos, Vector3Scale(enemyVel, timeToReach));

    return predictedPos;
}

void Plane::UpdateEvade(const Vector3& enemyPos, const Vector3& enemyVel, float deltaTime, const MPCController& mpc) {
    // הגנה: אם המהירות של האויב היא אפס, נשתמש בכיוון אליו הוא מסתכל או וקטור ברירת מחדל
    Vector3 enemyForward = (Vector3LengthSqr(enemyVel) > 0.001f)
                           ? Vector3Normalize(enemyVel)
                           : Vector3{0, 0, 1};

    Vector3 toEnemy = Vector3Subtract(enemyPos, m_position);
    Vector3 forward = (Vector3LengthSqr(m_velocity) > 0.001f)
                      ? Vector3Normalize(m_velocity)
                      : Vector3{0, 0, 1};

    // חישוב וקטור הצידה - עם הגנה למקרה שהאויב טס בדיוק למעלה
    Vector3 up = {0, 1, 0};
    if (fabsf(Vector3DotProduct(enemyForward, up)) > 0.9f) up = {1, 0, 0}; // אם הוא טס למעלה, נשתמש בציר X

    Vector3 sideEscape = Vector3Normalize(Vector3CrossProduct(enemyForward, up));

    // בחר צד בריחה
    if (Vector3DotProduct(sideEscape, toEnemy) > 0) {
        sideEscape = Vector3Negate(sideEscape);
    }

    Vector3 escapeTarget = Vector3Add(m_position, Vector3Scale(sideEscape, 150.0f));
    escapeTarget.y += 30.0f;

    // הגנה על ה-D*: וודא שהיעד לא זהה למיקום הנוכחי
    if (Vector3DistanceSqr(m_position, escapeTarget) < 1.0f) return;

    if (m_delay >  DELAY_TIME)
        m_path = m_dstar->PlanPath(m_position, escapeTarget);

    if (m_path.size() > 1) {
        Vector3 steerDir = mpc.CalculateSteering(m_position, m_velocity, m_path);

        // הגנה על SteerDir
        if (Vector3LengthSqr(steerDir) < 0.001f) steerDir = forward;
        else steerDir = Vector3Normalize(steerDir);

        m_speed = 180.0f;
        Vector3 targetVelocity = Vector3Scale(steerDir, m_speed);
        m_velocity = Vector3Lerp(m_velocity, targetVelocity, 6.0f * deltaTime);
        m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

        UpdateRotationAndTilt(deltaTime);
    }
}
void Plane::UpdateRotationAndTilt(float deltaTime) {
    if (Vector3Length(m_velocity) > 0.5f) {
        float targetRot = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
        float deltaAngle = targetRot - m_rotation;

        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle < -180) deltaAngle += 360;

        float targetTilt = Clamp(deltaAngle * 4.0f, -60.0f, 60.0f); // בחמיקה הנטייה קיצונית יותר
        m_tilt = Lerp(m_tilt, targetTilt, 6.0f * deltaTime);
        m_rotation += deltaAngle * 5.0f * deltaTime;
    }
}


void Plane::Draw() const {
    // 1. חישוב מטריצת הסיבוב המאוחדת (בלי המיקום!)
    // קודם הנטייה (Roll) ואז הכיוון (Yaw)
    Matrix tiltMat = MatrixRotateZ(-m_tilt * DEG2RAD);
    Matrix rotationMat = MatrixRotateY(m_rotation * DEG2RAD);

    // שילוב עם מטריצת הבסיס מה-Constructor (היישור המקורי)
    // הערה: m_baseTransform צריכה להישמר ב-Constructor ולא להידרס
    Matrix modelRotation = MatrixMultiply(tiltMat, rotationMat);

    // 2. ציור באמצעות DrawModel
    // במקום לשנות את המודל, אנחנו נשתמש ב-Quaternion כדי לעקוף את מגבלת הציר היחיד
    Quaternion q = QuaternionFromMatrix(modelRotation);
    Vector3 axis;
    float angle;
    QuaternionToAxisAngle(q, &axis, &angle);

    // ציור המודל במיקום האמיתי שלו
    DrawModelEx(m_model, m_position, axis, angle * RAD2DEG, { 0.1f, 0.1f, 0.1f }, WHITE);
}
