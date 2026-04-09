#include "Plane.h"
#include <iostream>
#include "DStarLite.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include "MPCController.h"



static const float WORLD_BOUND_LIMIT = 850.0f;
static const float EMERGENCY_RETURN_STRENGTH = 8.0f;

Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration, const Color& color, NavigationGraph &graph)
    : m_position(position),
      m_velocity(velocity),
      m_acceleration(acceleration),
      m_graph(graph),
      CurrentState(AIState::IDLE),
      CurrentEvent(AIEvent::NONE)
{
    m_model = LoadModel("plane.glb");
    for (int i = 0; i < m_model.materialCount; i++) {
        m_model.materials[i].maps[MATERIAL_MAP_ALBEDO].color = color;
    }

    Matrix rotX = MatrixRotateX(90.0f * DEG2RAD);
    Matrix rotY = MatrixRotateY(-90.0f * DEG2RAD);
    m_baseTransform = MatrixMultiply(rotX, rotY);

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
    m_delay += deltaTime;
    bool shouldUpdatePath = (m_delay >= UPDATE_PATH_DELAY);

    switch (CurrentState) {
        case AIState::PATROL:
            UpdatePatrol(deltaTime, mpc);
            break;
        case AIState::PURSUIT:
            UpdatePursuit(enemy, deltaTime, mpc, shouldUpdatePath);
            break;
        case AIState::EVADE:
            UpdateEvade(enemy.GetPosition(), enemy.GetVelocity(), deltaTime, mpc, shouldUpdatePath);
            break;
        default:
            break;
    }

    if (shouldUpdatePath) {
        m_delay = 0.0f;
    }
}

void Plane::UpdatePatrol(float deltaTime, const MPCController &mpc) {
    deltaTime = fminf(deltaTime, 0.05f);

    if (m_path.empty()) return;

    while (m_path.size() > 1) {
        if (Vector3Distance(m_path[0], m_position) < 40.0f)
            m_path.pop_front();  // O(1)
        else
            break;
    }

    if (m_path.size() == 1 && Vector3Distance(m_path[0], m_position) < 40.0f) {
        m_path.clear();
        return;
    }

    if (m_path.empty()) return;

    Vector3 steerDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));
    steerDir.y = Clamp(steerDir.y, -0.4f, 0.4f);
    steerDir = Vector3Normalize(steerDir);

    m_velocity = Vector3Lerp(m_velocity, Vector3Scale(steerDir, m_speed), 4.0f * deltaTime);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    UpdateRotationAndTilt(deltaTime);
}

void Plane::ApplyWorldBounds(float deltaTime) {
    if (fabsf(m_position.x) > WORLD_BOUND_LIMIT || fabsf(m_position.z) > WORLD_BOUND_LIMIT) {
        Vector3 toCenter = Vector3Normalize(Vector3Subtract({0, m_position.y, 0}, m_position));
        // פנייה אגרסיבית חזרה למרכז
        m_velocity = Vector3Lerp(m_velocity, Vector3Scale(toCenter, m_speed), EMERGENCY_RETURN_STRENGTH * deltaTime);
        m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));
        UpdateRotationAndTilt(deltaTime);
    }
}

void Plane::UpdatePursuit(Plane &enemy, float deltaTime, const MPCController &mpc, bool shouldUpdatePath) {
    deltaTime = fminf(deltaTime, 0.05f);

    m_speed = NORMAL_SPEED;


    // 1. עדכון נתיב דינמי (D* Lite) - כמו ב-SetDestination רק אוטומטי
    if (m_path.empty() || shouldUpdatePath) {
        auto newPath = m_dstar->PlanPath(m_position, enemy.GetPosition());
        if (!newPath.empty()) {
            m_path.clear();
            for (const auto& p : newPath) m_path.push_back(p);
        } else if (m_path.empty()) {
            // אם באמת אין כלום, שים לפחות את היעד כדי שלא יעוף לאופק
            m_path.push_back(enemy.GetPosition());
        }
    }

    // 2. לוגיקת ה-Pruning המדויקת מה-Patrol שלך (O(1))
    while (m_path.size() > 1) {
        // השתמשתי ב-40.0f כמו שכתבת ב-Patrol המקורי שלך
        if (Vector3Distance(m_path[0], m_position) < 40.0f) {
            m_path.pop_front();
        } else {
            break;
        }
    }

    // הגעה לנקודה האחרונה (כמו ב-Patrol)
    if (m_path.size() == 1 && Vector3Distance(m_path[0], m_position) < 40.0f) {
        // ב Pursuit אנחנו לא מוחקים את הנקודה האחרונה אלא נצמדים אליה
    }

    if (m_path.empty()) return;

    // 3. תנועה ישירה (Direct Steering) - ללא MPC, בדיוק כמו ב-Patrol
    Vector3 steerDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));

    // הגבלת זווית עלייה/ירידה (כמו ב-Patrol)
    steerDir.y = Clamp(steerDir.y, -0.4f, 0.4f);
    steerDir = Vector3Normalize(steerDir);

    // 4. החלת המהירות והמיקום (עם מקדם 4.0f כמו ב-Patrol)
    m_velocity = Vector3Lerp(m_velocity, Vector3Scale(steerDir, m_speed), 4.0f * deltaTime);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    // 5. הגנת גבולות עולם (World Bounds)
    ApplyWorldBounds(deltaTime);

    UpdateRotationAndTilt(deltaTime);
}

void Plane::UpdateEvade(const Vector3& enemyPos, const Vector3& enemyVel, float deltaTime, const MPCController& mpc, bool shouldUpdatePath) {
    deltaTime = fminf(deltaTime, 0.05f);
    float distToEnemy = Vector3Distance(m_position, enemyPos);

    const float PANIC_DISTANCE = 150.0f;
    const float TURN_DISTANCE = 550.0f; // הגדלנו מעט כדי לתת מרחב לטיפוס

    bool isCurrentlyTurning = false;
    if (!m_path.empty()) {
        float distTargetToEnemy = Vector3Distance(m_path.back(), enemyPos);
        if (distTargetToEnemy < 300.0f) isCurrentlyTurning = true;
    }

    bool needNewPath = shouldUpdatePath || m_path.empty();

    if (isCurrentlyTurning && distToEnemy < PANIC_DISTANCE) {
        needNewPath = true;
    } else if (!isCurrentlyTurning && distToEnemy > TURN_DISTANCE) {
        needNewPath = true;
    }

    if (needNewPath) {
        Vector3 finalTarget;

        if (distToEnemy < TURN_DISTANCE && !isCurrentlyTurning || distToEnemy < PANIC_DISTANCE) {
            // --- שלב הבריחה (DASH) ---
            Vector3 forward = (Vector3LengthSqr(m_velocity) > 1.0f) ? Vector3Normalize(m_velocity) : Vector3{0, 0, 1};
            Vector3 awayFromEnemy = Vector3Normalize(Vector3Subtract(m_position, enemyPos));
            Vector3 dashDir = Vector3Normalize(Vector3Add(Vector3Scale(forward, 0.5f), Vector3Scale(awayFromEnemy, 0.5f)));

            // בבריחה נשמור על גובה יציב או נרד מעט כדי לצבור מהירות
            finalTarget = Vector3Add(m_position, Vector3Scale(dashDir, 600.0f));
            finalTarget.y = Clamp(m_position.y - 20.0f, 150.0f, 400.0f);
        }
        else {
            // --- שלב הסיבוב הטקטי עם יתרון גובה (HIGH YO-YO) ---
            Vector3 enemyDir = (Vector3LengthSqr(enemyVel) > 1.0f) ? Vector3Normalize(enemyVel) : Vector3{0, 0, 1};

            // 1. נקודת הזנב הבסיסית
            Vector3 tailPoint = Vector3Subtract(enemyPos, Vector3Scale(enemyDir, 200.0f));

            // 2. הוספת יתרון גובה משמעותי
            // אנחנו שואפים להיות בערך 80-100 יחידות מעל האויב בזמן הסיבוב
            float altitudeAdvantage = 100.0f;
            tailPoint.y = enemyPos.y + altitudeAdvantage;

            finalTarget = tailPoint;
            std::cout << "EVADE: Climbing for altitude advantage and turning back!" << std::endl;
        }

        // הגבלת גובה עליון כדי לא לצאת מגבולות המפה
        finalTarget.y = Clamp(finalTarget.y, 150.0f, 600.0f);

        auto newNodes = m_dstar->PlanPath(m_position, finalTarget);
        if (!newNodes.empty()) {
            m_path.clear();
            for (const auto& node : newNodes) m_path.push_back(node);
        }
    }

    if (m_path.empty()) return;

    // Pruning
    float pruningDist = isCurrentlyTurning ? 100.0f : 60.0f;
    while (m_path.size() > 1 && Vector3Distance(m_path[0], m_position) < pruningDist) {
        m_path.pop_front();
    }

    Vector3 targetDir = Vector3Normalize(Vector3Subtract(m_path[0], m_position));
    Vector3 currentDir = Vector3Normalize(m_velocity);

    // בטיפוס (isCurrentlyTurning), אנחנו צריכים יותר כוח פנייה
    float turnSensitivity = isCurrentlyTurning ? 5.5f : 2.5f;

    // בסיבוב טיפוס המהירות יורדת מעט באופן טבעי (המרת אנרגיה קינטית לפוטנציאלית)
    m_speed = isCurrentlyTurning ? (NORMAL_SPEED * 0.9f) : EVADE_SPEED;

    Vector3 smoothDir = Vector3Lerp(currentDir, targetDir, turnSensitivity * deltaTime);
    smoothDir = Vector3Normalize(smoothDir);

    m_velocity = Vector3Scale(smoothDir, m_speed);
    m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));

    ApplyWorldBounds(deltaTime);
    UpdateRotationAndTilt(deltaTime);
}


void Plane::UpdateRotationAndTilt(float deltaTime) {
    if (Vector3LengthSqr(m_velocity) > 0.25f) {
        float targetRot = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
        float deltaAngle = targetRot - m_rotation;

        while (deltaAngle > 180.0f) deltaAngle -= 360.0f;
        while (deltaAngle < -180.0f) deltaAngle += 360.0f;

        float targetTilt = Clamp(deltaAngle * 4.0f, -60.0f, 60.0f);
        m_tilt = Lerp(m_tilt, targetTilt, fminf(6.0f * deltaTime, 0.85f));
        m_rotation += deltaAngle * 5.0f * deltaTime;
    }
}

void Plane::Draw(Camera3D camera) const {
    // מודל
    Matrix tiltMat       = MatrixRotateZ(-m_tilt * DEG2RAD);
    Matrix rotationMat   = MatrixRotateY(m_rotation * DEG2RAD);
    Matrix dynamicRotation = MatrixMultiply(tiltMat, rotationMat);
    Matrix finalRotation   = MatrixMultiply(m_baseTransform, dynamicRotation);

    Quaternion q = QuaternionFromMatrix(finalRotation);
    Vector3 axis;
    float angle;
    QuaternionToAxisAngle(q, &axis, &angle);

    DrawModelEx(m_model, m_position, axis, angle * RAD2DEG, {0.1f, 0.1f, 0.1f}, WHITE);

    // נתיב
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
        // 1. נסיים זמנית את מצב ה-3D
        EndMode3D();

        // 2. המרת מיקום המטוס למסך באמצעות המצלמה האמיתית
        Vector2 screenPos = GetWorldToScreen(m_position, camera);

        // 3. הגדרת גודל הריבוע
        float size = 50.0f;

        // בונוס: הגדלת הריבוע ככל שהמטוס קרוב יותר למצלמה
        float dist = Vector3Distance(camera.position, m_position);
        size = Clamp(2000.0f / dist, 20.0f, 150.0f);

        // 4. ציור הריבוע (נשתמש בקידומת :: כדי לוודא שאנחנו קוראים ל-Struct של Raylib)
        Rectangle rect = { screenPos.x - size/2, screenPos.y - size/2, size, size };
        DrawRectangleLinesEx(rect, 2.0f, RED);

        // פינות של כוונת (HUD Style)
        float l = size * 0.2f;
        // פינה שמאלית עליונה
        DrawLine(rect.x, rect.y, rect.x + l, rect.y, RED);
        DrawLine(rect.x, rect.y, rect.x, rect.y + l, RED);
        // פינה ימנית עליונה
        DrawLine(rect.x + size, rect.y, rect.x + size - l, rect.y, RED);
        DrawLine(rect.x + size, rect.y, rect.x + size, rect.y + l, RED);
        // פינה שמאלית תחתונה
        DrawLine(rect.x, rect.y + size, rect.x + l, rect.y + size, RED);
        DrawLine(rect.x, rect.y + size, rect.x, rect.y + size - l, RED);
        // פינה ימנית תחתונה
        DrawLine(rect.x + size, rect.y + size, rect.x + size - l, rect.y + size, RED);
        DrawLine(rect.x + size, rect.y + size, rect.x + size, rect.y + size - l, RED);

        // 5. חזרה למצב 3D
        BeginMode3D(camera);
    }
}