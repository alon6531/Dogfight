//
// Created by User on 12/04/2026.
//

#include "EvasionState.h"
#include "../../Plane.h"
#include "raymath.h"

EvasionState::EvasionState(Plane &self, NavigationGraph &navGraph,
                           std::shared_ptr<Plane> enemy) : AIState(self, navGraph), m_enemy(std::move(enemy)) {
    m_dStarLite = std::make_unique<DStarLite>(p_graph);


}

EvasionState::~EvasionState() = default;

AIStateType EvasionState::Update(float deltaTime) {
    const int   MIN_PATH_NODES      = 2;         // אם יש פחות מ-3 נודים, תחשב נתיב חדש
    const int   TARGET_PATH_LENGTH  = 4;        // כמה נודים לייצר בכל תכנון
    const float REACH_RADIUS_SQ      = 6400.0f;   // 80^2
    const float TACTICAL_ALT_DIFF    = 50.0f;
    const float MAX_ALLOWED_WEIGHT   = 12000.0f;
    const float TURN_DIST   = 300.0f;


    Vector3 selfPos = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();
    float distToEnemy = Vector3Distance(selfPos, enemyPos);

    // בדיקה: האם האויב נועל אותי? (m_enemy->isLocked אומר שהוא נועל מטרה)
    bool beingLocked = m_enemy->GetTargetLock().isLocked;

    // --- שלב 1: ניהול תימרון היפוך (Loop) ---
    if (m_isPerformingLoop) {
        Vector3 dirToEnemy = Vector3Normalize(Vector3Subtract(enemyPos, selfPos));
        float alignment = Vector3DotProduct(p_self.GetForward(), dirToEnemy);

        // אם השלמנו את הסיבוב, אנחנו מעליו ומסתכלים עליו - חזרה למרדף
        if (alignment > 0.85f && selfPos.y > enemyPos.y + 20.0f) {
            m_isPerformingLoop = false;
            p_path.clear();
            return AIStateType::PURSUIT;
        }

        if (p_path.empty()) m_isPerformingLoop = false;
    }

    // --- שלב 2: בחירת אסטרטגיה (לפי כמות Nodes) ---
    if (!m_isPerformingLoop && p_path.size() < MIN_PATH_NODES) {

        // 1. הגנה אקטיבית: שבירת נעילה (Notch)
        if (beingLocked) {
            Vector3 dirFromEnemy = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
            Vector3 notchDir = Vector3Normalize(Vector3CrossProduct(dirFromEnemy, {0, 1, 0}));
            if (Vector3DotProduct(p_self.GetForward(), notchDir) < 0) notchDir = Vector3Negate(notchDir);

            p_path.clear();
            p_path.push_back(Vector3Add(selfPos, Vector3Scale(notchDir, 600.0f)));
        }

        else if (distToEnemy < TURN_DIST) {
            for (int attempt = 0; attempt < 5; attempt++) {
                int escapeIdx = p_graph.GetRandomNodeFarFrom(enemyPos, 2000.0f);
                if (escapeIdx == -1) continue;

                auto pts = m_dStarLite->PlanPath(selfPos, p_graph.GetNodes()[escapeIdx].position);
                float cost = m_dStarLite->GetLastPathCost();

                if (!pts.empty() && cost > 0 && cost < MAX_ALLOWED_WEIGHT) {
                    p_path.clear();
                    int count = 0;
                    for(const auto& p : pts) {
                        p_path.push_back(p);
                        if (++count >= TARGET_PATH_LENGTH) break; // מגביל את אורך הנתיב
                    }
                    break;
                }
            }
        }
        // 3. יתרון גובה: אם האויב רחוק מספיק, נתחיל את הטיפוס
        else {
            m_isPerformingLoop = true;
            p_path.clear();

            // במקום לטוס לכיוון האויב, אנחנו מושכים ישר למעלה מהמיקום הנוכחי שלנו
            // זה מבטל את ה"איגוף" המיותר ב-X ו-Z
            Vector3 climbPoint = { selfPos.x, enemyPos.y + TACTICAL_ALT_DIFF, selfPos.z };

            // נקודת ההיפוך תהיה מעט קדימה מהכיוון הנוכחי שלנו, אבל בגובה
            Vector3 forwardBoost = Vector3Scale(p_self.GetForward(), 100.0f);
            Vector3 flipPoint = Vector3Add(climbPoint, forwardBoost);

            p_path.push_back(climbPoint);
            p_path.push_back(flipPoint);
        }
    }

    // --- שלב 3: תנועה ---
    if (!p_path.empty()) {
        Vector3 nextTarget = p_path.front();
        m_currentDir = nextTarget;

        float dx = nextTarget.x - selfPos.x;
        float dy = nextTarget.y - selfPos.y;
        float dz = nextTarget.z - selfPos.z;

        if ((dx*dx + dy*dy + dz*dz) < REACH_RADIUS_SQ) {
            p_path.pop_front();
        }

        p_self.SteerTowards(nextTarget, deltaTime);
    } else {
        // Fallback
        Vector3 escapeDir = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
        m_currentDir = Vector3Add(selfPos, Vector3Scale(escapeDir, 1000.0f));
        p_self.SteerTowards(m_currentDir, deltaTime);
    }

    return AIStateType::EVASION;
}

Vector3 EvasionState::GetCurrentTargetFromAI() {
    return m_currentDir;
}
