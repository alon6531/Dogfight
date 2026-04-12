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
    // --- הגדרות מאוזנות למהירות גבוהה (3.5x) ---
    const float WAYPOINT_REACH_DIST  = 250.0f;   // הגדלנו: במהירות גבוהה צריך רדיוס פגיעה גדול יותר
    const int   MAX_NODES_IN_PATH    = 10;         // פחות צמתים = חישוב מהיר ופחות זיגזגים
    const float ESCAPE_MIN_DIST      = 1000.0f;   // הגדלנו: תברח רחוק באמת לפני שאתה מנסה להילחם
    const float ESCAPE_SEARCH_RADIUS = 3000.0f;

    if (!m_enemy || !m_dStarLite) return AIStateType::IDLE;

    Vector3 selfPos = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();
    float distToEnemy = Vector3Distance(selfPos, enemyPos);
    AIStateType enemyState = m_enemy->GetCurrentStateType();

    // --- 1. בדיקת יציאה חכמה (מניעת שיגעון) ---
    Vector3 dirToMe = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
    float enemyAlignment = Vector3DotProduct(m_enemy->GetForward(), dirToMe);

    Vector3 dirToEnemy = Vector3Normalize(Vector3Subtract(enemyPos, selfPos));
    float myAlignmentOnEnemy = Vector3DotProduct(p_self.GetForward(), dirToEnemy);

    // יציאה ל-PURSUIT רק אם:
    // א. האויב בורח מאיתנו (ניצחנו במרדף)
    // ב. או שאנחנו רחוקים מאוד וגם האף שלנו כבר פונה לכיוונו
    bool canTurnToFight = (distToEnemy > ESCAPE_MIN_DIST && myAlignmentOnEnemy > 0.3f);

    if (enemyState == AIStateType::EVASION || canTurnToFight) {
        p_path.clear();
        return AIStateType::PURSUIT;
    }

    // --- 2. חישוב נתיב בריחה (אופטימיזציה ל-CPU) ---
    // אנחנו מחשבים נתיב חדש רק כשהישן כמעט נגמר, ולא מוחקים אותו עד שהחדש מוכן
    if (p_path.size() < 2) {
        int escapeNodeIdx = p_graph.GetRandomNodeFarFrom(enemyPos, ESCAPE_SEARCH_RADIUS);

        if (escapeNodeIdx != -1) {
            Vector3 targetPos = p_graph.GetNodes()[escapeNodeIdx].position;

            // חישוב נתיב לוקאלי
            auto pathPoints = m_dStarLite->PlanPath(selfPos, targetPos);

            if (!pathPoints.empty()) {
                p_path.clear();
                int nodesCount = 0;
                for (const auto &pt : pathPoints) {
                    if (nodesCount >= MAX_NODES_IN_PATH) break;
                    p_path.push_back(pt);
                    nodesCount++;
                }
            }
        }
    }

    // --- 3. ניווט והיגוי ---
    if (!p_path.empty()) {
        Vector3 nextTarget = p_path.front();
        m_currentDir = nextTarget;

        // בדיקת הגעה לנקודה
        if (Vector3Distance(selfPos, nextTarget) < WAYPOINT_REACH_DIST) {
            p_path.pop_front();
            if (!p_path.empty()) nextTarget = p_path.front();
        }

        p_self.SteerTowards(nextTarget, deltaTime);
    } else {
        // Fallback: אם אין נתיב, פשוט טוס ישר קדימה כדי לשמור על המהירות והבוסט
        Vector3 forwardEscape = Vector3Add(selfPos, Vector3Scale(p_self.GetForward(), 500.0f));
        p_self.SteerTowards(forwardEscape, deltaTime);
    }

    return AIStateType::EVASION;
}

Vector3 EvasionState::GetCurrentTargetFromAI() {
    return m_currentDir;
}
