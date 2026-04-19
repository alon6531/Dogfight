//
// Created by User on 12/04/2026.
//

#include "PursuitState.h"

#include <iostream>

#include "raymath.h"
#include "../../Plane.h"

PursuitState::PursuitState(Plane &self, NavigationGraph &navGraph, std::shared_ptr<Plane> enemy)
    : AIState(self, navGraph), m_enemy(enemy) {


    m_dStarLite = std::make_unique<DStarLite>(p_graph);


}

PursuitState::~PursuitState() = default;

AIStateType PursuitState::Update(float deltaTime) {
    const float BREAK_DISTANCE_SQ = 640000.0f; // 800^2
    const float WAYPOINT_RADIUS_SQ = 10000.0f;
    const float MAX_PURSUIT_PATH_WEIGHT = 16000.0f;

    Vector3 selfPos = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();

    // 1. לוגיקת שבירה לבריחה (Evasion) - חכם יותר!
    AIStateType enemyState = m_enemy->GetCurrentStateType();
    if (enemyState == AIStateType::PURSUIT || enemyState == AIStateType::PATROL) {
        Vector3 dirToMe = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
        float enemyAlignment = Vector3DotProduct(m_enemy->GetForward(), dirToMe);

        float dx = enemyPos.x - selfPos.x;
        float dy = enemyPos.y - selfPos.y;
        float dz = enemyPos.z - selfPos.z;
        float distSq = dx*dx + dy*dy + dz*dz;

        // התנאי המתוקן: תברח רק אם האויב מכוון אליך, קרוב, **ואתה לא ביתרון גובה**
        // אם אתה מעל האויב (selfPos.y > enemyPos.y + 50), אל תברח! תלחץ לנעילה!
        if (enemyAlignment > 0.8f && distSq < BREAK_DISTANCE_SQ && selfPos.y <= enemyPos.y + 50.0f) {
            p_path.clear();
            return AIStateType::EVASION;
        }
    }

    // 2. תזמון חישוב נתיב (אל תחשב ב-4000 FPS!)
    m_pathTimer += deltaTime;
    // נחשב נתיב רק פעם ב-0.2 שניות או אם הנתיב נגמר
    if (m_pathTimer > 0.2f || p_path.empty()) {
        m_pathTimer = 0;

        auto pathPoints = m_dStarLite->PlanPath(selfPos, enemyPos);
        float pathWeight = m_dStarLite->GetLastPathCost();

        // בדיקה אם הנתיב חוקי ולא חסום בכבדות
        if (!pathPoints.empty() && pathWeight < MAX_PURSUIT_PATH_WEIGHT) {
            p_path.clear();
            for (const auto& pt : pathPoints)
                p_path.push_back(pt);
        }
    }

    // 3. ניווט
    if (!p_path.empty()) {
        Vector3 nextTarget = p_path.front();

        float tdx = nextTarget.x - selfPos.x;
        float tdy = nextTarget.y - selfPos.y;
        float tdz = nextTarget.z - selfPos.z;

        if ((tdx*tdx + tdy*tdy + tdz* tdz) < WAYPOINT_RADIUS_SQ) {
            p_path.pop_front();
            if (!p_path.empty()) nextTarget = p_path.front();
        }

        p_self.SteerTowards(nextTarget, deltaTime);
    } else {
        // Fallback: אם אין נתיב בגרף, טוס ישירות לכיוון האויב (מתמטית)
        p_self.SteerTowards(enemyPos, deltaTime);
    }

    return AIStateType::PURSUIT;
}

Vector3 PursuitState::GetCurrentTargetFromAI() { // החזרה בערך
    if (!m_enemy) return p_self.GetPosition();
    return m_enemy->GetPosition();
}