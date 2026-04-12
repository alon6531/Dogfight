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

    Vector3 selfPos = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();

    // בדיקת הסטטוס של היריב
    AIStateType enemyState = m_enemy->GetCurrentStateType();

    // לוגיקת מעבר לבריחה (רק אם היריב בעמדת תקיפה/רדיפה)
    if (enemyState == AIStateType::PURSUIT || enemyState == AIStateType::PATROL) {
        Vector3 dirToMe = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
        float enemyAlignment = Vector3DotProduct(m_enemy->GetForward(), dirToMe);

        // אם היריב "עלי" ורודף אחרי, אני נשבר לבריחה
        if (enemyAlignment > 0.8f && Vector3Distance(selfPos, enemyPos) < 800.0f) {
            return AIStateType::EVASION;
        }
    }





    m_pathTimer += deltaTime;
    if (m_pathTimer > 0.0f || p_path.empty()) {
        m_pathTimer = 0;
        auto pathPoints = m_dStarLite->PlanPath(selfPos, enemyPos);
        p_path.clear();
        for (const auto& pt : pathPoints)
            p_path.push_back(pt);
    }

    if (!p_path.empty()) {

        Vector3 nextTarget = p_path.front();


        if (Vector3Distance(selfPos, nextTarget) < 40.0f) {
            p_path.pop_front();
            if (!p_path.empty()) nextTarget = p_path.front();
        }

        p_self.SteerTowards(nextTarget, deltaTime);
    } else {

        p_self.SteerTowards(enemyPos, deltaTime);
    }


    return AIStateType::PURSUIT;
}

Vector3 PursuitState::GetCurrentTargetFromAI() { // החזרה בערך
    if (!m_enemy) return p_self.GetPosition();
    return m_enemy->GetPosition();
}