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

    Vector3 selfPos  = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();
    float distSq     = Vector3DistanceSqr(selfPos, enemyPos);

    Vector3 dirToMe      = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
    float enemyAlignment = Vector3DotProduct(m_enemy->GetForward(), dirToMe);

    if (enemyAlignment > PURSUIT_EVASION_ALIGNMENT_THRESHOLD &&
        distSq < PURSUIT_EVASION_DIST_SQ_MAX &&
        selfPos.y < enemyPos.y + PURSUIT_EVASION_HEIGHT_MARGIN) {
        p_path.clear();
        return AIStateType::EVASION;
    }

    m_pathTimer += deltaTime;
    if (m_pathTimer >= PURSUIT_REPLAN_INTERVAL || p_path.empty()) {
        m_pathTimer = 0.0f;

        Vector3 pursuitTarget = enemyPos;
        pursuitTarget.y += PURSUIT_TARGET_HEIGHT_OFFSET;

        auto pathPoints = m_dStarLite->PlanPath(selfPos, pursuitTarget);
        if (!pathPoints.empty()) {
            p_path.clear();
            int nodesToCopy = fmin((int)pathPoints.size(), PURSUIT_MAX_PATH_NODES);
            for (int i = 0; i < nodesToCopy; i++) {
                p_path.push_back(pathPoints[i]);
            }
        }
    }

    if (!p_path.empty()) {
        Vector3 nextWaypoint = p_path.front();

        if (Vector3DistanceSqr(selfPos, nextWaypoint) < PURSUIT_WAYPOINT_RADIUS_SQ) {
            p_path.pop_front();
            if (!p_path.empty()) nextWaypoint = p_path.front();
        }

        p_self.SteerTowards(nextWaypoint, deltaTime);
    } else {
        Vector3 directTarget = enemyPos;
        directTarget.y += PURSUIT_TARGET_HEIGHT_OFFSET;
        p_self.SteerTowards(directTarget, deltaTime);
    }

    return AIStateType::PURSUIT;
}

Vector3 PursuitState::GetCurrentTargetFromAI() {
    if (!m_enemy) return p_self.GetPosition();
    return m_enemy->GetPosition();
}