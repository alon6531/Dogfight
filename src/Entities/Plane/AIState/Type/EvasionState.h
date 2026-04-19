//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_EVASIONSTATE_H
#define DOGFIGHT_EVASIONSTATE_H

#define MAX_THRUST_EVASION 270.0f

#include "../Base/AIState.h"

#include "../../../../Navigation/DStarLite.h"
#include <memory>
#include <deque>

class EvasionState : public AIState {
public:
    EvasionState(Plane& self, NavigationGraph& navGraph, std::shared_ptr<Plane> enemy);
    ~EvasionState() override;

    AIStateType Update(float deltaTime) override;
    Vector3 GetCurrentTargetFromAI() override;
    void DrawDebugUI() override;

private:
    std::shared_ptr<Plane> m_enemy;
    std::unique_ptr<DStarLite> m_dStarLite;

    Vector3  m_currentDir    = { 0, 0, 0 };
    float    m_pathTimer     = 999.0f;   // force first replan immediately
    int      m_escapeNodeIdx = -1;

    float m_replanInterval = 0.25f;
    float m_waypointRadiusSq = 10000.0f;
    float m_safeDistSq = 160000.0f;
    float m_highGroundAdvantage = 80.0f;
    float m_minEscapeDist = 300.0f;
    float m_altitudeWeight = 2.5f;
    float m_distanceWeight = 1.0f;
    int   m_candidateSamples = 80;
    float m_maxPathCost = 5000.0f;
    float m_escapeNodeReachedSq = 40000.0f;

    // Finds a scored escape node: penalises low altitude and proximity to enemy
    int PickEscapeNode(Vector3 selfPos, Vector3 enemyPos) const;
};


#endif //DOGFIGHT_EVASIONSTATE_H