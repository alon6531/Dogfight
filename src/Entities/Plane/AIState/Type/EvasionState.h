//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_EVASIONSTATE_H
#define DOGFIGHT_EVASIONSTATE_H

#define MAX_THRUST_EVASION 470.0f

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

    float m_replanInterval = 0.1f;
    float m_waypointRadiusSq = 5000.0f;
    float m_safeDistSq = 490000.0f;
    float m_highGroundAdvantage = 150.0f;
    float m_minEscapeDist = 600.0f;
    float m_altitudeWeight = 8.5f;
    float m_distanceWeight = 1.2f;
    int m_candidateSamples = 150;
    float m_maxPathCost = 8000.0f;
    float m_escapeNodeReachedSq = 15000.0f;
    int m_maxEvasionNodes = 3;

    float m_stateTime = 0.0f;


    // Finds a scored escape node: penalises low altitude and proximity to enemy
    int PickEscapeNode(Vector3 selfPos, Vector3 enemyPos) const;
    bool ShouldReturnToPursuit(Vector3 selfPos, Vector3 enemyPos) const;
    void ResetState();
    void UpdateEscapeTarget(Vector3 selfPos, Vector3 enemyPos);
    void ReplanPathIfNeeded(Vector3 selfPos);
    void NavigatePath(Vector3 selfPos, float deltaTime, Vector3 enemyPos);
    void ExecuteEmergencyClimb(Vector3 selfPos, Vector3 enemyPos, float deltaTime);

};


#endif //DOGFIGHT_EVASIONSTATE_H