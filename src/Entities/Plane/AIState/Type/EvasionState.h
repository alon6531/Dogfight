//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_EVASIONSTATE_H
#define DOGFIGHT_EVASIONSTATE_H

#define MAX_THRUST_EVASION 470.0f
// --- PickEscapeNode ---
#define ESCAPE_BOUNDARY_MARGIN_XZ           150.0f
#define ESCAPE_BOUNDARY_MARGIN_Y_TOP        100.0f
#define ESCAPE_BOUNDARY_MIN_Y                50.0f
#define ESCAPE_ALIGNMENT_TARGET_DOT          0.5f
#define ESCAPE_ALIGNMENT_SCALE               1.0f
#define ESCAPE_BOUNDARY_PENALTY           5000.0f
#define ESCAPE_DIST_SCORE_SCALE             20.0f
#define ESCAPE_CHAOS_MAX                  1600

// --- ShouldReturnToPursuit ---
#define PURSUIT_MIN_STATE_TIME               2.0f
#define PURSUIT_HEIGHT_ADVANTAGE_BONUS      20.0f
#define PURSUIT_SAFE_DIST_SQ_MULT            1.44f

// --- DrawDebugUI ---
#define DEBUG_WIN_PADDING                   20.0f
#define DEBUG_WIN_WIDTH                    300.0f
#define DEBUG_WIN_HEIGHT                   550.0f
#define DEBUG_STATE_COLOR_R                  0.0f
#define DEBUG_STATE_COLOR_G                255.0f
#define DEBUG_STATE_COLOR_B                  0.0f
#define DEBUG_STATE_COLOR_A                255.0f
#define DEBUG_REPLAN_MIN                     0.05f
#define DEBUG_REPLAN_MAX                     2.0f
#define DEBUG_PATH_COST_MIN                500.0f
#define DEBUG_PATH_COST_MAX              15000.0f
#define DEBUG_WEIGHT_STEP                    0.1f
#define DEBUG_WEIGHT_MIN                     0.0f
#define DEBUG_WEIGHT_MAX                    20.0f
#define DEBUG_ESCAPE_DIST_MIN              100.0f
#define DEBUG_ESCAPE_DIST_MAX             2000.0f
#define DEBUG_SAFE_DIST_SQ_MIN           10000.0f
#define DEBUG_SAFE_DIST_SQ_MAX         1000000.0f
#define DEBUG_PATH_NODES_MIN                 1
#define DEBUG_PATH_NODES_MAX                50
#define DEBUG_RESET_BTN_HEIGHT              30.0f


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
    float m_highGroundAdvantage = 150.0f;
    float m_minEscapeDist = 600.0f;
    float m_altitudeWeight = 8.5f;
    float m_distanceWeight = 1.2f;
    int m_candidateSamples = 150;
    float m_maxPathCost = 8000.0f;
    float m_escapeNodeReachedSq = 15000.0f;
    int m_maxEvasionNodes = 10;

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