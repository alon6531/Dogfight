//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_PURSUITSTATE_H
#define DOGFIGHT_PURSUITSTATE_H
#include <memory>

#include "../Base/AIState.h"

// --- Update: evasion trigger ---
#define PURSUIT_EVASION_ALIGNMENT_THRESHOLD     0.95f
#define PURSUIT_EVASION_DIST_SQ_MAX         400000.0f
#define PURSUIT_EVASION_HEIGHT_MARGIN           20.0f

// --- Update: replan ---
#define PURSUIT_REPLAN_INTERVAL                  0.2f
#define PURSUIT_TARGET_HEIGHT_OFFSET            50.0f
#define PURSUIT_MAX_PATH_NODES                   8

// --- Update: waypoint advance ---
#define PURSUIT_WAYPOINT_RADIUS_SQ           10000.0f


class PursuitState : public AIState{
private:
    std::shared_ptr<Plane> m_enemy= nullptr;
    std::unique_ptr<DStarLite> m_dStarLite= nullptr;
    float m_pathTimer = 0.0f;

public:
    PursuitState(Plane &self, NavigationGraph &navGraph, std::shared_ptr<Plane> enemy);

    ~PursuitState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_PURSUITSTATE_H