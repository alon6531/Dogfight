//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_PURSUITSTATE_H
#define DOGFIGHT_PURSUITSTATE_H
#include <memory>

#include "../Base/AIState.h"


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