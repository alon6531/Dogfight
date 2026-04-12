//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_EVASIONSTATE_H
#define DOGFIGHT_EVASIONSTATE_H

#define MAX_THRUST_EVASION 50.0f

#include <memory>
#include <utility>

#include "../Base/AIState.h"


class EvasionState : public AIState{
private:
    std::shared_ptr<Plane> m_enemy= nullptr;
    std::unique_ptr<DStarLite> m_dStarLite= nullptr;
    Vector3 m_currentDir = {};


public:
    EvasionState(Plane &self, NavigationGraph &navGraph, std::shared_ptr<Plane> enemy);


    ~EvasionState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_EVASIONSTATE_H