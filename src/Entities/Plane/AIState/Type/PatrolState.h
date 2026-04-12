//
// Created by User on 11/04/2026.
//

#ifndef DOGFIGHT_PATROLSTATE_H
#define DOGFIGHT_PATROLSTATE_H
#include <vector>

#include "raylib.h"
#include "../Base/AIState.h"



class PatrolState : public AIState{
private:
    Vector3 m_targetPos;
    AStar m_astar;
public:
    PatrolState(Plane& self, NavigationGraph& navGraph, const Vector3 &targetPos);


    ~PatrolState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_PATROLSTATE_H