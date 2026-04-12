//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_TAKEOFFSTATE_H
#define DOGFIGHT_TAKEOFFSTATE_H
#include "../Base/AIState.h"


class TakeOffState : public AIState{
public:
    TakeOffState(Plane &self, NavigationGraph &navGraph);


    ~TakeOffState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_TAKEOFFSTATE_H