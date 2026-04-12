//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_FUELSTATE_H
#define DOGFIGHT_FUELSTATE_H
#include "../Base/AIState.h"


class FuelState : public AIState{
public:
    FuelState(Plane &self, NavigationGraph &navGraph)
        : AIState(self, navGraph) {
    }

    ~FuelState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_FUELSTATE_H