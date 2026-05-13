//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_FUELSTATE_H
#define DOGFIGHT_FUELSTATE_H
#include "../Base/AIState.h"

// --- Constructor (landing sequence) ---
#define LANDING_APPROACH_DIST           150.0f
#define LANDING_APPROACH_HEIGHT_OFFSET   10.0f
#define LANDING_TOUCHDOWN_DIST          120.0f

// --- Update ---
#define LANDING_WAYPOINT_RADIUS          10.0f
#define LANDING_DECEL_FACTOR              0.985f
#define LANDING_THRUST_FRACTION           0.2f
#define REFUEL_RATE                    5000.0f


class FuelState : public AIState{
private:
public:
    FuelState(Plane &self, NavigationGraph &navGraph);

    ~FuelState() override = default;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_FUELSTATE_H