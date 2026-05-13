//
// Created by User on 12/04/2026.
//

#ifndef DOGFIGHT_TAKEOFFSTATE_H
#define DOGFIGHT_TAKEOFFSTATE_H
#include "../Base/AIState.h"

// --- Constructor (takeoff sequence) ---
#define TAKEOFF_LIFT_COEFFICIENT            0.85f
#define TAKEOFF_RUNWAY_DIST_MULTIPLIER       2.8f
#define TAKEOFF_GROUND_ROLL_DIST            40.0f
#define TAKEOFF_LIFTOFF_EXTEND_DIST        200.0f
#define TAKEOFF_LIFTOFF_HEIGHT_OFFSET       40.0f
#define TAKEOFF_FORWARD_DIR                 { 1, 0, 0 }

// --- Update ---
#define TAKEOFF_WAYPOINT_RADIUS             20.0f

class TakeOffState : public AIState{
public:
    TakeOffState(Plane &self, NavigationGraph &navGraph);


    ~TakeOffState() override;

    AIStateType Update(float deltaTime) override;

    Vector3 GetCurrentTargetFromAI() override;
};


#endif //DOGFIGHT_TAKEOFFSTATE_H