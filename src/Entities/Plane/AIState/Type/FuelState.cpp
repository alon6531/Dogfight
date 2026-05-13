#include "FuelState.h"

#include <iostream>

#include "raymath.h"
#include "../../Plane.h"



FuelState::FuelState(Plane &self, NavigationGraph &navGraph) : AIState(self, navGraph) {

    Vector3 basePos   = p_self.GetBasePos();
    Vector3 selfPos   = p_self.GetPosition();
    Vector3 dirToBase = Vector3Normalize(Vector3Subtract(basePos, selfPos));

    Vector3 approachPoint = Vector3Subtract(basePos, Vector3Scale(dirToBase, LANDING_APPROACH_DIST));
    approachPoint.y = basePos.y + LANDING_APPROACH_HEIGHT_OFFSET;

    Vector3 touchdownPoint = Vector3Subtract(basePos, Vector3Scale(dirToBase, LANDING_TOUCHDOWN_DIST));
    touchdownPoint.y = basePos.y;

    Vector3 stopPoint = basePos;

    p_path.push_back(approachPoint);
    p_path.push_back(touchdownPoint);
    p_path.push_back(stopPoint);
}

AIStateType FuelState::Update(float deltaTime) {

    std::cout << "FuelState::Update | Path Size: " << p_path.size() << " | Fuel: " << p_self.GetFuel() << std::endl;

    Vector3 selfPos     = p_self.GetPosition();
    Vector3 targetPoint = p_path.front();
    float distance      = Vector3Distance(selfPos, targetPoint);

    if (p_path.size() == 1) {
        p_self.SetThrust(0);
        p_self.SetVelocity(Vector3Scale(p_self.GetVelocity(), LANDING_DECEL_FACTOR));
        p_self.SteerTowards(targetPoint, deltaTime);
    }

    if (p_path.empty()) {
        p_self.SetVelocity({0, 0, 0});
        if (p_self.GetFuel() < MAX_FUEL)
            p_self.SetFuel(p_self.GetFuel() + deltaTime * REFUEL_RATE);
        else {
            p_self.SetTargetPos(Vector3());
            return AIStateType::TAKEOFF;
        }
    }

    if (distance < LANDING_WAYPOINT_RADIUS) {
        p_path.pop_front();
    } else {
        p_self.SetThrust(MAX_THRUST * LANDING_THRUST_FRACTION);
        p_self.SteerTowards(targetPoint, deltaTime);
    }

    return AIStateType::FUEL;
}

Vector3 FuelState::GetCurrentTargetFromAI() {
    if (p_path.empty()) return p_self.GetPosition();
    return p_path.front();
}