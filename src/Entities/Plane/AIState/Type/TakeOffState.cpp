//
// Created by User on 12/04/2026.
//

#include "TakeOffState.h"

#include "raymath.h"
#include "../../Plane.h"

TakeOffState::TakeOffState(Plane &self, NavigationGraph &navGraph)
    : AIState(self, navGraph) {

    float requiredSpeed = (GRAVITY * MASS) / 0.85f;
    float acceleration = MAX_THRUST / MASS;
    float minTakeoffDist = (requiredSpeed * requiredSpeed) / (2.0f * acceleration);

    Vector3 currentPos = p_self.GetPosition();
    Vector3 forward = p_self.GetForward();


    Vector3 stayOnGround = Vector3Add(currentPos, Vector3Scale(forward, 40.0f));
    stayOnGround.y = currentPos.y - 5;

    Vector3 endOfRunway = Vector3Add(currentPos, Vector3Scale(forward, minTakeoffDist * 2.8f));
    endOfRunway.y = currentPos.y;

    Vector3 liftOffPoint = Vector3Add(endOfRunway, Vector3Scale(forward, 200.0f));
    liftOffPoint.y += 40.0f;


    p_path.push_back(stayOnGround);
    p_path.push_back(endOfRunway);
    p_path.push_back(liftOffPoint);

}

TakeOffState::~TakeOffState() = default;

AIStateType TakeOffState::Update(float deltaTime) {
    if (p_path.empty()) return AIStateType::IDLE;

    Vector3 targetPoint = p_path.front();
    float distance = Vector3Distance(p_self.GetPosition(), targetPoint);

    if (distance < 20.0f) {

        p_path.pop_front();
        if (p_path.empty()) return AIStateType::PATROL;

        targetPoint = p_path.front();
    }

    p_self.SteerTowards(targetPoint, deltaTime);
    return AIStateType::TAKEOFF;
}

Vector3 TakeOffState::GetCurrentTargetFromAI() {
    if (p_path.empty()) return p_self.GetPosition();
    return p_path.back();
}
