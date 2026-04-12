//
// Created by User on 12/04/2026.
//

#include "FuelState.h"

#include "../../Plane.h"

FuelState::~FuelState() = default;

AIStateType FuelState::Update(float deltaTime) {
    if (p_self.GetFuel() < MAX_FUEL)
        p_self.SetFuel(p_self.GetFuel() + deltaTime * 10000);
    else
        return AIStateType::PATROL;

    return AIStateType::FUEL;
}

Vector3 FuelState::GetCurrentTargetFromAI() {
    return p_self.GetPosition();
}
