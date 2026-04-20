#include "FuelState.h"

#include <iostream>

#include "raymath.h"
#include "../../Plane.h"

FuelState::FuelState(Plane &self, NavigationGraph &navGraph) : AIState(self, navGraph) {
    if (!p_path.empty()) return;

    Vector3 basePos = p_self.GetBasePos();
    Vector3 selfPos = p_self.GetPosition();
    Vector3 dirToBase = Vector3Normalize(Vector3Subtract(basePos, selfPos));

    // 1. גישה: הכי רחוקה - 100 מטר לפני הבסיס, בגובה 10
    Vector3 approachPoint = Vector3Subtract(basePos, Vector3Scale(dirToBase, 150.0f));
    approachPoint.y = basePos.y + 10.0f;

    // 2. נגיעה: קרובה יותר - 50 מטר מהבסיס, בגובה הקרקע
    Vector3 touchdownPoint = Vector3Subtract(basePos, Vector3Scale(dirToBase, 120.0f));
    touchdownPoint.y = basePos.y;

    // 3. עצירה: הבסיס עצמו (0 מטר)
    Vector3 stopPoint = basePos;


    p_path.push_back(approachPoint);
    p_path.push_back(touchdownPoint);
    p_path.push_back(stopPoint);
}

AIStateType FuelState::Update(float deltaTime) {
    // לוג לבדיקה
    std::cout << "FuelState::Update | Path Size: " << p_path.size() << " | Fuel: " << p_self.GetFuel() << std::endl;


    Vector3 selfPos = p_self.GetPosition();
    Vector3 targetPoint = p_path.front();
    float distance = Vector3Distance(selfPos, targetPoint);

    // טיפול בנקודה האחרונה (עצירה ותדלוק)
    if (p_path.size() == 1) {
        p_self.SetThrust(0);
        p_self.SetVelocity(Vector3Scale(p_self.GetVelocity(), 0.985f)); // בלימה
        p_self.SteerTowards(targetPoint, deltaTime);
    }



    if (p_path.empty()) {
        p_self.SetVelocity({0, 0, 0});
        if (p_self.GetFuel() < MAX_FUEL)
            p_self.SetFuel(p_self.GetFuel() + deltaTime * 5000);
        else {
            p_self.SetTargetPos(Vector3());
            return AIStateType::TAKEOFF;
        }
    }


    // ניווט לנקודות 3 ו-2
    if (distance < 10.0f) {
        p_path.pop_front();
    } else {
        p_self.SetThrust(MAX_THRUST * 0.2f);
        p_self.SteerTowards(targetPoint, deltaTime);

    }

    return AIStateType::FUEL;

}

Vector3 FuelState::GetCurrentTargetFromAI() {
    if (p_path.empty()) return p_self.GetPosition();
    return p_path.front();
}