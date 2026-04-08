//
// Created by User on 03/04/2026.
//

#include "Plane.h"

#include <iostream>
#include <memory>
#include <ostream>

#include "DStarLite.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include "MPCController.h"

Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration, const Color& color, NavigationGraph &graph)
    : m_graph(graph),
      m_position(position),
      m_velocity(velocity),
      m_acceleration(acceleration) {
    const Mesh mesh = GenMeshCube(5.0f, 2.5f, 10.0f);
    m_model = LoadModelFromMesh(mesh);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = color;
    m_acceleration = Vector3();
    CurrentState = AIState::IDLE;
    CurrentEvent = AIEvent::NONE;

    m_dstar = std::make_unique<DStarLite>(m_graph);

}

void Plane::SetDestinationViaAStar(int targetNodeIdx) {
    int startNodeIdx = m_graph.GetClosestNode(m_position);

    if (startNodeIdx != -1) {
        m_path = m_graph.FindPathViaAStar(startNodeIdx, targetNodeIdx);


        m_rotation = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
    }
}

void Plane::Update(const Vector3& enemyPos,float deltaTime, const MPCController &mpc) {

    if (GetCurrentState() == AIState::PATROL) {
        UpdatePatrol(deltaTime, mpc);

    }
    if (GetCurrentState() == AIState::PURSUIT) {
        UpdatePursuit(enemyPos, deltaTime, mpc);
    }
}

void Plane::UpdatePatrol(float deltaTime, const MPCController &mpc) {
    // 1. Handle Empty Path (Deceleration state)
    if (m_path.empty()) {


        return;
    }
    std::cout << "pos: " << m_position.x << " " << m_position.y << " " << m_position.z << std::endl;
    // 2. Establish Orientation
    // If the plane is stationary, assume it's facing its current rotation or world forward
    Vector3 forward = (Vector3Length(m_velocity) > 0.1f)
                      ? Vector3Normalize(m_velocity)
                      : Vector3{ sinf(m_rotation * DEG2RAD), 0, cosf(m_rotation * DEG2RAD) };

    // 3. Path Pruning (Waypoint Management)
    // We check if the current target waypoint is reached or behind us
    const float arrivalThreshold = 30.0f;

    while (m_path.size() > 1) {
        Vector3 toWaypoint = Vector3Subtract(m_path[0], m_position);
        float distance = Vector3Length(toWaypoint);

        if (distance < arrivalThreshold) {
            m_path.erase(m_path.begin());
        } else {
            break;
        }
    }

    // 4. Steering and Movement
    if (!m_path.empty()) {
        // MPC requires a horizon; if only one point left, duplicate it to maintain path shape
        if (m_path.size() == 1) {
            m_path.push_back(m_path[0]);
        }

        // Calculate steering vector from MPC
        Vector3 steerDir = mpc.CalculateSteering(m_position, m_velocity, m_path);

        // Fallback for invalid MPC output
        if (isnan(steerDir.x) || Vector3Length(steerDir) < 0.001f) {
            steerDir = forward;
        }

        // Pitch Constraints (Keep the plane from diving/climbing too steeply)
        const float maxPitch = 0.3f;
        steerDir.y = Clamp(steerDir.y, -maxPitch, maxPitch);
        steerDir = Vector3Normalize(steerDir);

        // Apply Velocity and Physics
        Vector3 targetVelocity = Vector3Scale(steerDir, m_speed);
        m_velocity = Vector3Lerp(m_velocity, targetVelocity, 3.0f * deltaTime);

        Vector3 moveStep = Vector3Scale(m_velocity, deltaTime);

        // Safety check to prevent "teleporting" due to huge deltaTimes or NaN
        if (!isnan(moveStep.x) && Vector3Length(moveStep) < 100.0f) {
            m_position = Vector3Add(m_position, moveStep);
        }

        // 5. Visual Rotation (Yaw)
        // Lerp the visual rotation toward the movement direction
        if (Vector3Length(m_velocity) > 0.5f) {
            float targetYaw = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
            m_rotation = Lerp(m_rotation, targetYaw, 5.0f * deltaTime);
        }
    }
}

void Plane::UpdatePursuit(const Vector3 &enemyPos, float deltaTime, const MPCController &mpc) {



        m_path = m_dstar->PlanPath(m_position, PredictEnemyPos(enemyPos, deltaTime));


        if (m_path.size() > 1) {
            Vector3 forward = Vector3Normalize(m_velocity);

            while (m_path.size() > 2) {
                Vector3 toNode = Vector3Subtract(m_path[0], m_position);
                float dist = Vector3Length(toNode);


                float dot = Vector3DotProduct(forward, Vector3Normalize(toNode));

                if (dot < 0.1f || dist < 35.0f) {
                    m_path.erase(m_path.begin());
                } else {
                    break;
                }
            }


            //static MPCController mpc({12, 0.15f, 50.0f, 2.5f, 1.0f});
            Vector3 steerDir = mpc.CalculateSteering(m_position, m_velocity, m_path);


            if (steerDir.y > 0.4f) steerDir.y = 0.4f;
            steerDir = Vector3Normalize(steerDir);


            Vector3 targetVelocity = Vector3Scale(steerDir, m_speed);
            m_velocity = Vector3Lerp(m_velocity, targetVelocity, 4.0f * deltaTime);

            m_position = Vector3Add(m_position, Vector3Scale(m_velocity, deltaTime));


            if (Vector3Length(m_velocity) > 0.5f) {
                float targetRot = atan2f(m_velocity.x, m_velocity.z) * RAD2DEG;
                m_rotation = Lerp(m_rotation, targetRot, 5.0f * deltaTime);
            }
        }
}


Vector3 Plane::PredictEnemyPos(const Vector3 &enemyPos, float deltaTime) {

    if (deltaTime <= 0.0001f) return enemyPos;


    Vector3 rawEnemyVel = Vector3Scale(Vector3Subtract(enemyPos, m_lastEnemyPos), 1.0f / deltaTime);
    m_lastEnemyPos = enemyPos;


    m_smoothedEnemyVel = Vector3Lerp(m_smoothedEnemyVel, rawEnemyVel, 10.0f * deltaTime);

    float distToEnemy = Vector3Distance(m_position, enemyPos);

    float speed = (m_speed > 0.1f) ? m_speed : 10.0f;
    float timeToIntercept = distToEnemy / speed;

    float lookAheadTime = fminf(timeToIntercept, 3.0f);
    Vector3 predictedPos = enemyPos;

    if (Vector3Length(m_smoothedEnemyVel) > 0.1f) {
        predictedPos = Vector3Add(enemyPos, Vector3Scale(m_smoothedEnemyVel, lookAheadTime));
    }


    if (isnan(predictedPos.x)) return enemyPos;

    return predictedPos;
}


void Plane::Draw() const {
    DrawModelEx(m_model, m_position, { 0, 1, 0 }, m_rotation, { 1.0f, 1.0f, 1.0f }, WHITE);
}
