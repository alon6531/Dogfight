//
// Created by User on 03/04/2026.
//

#include "Plane.h"

#include <iostream>
#include <memory>
#include <ostream>

#include "raymath.h"
#include "GraphBuilder.h"

Plane::Plane(const Vector3 &position, const Vector3 &velocity, const Vector3 &acceleration)
    : m_position(position),
      m_velocity(velocity),
      m_acceleration(acceleration) {
    const Mesh mesh = GenMeshCube(5.0f, 2.5f, 10.0f);
    m_model = LoadModelFromMesh(mesh);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = PINK;
    m_acceleration = Vector3();

}

void Plane::SetDestination(int targetNodeIdx, NavigationGraph& graph) {

    int startNodeIdx = graph.GetClosestNode(m_position);

    if (startNodeIdx != -1)
        m_path = graph.FindPathViaAStar(startNodeIdx, targetNodeIdx);


    m_targetPathIdx = 0;
}

void Plane::Update(float deltaTime) {

    if (!m_path.empty() || m_targetPathIdx < m_path.size()) {
        Vector3 targetPos = m_path[m_targetPathIdx];

        // Calculate the direction vector and the remaining distance to the current waypoint
        Vector3 direction = Vector3Subtract(targetPos, m_position);
        float distToTarget = Vector3Length(direction);

        if (distToTarget < 0.5f) {
            m_targetPathIdx++;
        }
        else {
            // Normalize the direction to ensure consistent movement speed
            direction = Vector3Normalize(direction);

            m_position = Vector3Add(m_position, Vector3Scale(direction, m_speed * deltaTime));

        }

    }
}

void Plane::Draw() const {
    DrawModelEx(m_model, m_position, { 0, 1, 0 }, 0, { 1.0f, 1.0f, 1.0f }, WHITE);
}
