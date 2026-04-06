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
    const Mesh mesh = GenMeshCube(1.0f, 0.5f, 2.0f);
    m_model = LoadModelFromMesh(mesh);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = BLUE;
    m_acceleration = Vector3();

}

void Plane::SetDestination(int targetNodeIdx, NavigationGraph& graph) {

    int startNodeIdx = graph.GetClosestNode(m_position);


    m_path = graph.FindPathViaAStar(startNodeIdx, targetNodeIdx);


    m_targetPathIdx = 0;
}

void Plane::Update(float deltaTime) {
    // Check if the path is empty or if we have already reached the final destination
    if (m_path.empty() || m_targetPathIdx >= m_path.size()) return;

    // Get the position of the current target waypoint in the sequence
    Vector3 targetPos = m_path[m_targetPathIdx];

    // Calculate the direction vector and the remaining distance to the current waypoint
    Vector3 direction = Vector3Subtract(targetPos, m_position);
    float distToTarget = Vector3Length(direction);

    // If close enough to the current waypoint, switch to the next one in the path
    if (distToTarget < 0.5f) {
        m_targetPathIdx++;
    }
    else {
        // Normalize the direction to ensure consistent movement speed
        direction = Vector3Normalize(direction);

        // Update the plane's position based on direction, constant speed, and frame time
        m_position = Vector3Add(m_position, Vector3Scale(direction, m_speed * deltaTime));

        // Optional: Update the model transformation to face the direction of flight
        // m_model.transform = MatrixLookAt(m_position, targetPos, {0, 1, 0});
    }
}

void Plane::Draw() const {
    DrawModelEx(m_model, m_position, { 0, 1, 0 }, 0, { 1.0f, 1.0f, 1.0f }, WHITE);
}
