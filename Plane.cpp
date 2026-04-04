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

void Plane::Update(const float deltaTime, NavigationGraph& graph) {

    Vector3 closestNode = {};
    float minDistance = 999999.0f;

    m_acceleration = Vector3Add(m_acceleration, Vector3Scale(m_velocity, deltaTime));

    for (auto& node : graph.nodes()) {
        float dist = Vector3Distance(m_acceleration, node.position);
        if (dist < minDistance) {
            minDistance = dist;
            closestNode = Vector3(node.position);
        }
    }
    m_position = closestNode;

}

void Plane::Draw() const {
    DrawModelEx(m_model, m_position, { 0, 1, 0 }, 0, { 1.0f, 1.0f, 1.0f }, WHITE);
}
