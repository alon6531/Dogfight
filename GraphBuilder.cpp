//
// Created by User on 03/04/2026.
//
#include "GraphBuilder.h"

#include "raymath.h"


NavigationGraph BuildGraphFromMap(const std::vector<Vector3> &rawPositions) {
    NavigationGraph graph;
    const unsigned int V = rawPositions.size();

    // Initialize nodes with positions
    for (int i = 0; i < V; i++) {
        graph.nodes.push_back({ i, rawPositions[i], {} });
    }

    // Build Adjacency List
    // Defines the max distance between two waypoints to consider them connected
    const float connectionRadius = 100.0f;

    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (i == j) continue; // Skip self-connection

            float dist = Vector3Distance(graph.nodes[i].position, graph.nodes[j].position);

            // Connect waypoints if they are within range
            // In a real scenario, you would also add a Line-of-Sight check here
            if (dist < connectionRadius) {
                graph.nodes[i].neighbors.push_back({ j, dist });
            }
        }
    }

    return graph;
}

void DrawNavigationGraph(const NavigationGraph& graph) {
    for (const auto& node : graph.nodes) {
        // Draw the waypoint node
        DrawSphere(node.position, 0.5f, RED);

        // Draw lines to all neighbors
        for (const auto& edge : node.neighbors) {
            DrawLine3D(node.position, graph.nodes[edge.target].position, Fade(GRAY, 0.5f));
        }
    }
}

std::vector<Vector3> GenerateRandomWaypoints(int count, Vector3 arenaSize) {
    std::vector<Vector3> positions;

    for (int i = 0; i < count; i++) {
        positions.push_back({
            (float)GetRandomValue(-arenaSize.x / 2, arenaSize.x / 2), // X position
            (float)GetRandomValue(10, arenaSize.y),                  // Y (Altitude - keep above ground)
            (float)GetRandomValue(-arenaSize.z / 2, arenaSize.z / 2)  // Z position
        });
    }

    return positions;
}