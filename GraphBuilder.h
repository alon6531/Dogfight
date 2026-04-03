//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_GRAPHBUILDER_H
#define DOGFIGHT_GRAPHBUILDER_H
#include <vector>

#endif //DOGFIGHT_GRAPHBUILDER_H

#include "raylib.h"


// Represents an edge in the adjacency list
struct Edge {
    int target;      // Index of the destination waypoint
    float weight;    // Distance or traversal cost
};

// Represents a node (Waypoint) in the 3D aerial map
struct Waypoint {
    int id;
    Vector3 position;            // 3D coordinates in the arena
    std::vector<Edge> neighbors; // List of reachable waypoints (Adjacency List)
};

// The main container for the arena's navigation data
struct NavigationGraph {
    std::vector<Waypoint> nodes;
    std::vector<std::vector<float>> distanceMatrix; // Pre-calculated costs (Floyd-Warshall)
};

extern NavigationGraph BuildGraphFromMap(const std::vector<Vector3>& rawPositions);


extern void DrawNavigationGraph(const NavigationGraph& graph);

extern std::vector<Vector3> GenerateRandomWaypoints(int count, Vector3 arenaSize);