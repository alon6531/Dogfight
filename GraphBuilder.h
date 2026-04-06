//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_GRAPHBUILDER_H
#define DOGFIGHT_GRAPHBUILDER_H


#endif //DOGFIGHT_GRAPHBUILDER_H

#include <vector>
#include "raylib.h"



// Represents an edge in the adjacency list
struct Edge {
    int target;      // Index of the destination waypoint
    float weight;    // Distance or traversal cost
};

// Represents a node (Waypoint) in the 3D aerial map
struct Node {
    int id;
    Vector3 position;            // 3D coordinates in the arena
    std::vector<Edge> neighbors; // List of reachable waypoints (Adjacency List)
};

struct Obstacle {
    Vector3 pos;
    float radius;
};

// The main container for the arena's navigation data
class NavigationGraph {
private:
    std::vector<Node> m_nodes;
    std::vector<std::vector<float>> distanceMatrix;

    int m_gridSizeX{}, m_gridSizeY{}, m_gridSizeZ{};



    std::vector<Matrix> m_nodeTransforms;
    std::vector<Model> m_nodeModels;
    bool m_isModelReady = false;

    static bool IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles);

    static bool IsPathBlocked(Vector3 start, Vector3 end, const std::vector<Obstacle> &obstacles);

public:
    NavigationGraph() = default;
    ~NavigationGraph() = default;

    void BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle>& obstacles);



    void BuildDistanceMatrix();

    float GetHeuristic(int nIdx, int targetIdx);


    //float GetHeuristic(int startNodeIdx, int targetNodeIdx);

    std::vector<Vector3> FindPathViaAStar(int startIdx, int targetIdx);

    int GetClosestNode(Vector3 position);

    void PrepareGPUData();

    void Draw(Vector3 cameraPos, float renderRadius) const;


    [[nodiscard]] const std::vector<Node>& nodes() const {
        return m_nodes;
    }

    [[nodiscard]] const std::vector<std::vector<float>>& distance_matrix() const {
        return distanceMatrix;
    }



};






