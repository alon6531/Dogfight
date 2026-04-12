//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_GRAPHBUILDER_H
#define DOGFIGHT_GRAPHBUILDER_H
#include <functional>
#include <bits/basic_string.h>


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

    static int IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles);

    static bool IsPathBlocked(Vector3 start, Vector3 end, const std::vector<Obstacle> &obstacles, const class Map *gameMap);

    void BuildNodes(Vector3 arenaSize, float spacing, const Map &gameMap, int &idCounter, std::unordered_map<std::string, int> &posToId);

    void BuildEdges(Vector3 arenaSize, float spacing, const Map &gameMap,
                    const std::vector<Obstacle> &obstacles, std::unordered_map<std::string, int> &posToId);

public:
    NavigationGraph() = default;
    ~NavigationGraph() = default;

    void BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles, const Map &gameMap);



    void BuildDistanceMatrix();

    [[nodiscard]] float GetHeuristic(int nIdx, int targetIdx) const;


    //float GetHeuristic(int startNodeIdx, int targetNodeIdx);


    int GetClosestNode(Vector3 position);

    int GetRandomNodeFarFrom(Vector3 position, float minDistance) const;


    void PrepareGPUData();

    void Draw(Vector3 cameraPos, float renderRadius) const;


    [[nodiscard]] const std::vector<Node>& GetNodes() const {
        return m_nodes;
    }

    [[nodiscard]] const std::vector<std::vector<float>>& distance_matrix() const {
        return distanceMatrix;
    }



};



#endif //DOGFIGHT_GRAPHBUILDER_H



