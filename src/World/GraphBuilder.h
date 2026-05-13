//
// Created by User on 03/04/2026.
//

#ifndef DOGFIGHT_GRAPHBUILDER_H
#define DOGFIGHT_GRAPHBUILDER_H
#include <functional>
#include <stdint.h>
#include <bits/basic_string.h>


#include <vector>
#include "raylib.h"

// --- IsPointBlocked ---
#define OBSTACLE_SAFETY_MARGIN      64.0f

// --- GetPathWeight ---
#define PATH_SAMPLE_MIN_COUNT       3
#define PATH_SAMPLE_INTERVAL        0.5f
#define PATH_WEIGHT_BLOCKED_MULT    100.0f
#define PATH_WEIGHT_CLEAR_MULT      1.0f

// --- BuildNodes / BuildEdges (getPosKey) ---
#define POS_KEY_OFFSET              10000
#define POS_KEY_MASK                0xFFFFF
#define POS_KEY_X_SHIFT             40
#define POS_KEY_Y_SHIFT             20
#define POS_KEY_GRID_EPSILON        0.1f

// --- BuildDistanceMatrix ---
#define LANDMARK_COUNT_START        0
#define LANDMARK_INVALID_DIST      -1.0f

// --- PrepareGPUData ---
#define NODE_CUBE_SIZE              0.2f
#define NODES_PER_MESH_CHUNK        500

// --- Draw ---
#define EDGE_FALLBACK_COLOR_R       180
#define EDGE_FALLBACK_COLOR_G       180
#define EDGE_FALLBACK_COLOR_B       180
#define EDGE_FALLBACK_COLOR_A       120
#define EDGE_HUE_MULTIPLIER         150.0f
#define EDGE_HUE_MODULO             360.0f
#define EDGE_SPATIAL_SCALE          0.1f
#define EDGE_SATURATION             0.9f
#define EDGE_VALUE                  0.9f
#define EDGE_BLOCKED_THRESHOLD      10.0f
#define EDGE_BLOCKED_COLOR_R        255
#define EDGE_BLOCKED_COLOR_G        0
#define EDGE_BLOCKED_COLOR_B        0
#define EDGE_BLOCKED_COLOR_A        50
#define EDGE_NORMAL_ALPHA           180



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
    Vector3 m_arenaSize;
    float m_spacing = 30.0f;
    std::unordered_map<uint64_t, int> m_posToId;

    std::vector<Model> m_nodeModels;
    bool m_isModelReady = false;

    static int IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles);

    static bool IsPathBlocked(Vector3 start, Vector3 end, const std::vector<Obstacle> &obstacles, const class Map *gameMap);

    static float GetPathWeight(Vector3 start, Vector3 end, const std::vector<Obstacle> &obstacles, const Map *gameMap);

    void BuildNodes(Vector3 arenaSize, const Map &gameMap, int &idCounter);

    void BuildEdges(const Map &gameMap,
                    const std::vector<Obstacle> &obstacles);

public:
    NavigationGraph() = default;
    ~NavigationGraph() = default;

    void BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles, const Map &gameMap);



    void BuildDistanceMatrix();

    [[nodiscard]] float GetHeuristic(int nIdx, int targetIdx) const;


    //float GetHeuristic(int startNodeIdx, int targetNodeIdx);


    int GetClosestNode(Vector3 position);

    int GetRandomNodeFarFrom(Vector3 position, Vector3 currentForward, float minDistance);


    void PrepareGPUData();

    void Draw(Vector3 cameraPos, float renderRadius) const;


    [[nodiscard]] const std::vector<Node>& GetNodes() const {
        return m_nodes;
    }

    [[nodiscard]] const std::vector<std::vector<float>>& distance_matrix() const {
        return distanceMatrix;
    }

    [[nodiscard]] Vector3 GetArenaSize() const {
        return m_arenaSize;
    }
};



#endif //DOGFIGHT_GRAPHBUILDER_H



