#ifndef DSTARLITE_H
#define DSTARLITE_H

#include <cmath>


#include <unordered_map>
#include <queue>
#include "raylib.h"

class NavigationGraph;

struct DStarNode {
    float g   = 1e9f;
    float rhs = 1e9f;
    int   gen = 0;
};


class DStarLite {
public:
    DStarLite(NavigationGraph& graph);


    std::vector<Vector3> PlanPath(Vector3 startPos, Vector3 targetPos);

private:
    NavigationGraph& m_graph;
    std::unordered_map<int, DStarNode> m_nodes;
    

    struct OpenEntry {
        std::pair<float,float> key;
        int node;
        int gen;
        bool operator>(const OpenEntry& o) const { return key > o.key; }
    };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>> m_openSet;

    float m_kM = 0;
    int m_startNodeIdx = -1;
    int m_targetNodeIdx = -1;

    std::pair<float, float> CalculateKey(int s);
    void UpdateVertex(int u);
    void ComputeShortestPath();
};

#endif