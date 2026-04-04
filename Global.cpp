//
// Created by User on 04/04/2026.
//

#include "Global.h"
#include <queue>
#include <limits>
#include "GraphBuilder.h"


void ComputeDijkstra(NavigationGraph& graph, int startNodeIdx, std::vector<float> &outDistances) {
    // Validation: Ensure the output vector matches the current graph size
    if (outDistances.size() != graph.nodes().size()) {
        outDistances.resize(graph.nodes().size());
    }

    // 1. Initialize all nodes with "infinity" distance
    std::fill(outDistances.begin(), outDistances.end(), std::numeric_limits<float>::max());

    // Boundary check for the starting waypoint
    if (startNodeIdx < 0 || startNodeIdx >= (int)graph.nodes().size()) return;

    outDistances[startNodeIdx] = 0.0f;

    // 2. Min-Priority Queue for efficient frontier exploration
    // Format: std::pair<float, int> where .first is distance and .second is node ID
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> pq;
    pq.push({0.0f, startNodeIdx});

    while (!pq.empty()) {
        float d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Standard Dijkstra optimization: skip processing if a shorter path was already finalized
        if (d > outDistances[u]) continue;

        // 3. Relax edges by traversing the adjacency list of node 'u'
        for (const auto& edge : graph.nodes()[u].neighbors) {
            int v = edge.target;
            float weight = edge.weight;

            // If a shorter path to neighbor 'v' is discovered via 'u'
            if (outDistances[u] + weight < outDistances[v]) {
                outDistances[v] = outDistances[u] + weight;
                pq.push({outDistances[v], v});
            }
        }
    }

    // 4. Mark unreachable nodes as -1.0f for ATL heuristic compatibility
    for (float& d : outDistances) {
        if (d == std::numeric_limits<float>::max()) {
            d = -1.0f;
        }
    }
}
