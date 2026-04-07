//
// Created by User on 03/04/2026.
//
#include "GraphBuilder.h"

#include <algorithm>
#include <map>

#include "rlgl.h"
#include "raymath.h"
#include <omp.h>
#include <queue>
#include <string>
#include <unordered_map>
#include "Global.h"
#include "Map.h"

int NavigationGraph::IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles) {
    for (const auto& obs : obstacles) {
        float distSq = Vector3DistanceSqr(p, obs.pos);
        if (distSq < (obs.radius * obs.radius)) {
            return 1;
        }
    }
    return 0;
}

bool NavigationGraph::IsPathBlocked(Vector3 start, Vector3 end, const std::vector<Obstacle>& obstacles, const Map* gameMap) {
    float distance = Vector3Distance(start, end);
    int samples = std::max(10, (int)(distance / 1.0f));

    for (int i = 1; i < samples; i++) {
        float t = (float)i / samples;
        Vector3 checkPoint = Vector3Lerp(start, end, t);

        if (IsPointBlocked(checkPoint, obstacles)) return true;
        if (gameMap != nullptr && gameMap->IsBelowGround(checkPoint)) return true;
    }
    return false;
}

void NavigationGraph::BuildNodes(Vector3 arenaSize, float spacing, const Map& gameMap, int& idCounter, std::unordered_map<std::string, int>& posToId) {
    m_nodes.clear();

    auto getPosKey = [](Vector3 p) -> std::string {
        return std::to_string((int)round(p.x)) + "," +
               std::to_string((int)round(p.y)) + "," +
               std::to_string((int)round(p.z));
    };


    float startX = -arenaSize.x / 2.0f;
    float endX = arenaSize.x / 2.0f;
    float startZ = -arenaSize.z / 2.0f;
    float endZ = arenaSize.z / 2.0f;


    float startY = gameMap.GetPosition().y;
    float endY = startY + arenaSize.y;

    for (float x = startX; x <= endX + 0.1f; x += spacing) {
        for (float z = startZ; z <= endZ + 0.1f; z += spacing) {
            for (float y = startY; y <= endY + 0.1f; y += spacing) {

                Vector3 currentPos = { x, y, z };

                if (gameMap.IsBelowGround(currentPos)) continue;

                Node n = { idCounter++, currentPos };
                m_nodes.push_back(n);
                posToId[getPosKey(currentPos)] = n.id;
            }
        }
    }
}

void NavigationGraph::BuildEdges(Vector3 arenaSize, float spacing, const Map& gameMap, const std::vector<Obstacle> &obstacles, std::unordered_map<std::string, int>& posToId) {
    auto getPosKey = [](Vector3 p) -> std::string {
        return std::to_string((int)round(p.x)) + "," +
               std::to_string((int)round(p.y)) + "," +
               std::to_string((int)round(p.z));
    };

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)m_nodes.size(); i++) {
        Vector3 p = m_nodes[i].position;


        for (float dx = -spacing; dx <= spacing; dx += spacing) {
            for (float dy = -spacing; dy <= spacing; dy += spacing) {
                for (float dz = -spacing; dz <= spacing; dz += spacing) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    Vector3 nPos = { p.x + dx, p.y + dy, p.z + dz };
                    std::string key = getPosKey(nPos);

                    if (posToId.count(key)) {
                        float weight = Vector3Distance(p, nPos);

                        if (IsPathBlocked(p, nPos, {}, &gameMap)) continue;

                        if (IsPathBlocked(p, nPos, obstacles, &gameMap)) {
                            weight *= 2.0f;
                        }

                        m_nodes[i].neighbors.push_back({posToId[key], weight});
                    }
                }
            }
        }
    }
}

void NavigationGraph::BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles, const Map& gameMap) {
    m_nodes.clear();
    int idCounter = 0;
    std::unordered_map<std::string, int> posToId;

    BuildNodes(arenaSize, spacing, gameMap, idCounter, posToId);

    BuildEdges(arenaSize, spacing, gameMap, obstacles, posToId);

}



void NavigationGraph::BuildDistanceMatrix() {
    std::vector<int> landmarkIndices;

    // Select corner nodes as Landmarks for the ALT heuristic
    landmarkIndices.push_back(0);
    landmarkIndices.push_back(m_nodes.size() - 1);
    landmarkIndices.push_back(m_nodes.size()/2);

    distanceMatrix.assign(landmarkIndices.size(), std::vector<float>(m_nodes.size(), -1.0f));

    for (size_t i = 0; i < landmarkIndices.size(); ++i) {
        // calculate shortest paths from each landmark to all nodes
        ComputeDijkstra(*this, landmarkIndices[i], distanceMatrix[i]);
    }
}

float NavigationGraph::GetHeuristic(int nIdx, int targetIdx) {
    float maxH = 0.0f;

    // Apply Triangle Inequality: h(n) = max(|dist(n, L) - dist(target, L)|)
    for (size_t i = 0; i < distanceMatrix.size(); ++i) {
        float d_n_L = distanceMatrix[i][nIdx];
        float d_t_L = distanceMatrix[i][targetIdx];

        if (d_n_L != -1.0f && d_t_L != -1.0f) {
            maxH = std::max(maxH, std::abs(d_n_L - d_t_L));
        }
    }

    // Return the tightest admissible heuristic (MAX of ALT and Euclidean)
    float euclidean = Vector3Distance(m_nodes[nIdx].position, m_nodes[targetIdx].position);
    return std::max(maxH, euclidean);
}

struct NodeSearchData {
    float gScore = INFINITY; // Weight sum from the start
    int parentIdx = -1; // Weight to the end
};

std::vector<Vector3> NavigationGraph::FindPathViaAStar(int startIdx, int targetIdx) {
    // Ensure indices are valid and not the same
    if (startIdx == -1 || targetIdx == -1 || startIdx == targetIdx) return {};

    // Priority Queue stores pairs of fScore, nodeIndex
    // std::greater ensures the node with the lowest fScore is always at the top.
    typedef std::pair<float, int> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> openSet;

    // Stores gScore and parent info for each node during the search
    std::unordered_map<int, NodeSearchData> searchMap;

    // Init start node
    searchMap[startIdx].gScore = 0;
    float fStart = GetHeuristic(startIdx, targetIdx);
    openSet.push({fStart, startIdx});

    while (!openSet.empty()) {
        // Get the node with the lowest estimated total cost (fScore)
        int current = openSet.top().second;
        openSet.pop();

        // Target reached and reconstruct path by following parent pointers back to start
        if (current == targetIdx) {
            std::vector<Vector3> path;
            int temp = current;
            while (temp != -1) {
                path.push_back(m_nodes[temp].position);
                temp = searchMap[temp].parentIdx;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Evaluate all neighbors of the current node
        for (auto& edge : m_nodes[current].neighbors) {
            // Calculate distance from start to neighbor through current node
            float tentative_gScore = searchMap[current].gScore + edge.weight;

            // If this path is better than any previously found path to this neighbor
            if (tentative_gScore < searchMap[edge.target].gScore) {
                searchMap[edge.target].parentIdx = current;
                searchMap[edge.target].gScore = tentative_gScore;

                // fScore = cost to reach node (g) + estimated cost to target (h)
                float fScore = tentative_gScore + GetHeuristic(edge.target, targetIdx);
                openSet.push({fScore, edge.target});
            }
        }
    }

    return {}; // No path found
}

// Finds the index of the node closest to a given 3D position
int NavigationGraph::GetClosestNode(Vector3 position) {
    int closestIdx = -1;
    float minOutsideDist = INFINITY;

    // Iterate through all nodes to find the one with the smallest distance
    for (int i = 0; i < (int)m_nodes.size(); i++) {
        // Use squared distance for performance (avoids expensive sqrt() operation)
        float d = Vector3DistanceSqr(position, m_nodes[i].position);

        // Update closest node if a nearer one is found
        if (d < minOutsideDist) {
            minOutsideDist = d;
            closestIdx = i;
        }
    }

    return closestIdx;
}



void NavigationGraph::PrepareGPUData() {
    // Clear previous GPU models to free VRAM
    for (auto &model: m_nodeModels) UnloadModel(model);
    m_nodeModels.clear();

    if (m_nodes.empty()) return;

    // Base geometry for a single node
    Mesh sphere = GenMeshSphere(0.2f, 8, 8);
    bool hasIndices = (sphere.indices != nullptr);
    int indexCount = (sphere.triangleCount * 3);

    // BATCHING: Grouping multiple nodes into a single mesh to reduce Draw Calls
    const int nodesPerMesh = 200;

    for (int startIdx = 0; startIdx < (int) m_nodes.size(); startIdx += nodesPerMesh) {
        int endIdx = std::min(startIdx + nodesPerMesh, (int)m_nodes.size());
        int currentChunkSize = endIdx - startIdx;

        // Allocate memory for the merged mesh (Vertex Buffer)
        Mesh chunkMesh = { 0 };
        chunkMesh.vertexCount = sphere.vertexCount * currentChunkSize;
        chunkMesh.triangleCount = sphere.triangleCount * currentChunkSize;
        chunkMesh.vertices = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        chunkMesh.normals = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        if (hasIndices) chunkMesh.indices = (unsigned short *)MemAlloc(chunkMesh.triangleCount * 3 * sizeof(unsigned short));

        // Fill the buffers by offsetting base sphere vertices to node positions
        for (int i = 0; i < currentChunkSize; i++) {
            int nodeIdx = startIdx + i;
            int vOffset = i * sphere.vertexCount;
            int iOffset = i * indexCount;

            for (int j = 0; j < sphere.vertexCount; j++) {
                // Copy and Translate vertex to its world position
                chunkMesh.vertices[(vOffset + j) * 3 + 0] = sphere.vertices[j * 3 + 0] + m_nodes[nodeIdx].position.x;
                chunkMesh.vertices[(vOffset + j) * 3 + 1] = sphere.vertices[j * 3 + 1] + m_nodes[nodeIdx].position.y;
                chunkMesh.vertices[(vOffset + j) * 3 + 2] = sphere.vertices[j * 3 + 2] + m_nodes[nodeIdx].position.z;

                if (sphere.normals) {
                    chunkMesh.normals[(vOffset + j) * 3 + 0] = sphere.normals[j * 3 + 0];
                    chunkMesh.normals[(vOffset + j) * 3 + 1] = sphere.normals[j * 3 + 1];
                    chunkMesh.normals[(vOffset + j) * 3 + 2] = sphere.normals[j * 3 + 2];
                }
            }

            if (hasIndices) {
                // Shift indices to point to the correct vertices in the merged buffer
                for (int j = 0; j < indexCount; j++) {
                    chunkMesh.indices[iOffset + j] = (unsigned short)(sphere.indices[j] + vOffset);
                }
            }
        }

        // Upload combined data to VRAM and load into a Model
        UploadMesh(&chunkMesh, false);
        Model model = LoadModelFromMesh(chunkMesh);
        model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = RED;
        m_nodeModels.push_back(model);
    }

    UnloadMesh(sphere); // Free the base template mesh from RAM
    m_isModelReady = true;
}

void NavigationGraph::Draw(Vector3 cameraPos, float renderRadius) const {
    if (!m_isModelReady) return;

    float radiusSq = renderRadius * renderRadius;

    // Use raylib's lower-level rendering for immediate line drawing
    rlBegin(RL_LINES);
    rlColor4ub(180, 180, 180, 120);

    for (size_t i = 0; i < m_nodes.size(); i++) {
        // FRUSTUM/DISTANCE CULLING: Skip drawing edges for distant nodes
        if (Vector3DistanceSqr(m_nodes[i].position, cameraPos) > radiusSq) continue;

        for (const auto &edge: m_nodes[i].neighbors) {
            // Avoid drawing the same edge twice (check target > i)
            if (edge.target > (int) i) {
                rlVertex3f(m_nodes[i].position.x, m_nodes[i].position.y, m_nodes[i].position.z);
                rlVertex3f(m_nodes[edge.target].position.x, m_nodes[edge.target].position.y,
                           m_nodes[edge.target].position.z);
            }
        }
    }
    rlEnd();

    // Render the batched node models
    for (const auto &model: m_nodeModels) {
        DrawModel(model, {0, 0, 0}, 1.0f, WHITE);
    }
}
