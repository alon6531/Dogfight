//
// Created by User on 03/04/2026.
//
#include "../World/GraphBuilder.h"
#include <cfloat>
#include <cstdint>
#include <iostream>
#include "rlgl.h"
#include "raymath.h"
#include <omp.h>
#include <queue>
#include "../World/Map.h"
#include "../Navigation/Dijkstra.h"



int NavigationGraph::IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles) {
    for (const auto& obs : obstacles) {
        float dx = p.x - obs.pos.x;
        float dy = p.y - obs.pos.y;
        float dz = p.z - obs.pos.z;
        float distSq = (dx * dx) + (dy * dy) + (dz * dz);

        float totalRadius = obs.radius + OBSTACLE_SAFETY_MARGIN;
        if (distSq < (totalRadius * totalRadius)) {
            return 1;
        }
    }
    return 0;
}

float NavigationGraph::GetPathWeight(Vector3 start, Vector3 end, const std::vector<Obstacle>& obstacles, const Map* gameMap) {
    float distance = Vector3Distance(start, end);
    int samples = std::max(PATH_SAMPLE_MIN_COUNT, (int)(distance / PATH_SAMPLE_INTERVAL));
    float weightMultiplier = PATH_WEIGHT_CLEAR_MULT;

    for (int i = 1; i <= samples; i++) {
        float t = (float)i / samples;
        Vector3 checkPoint = Vector3Lerp(start, end, t);

        if (gameMap != nullptr && gameMap->IsBelowGround(checkPoint)) return -1.0f;

        if (IsPointBlocked(checkPoint, obstacles)) {
            weightMultiplier = PATH_WEIGHT_BLOCKED_MULT;
        }
    }
    return distance * weightMultiplier;
}

void NavigationGraph::BuildNodes(Vector3 arenaSize, const Map& gameMap, int& idCounter) {
    m_nodes.clear();

    auto getPosKey = [](Vector3 p) -> uint64_t {
        uint64_t x = (uint64_t)((int)round(p.x) + POS_KEY_OFFSET) & POS_KEY_MASK;
        uint64_t y = (uint64_t)((int)round(p.y) + POS_KEY_OFFSET) & POS_KEY_MASK;
        uint64_t z = (uint64_t)((int)round(p.z) + POS_KEY_OFFSET) & POS_KEY_MASK;
        return (x << POS_KEY_X_SHIFT) | (y << POS_KEY_Y_SHIFT) | z;
    };

    float startX = -arenaSize.x / 2.0f;
    float endX   =  arenaSize.x / 2.0f;
    float startZ = -arenaSize.z / 2.0f;
    float endZ   =  arenaSize.z / 2.0f;

    float startY = gameMap.GetPosition().y;
    float endY   = startY + arenaSize.y;

    for (float x = startX; x <= endX + POS_KEY_GRID_EPSILON; x += m_spacing) {
        for (float z = startZ; z <= endZ + POS_KEY_GRID_EPSILON; z += m_spacing) {
            for (float y = startY; y <= endY + POS_KEY_GRID_EPSILON; y += m_spacing) {
                Vector3 currentPos = { x, y, z };

                if (!gameMap.IsBelowGround(currentPos)) {
                    Node n = { idCounter++, currentPos };
                    m_nodes.push_back(n);
                    m_posToId[getPosKey(currentPos)] = n.id;
                }
            }
        }
    }
}

void NavigationGraph::BuildEdges(const Map& gameMap, const std::vector<Obstacle> &obstacles) {

    auto getPosKey = [](Vector3 p) -> uint64_t {
        uint64_t x = (uint64_t)((int)round(p.x) + POS_KEY_OFFSET) & POS_KEY_MASK;
        uint64_t y = (uint64_t)((int)round(p.y) + POS_KEY_OFFSET) & POS_KEY_MASK;
        uint64_t z = (uint64_t)((int)round(p.z) + POS_KEY_OFFSET) & POS_KEY_MASK;
        return (x << POS_KEY_X_SHIFT) | (y << POS_KEY_Y_SHIFT) | z;
    };

#pragma omp parallel for schedule(guided)
    for (int i = 0; i < (int)m_nodes.size(); i++) {
        Vector3 p = m_nodes[i].position;

        for (float dx = -m_spacing; dx <= m_spacing; dx += m_spacing) {
            for (float dy = -m_spacing; dy <= m_spacing; dy += m_spacing) {
                for (float dz = -m_spacing; dz <= m_spacing; dz += m_spacing) {
                    Vector3 nPos = { p.x + dx, p.y + dy, p.z + dz };
                    uint64_t key = getPosKey(nPos);

                    auto it = m_posToId.find(key);
                    if (it != m_posToId.end()) {
                        float weight = GetPathWeight(p, nPos, obstacles, &gameMap);

                        if (weight > 0) {
                            m_nodes[i].neighbors.push_back({ it->second, weight });
                        }
                    }
                }
            }
        }
    }
}

void NavigationGraph::BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles, const Map& gameMap) {
    m_spacing = spacing;
    m_arenaSize = arenaSize;

    m_nodes.clear();
    int idCounter = 0;

    BuildNodes(arenaSize, gameMap, idCounter);
    BuildEdges(gameMap, obstacles);
}

void NavigationGraph::BuildDistanceMatrix() {
    std::vector<int> landmarkIndices;

    landmarkIndices.push_back(LANDMARK_COUNT_START);
    landmarkIndices.push_back(m_nodes.size() - 1);
    landmarkIndices.push_back(m_nodes.size() / 2);

    distanceMatrix.assign(landmarkIndices.size(), std::vector<float>(m_nodes.size(), LANDMARK_INVALID_DIST));

    Dijkstra solver;
    for (size_t i = 0; i < landmarkIndices.size(); ++i)
        solver.Compute(*this, landmarkIndices[i], distanceMatrix[i]);
}

float NavigationGraph::GetHeuristic(int nIdx, int targetIdx) const {
    float maxH = 0.0f;

    for (size_t i = 0; i < distanceMatrix.size(); ++i) {
        float d_n_L = distanceMatrix[i][nIdx];
        float d_t_L = distanceMatrix[i][targetIdx];

        if (d_n_L != LANDMARK_INVALID_DIST && d_t_L != LANDMARK_INVALID_DIST) {
            maxH = std::max(maxH, std::abs(d_n_L - d_t_L));
        }
    }

    float euclidean = Vector3Distance(m_nodes[nIdx].position, m_nodes[targetIdx].position);
    return std::max(maxH, euclidean);
}

int NavigationGraph::GetClosestNode(Vector3 position) {
    int closestIdx = -1;
    float minDistSq = FLT_MAX;

    for (int i = 0; i < (int)m_nodes.size(); i++) {
        if (m_nodes[i].neighbors.empty()) continue;

        Vector3 np = m_nodes[i].position;
        float dx = np.x - position.x;
        float dy = np.y - position.y;
        float dz = np.z - position.z;
        float distSq = (dx * dx) + (dy * dy) + (dz * dz);

        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestIdx = i;
        }
    }
    return (closestIdx != -1) ? closestIdx : 0;
}

void NavigationGraph::PrepareGPUData() {
    for (auto &model: m_nodeModels) UnloadModel(model);
    m_nodeModels.clear();

    if (m_nodes.empty()) return;

    Mesh box = GenMeshCube(NODE_CUBE_SIZE, NODE_CUBE_SIZE, NODE_CUBE_SIZE);
    int indexCount = (box.triangleCount * 3);

    for (int startIdx = 0; startIdx < (int)m_nodes.size(); startIdx += NODES_PER_MESH_CHUNK) {
        int endIdx = std::min(startIdx + NODES_PER_MESH_CHUNK, (int)m_nodes.size());
        int currentChunkSize = endIdx - startIdx;

        Mesh chunkMesh = { 0 };
        chunkMesh.vertexCount   = box.vertexCount   * currentChunkSize;
        chunkMesh.triangleCount = box.triangleCount * currentChunkSize;

        chunkMesh.vertices = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        chunkMesh.normals  = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        chunkMesh.indices  = (unsigned short *)MemAlloc(chunkMesh.triangleCount * 3 * sizeof(unsigned short));

        for (int i = 0; i < currentChunkSize; i++) {
            int nodeIdx = startIdx + i;
            int vOffset = i * box.vertexCount;
            int iOffset = i * indexCount;

            for (int j = 0; j < box.vertexCount; j++) {
                chunkMesh.vertices[(vOffset + j) * 3 + 0] = box.vertices[j * 3 + 0] + m_nodes[nodeIdx].position.x;
                chunkMesh.vertices[(vOffset + j) * 3 + 1] = box.vertices[j * 3 + 1] + m_nodes[nodeIdx].position.y;
                chunkMesh.vertices[(vOffset + j) * 3 + 2] = box.vertices[j * 3 + 2] + m_nodes[nodeIdx].position.z;

                if (box.normals) {
                    chunkMesh.normals[(vOffset + j) * 3 + 0] = box.normals[j * 3 + 0];
                    chunkMesh.normals[(vOffset + j) * 3 + 1] = box.normals[j * 3 + 1];
                    chunkMesh.normals[(vOffset + j) * 3 + 2] = box.normals[j * 3 + 2];
                }
            }

            for (int j = 0; j < indexCount; j++) {
                chunkMesh.indices[iOffset + j] = (unsigned short)(box.indices[j] + vOffset);
            }
        }

        UploadMesh(&chunkMesh, false);
        Model model = LoadModelFromMesh(chunkMesh);
        model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = RED;
        m_nodeModels.push_back(model);
    }

    UnloadMesh(box);
    m_isModelReady = true;
}

void NavigationGraph::Draw(Vector3 cameraPos, float renderRadius) const {
    if (m_isModelReady) {
        float radiusSq = renderRadius * renderRadius;

        rlBegin(RL_LINES);
        rlColor4ub(EDGE_FALLBACK_COLOR_R, EDGE_FALLBACK_COLOR_G, EDGE_FALLBACK_COLOR_B, EDGE_FALLBACK_COLOR_A);

        for (size_t i = 0; i < m_nodes.size(); i++) {
            if (Vector3DistanceSqr(m_nodes[i].position, cameraPos) > radiusSq) continue;

            for (const auto &edge : m_nodes[i].neighbors) {
                if (edge.target > (int)i) {
                    float relativeWeight  = edge.weight / m_spacing;
                    float hue             = fmodf(relativeWeight * EDGE_HUE_MULTIPLIER, EDGE_HUE_MODULO);
                    float spatialVariation = (m_nodes[i].position.x + m_nodes[i].position.z) * EDGE_SPATIAL_SCALE;
                    hue = fmodf(hue + spatialVariation, EDGE_HUE_MODULO);

                    Color color = ColorFromHSV(hue, EDGE_SATURATION, EDGE_VALUE);

                    if (relativeWeight > EDGE_BLOCKED_THRESHOLD) {
                        color = { EDGE_BLOCKED_COLOR_R, EDGE_BLOCKED_COLOR_G, EDGE_BLOCKED_COLOR_B, EDGE_BLOCKED_COLOR_A };
                    } else {
                        color.a = EDGE_NORMAL_ALPHA;
                    }

                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(m_nodes[i].position.x, m_nodes[i].position.y, m_nodes[i].position.z);
                    rlVertex3f(m_nodes[edge.target].position.x, m_nodes[edge.target].position.y,
                               m_nodes[edge.target].position.z);
                }
            }
        }
        rlEnd();

        for (const auto &model: m_nodeModels) {
            DrawModel(model, {0, 0, 0}, 1.0f, WHITE);
        }
    }
}