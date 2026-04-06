#include "Map.h"
#include "raymath.h"

void Map::Load(const char* heightmapPath, Vector3 size, const char* texturePath) {
    m_size = size;
    m_position = { -size.x / 2.0f, -size.y / 2.0f, -size.z / 2.0f };

    m_heightmapImage = LoadImage(heightmapPath);
    Mesh mesh = GenMeshHeightmap(m_heightmapImage, m_size);
    m_model = LoadModelFromMesh(mesh);
    m_texture = LoadTexture(texturePath);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture;

    m_shader = LoadShader("resources/shaders/lighting.vs", "resources/shaders/fog.fs");

    m_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(m_shader, "viewPos");
    m_fogDensityLoc = GetShaderLocation(m_shader, "fogDensity");


    float fogColor[4] = { 0.4f, 0.75f, 1.0f, 1.0f };
    SetShaderValue(m_shader, GetShaderLocation(m_shader, "fogColor"), fogColor, SHADER_UNIFORM_VEC4);


    m_model.materials[0].shader = m_shader;

    m_isLoaded = true;



    m_fogDensity = 0.1f;
}

void Map::UpdateFog(Vector3 cameraPos) {
    if (!m_isLoaded) return;


    SetShaderValue(m_shader, m_shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraPos, SHADER_UNIFORM_VEC3);


    SetShaderValue(m_shader, m_fogDensityLoc, &m_fogDensity, SHADER_UNIFORM_FLOAT);
}

bool Map::IsBelowGround(Vector3 position) const {
    if (!m_isLoaded) return false;


    float xRel = (position.x - m_position.x) / m_size.x;
    float zRel = (position.z - m_position.z) / m_size.z;

    int px = (int)(xRel * m_heightmapImage.width);
    int pz = (int)(zRel * m_heightmapImage.height);


    if (px < 0 || px >= m_heightmapImage.width || pz < 0 || pz >= m_heightmapImage.height) return false;


    Color pixel = GetImageColor(m_heightmapImage, px, pz);
    float groundHeight = (pixel.r / 255.0f) * m_size.y;

    return position.y < groundHeight;
}

void Map::Draw() const {
    if (m_isLoaded) {
        DrawModel(m_model, m_position, 1.0f, WHITE);
    }
}

Map::~Map() {
    if (m_isLoaded) {
        UnloadModel(m_model);
        UnloadTexture(m_texture);
        UnloadImage(m_heightmapImage);
    }
}