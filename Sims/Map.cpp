#include "Map.h"
#include "raymath.h"

void Map::Load(const char* heightmapPath, Vector3 size, const char* texturePath) {
    m_size = size;


    m_position = { -size.x / 2.0f, -size.y * 0.7f, -size.z / 2.0f };

    m_heightmapImage = LoadImage(heightmapPath);

    if (m_heightmapImage.width > 512) {
        ImageResize(&m_heightmapImage, 512, 512);
    }

    Mesh mesh = GenMeshHeightmap(m_heightmapImage, m_size);
    m_model = LoadModelFromMesh(mesh);
    m_texture = LoadTexture(texturePath);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture;

    m_shader = LoadShader("shaders/fog.vs", "shaders/fog.fs");

    // עדכון המיקומים (Locations)
    m_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(m_shader, "matModel");
    m_fogDensityLoc = GetShaderLocation(m_shader, "fogDensity");
    int fogColorLoc = GetShaderLocation(m_shader, "fogColor");

    // צבע ערפל שתואם לשמיים שלך (SKYBLUE)
    float fogColor[4] = { 0.4f, 0.75f, 1.0f, 1.0f };
    SetShaderValue(m_shader, fogColorLoc, fogColor, SHADER_UNIFORM_VEC4);

    m_model.materials[0].shader = m_shader;
    m_fogDensity = 0.001f;

    m_isLoaded = true;
}

void Map::UpdateFog(Vector3 cameraPos) {
    if (!m_isLoaded) return;
    SetShaderValue(m_shader, m_shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraPos, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_fogDensityLoc, &m_fogDensity, SHADER_UNIFORM_FLOAT);
}

// Map.cpp
bool Map::IsBelowGround(Vector3 position) const {
    if (!m_isLoaded) return false;

    // Use m_position as origin — matches exactly where DrawModel places the mesh
    float xRel = (position.x - m_position.x) / m_size.x;
    float zRel = (position.z - m_position.z) / m_size.z;

    if (xRel < 0.0f || xRel > 1.0f || zRel < 0.0f || zRel > 1.0f) return false;

    int px = (int)(xRel * (m_heightmapImage.width - 1));
    int pz = (int)(zRel * (m_heightmapImage.height - 1));

    Color col = GetImageColor(m_heightmapImage, px, pz);
    unsigned char heightVal = col.r;

    float groundHeight = m_position.y + ((float)heightVal / 255.0f) * m_size.y;

    return position.y < groundHeight + 1.0f; // +1 epsilon to catch surface nodes
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

float Map::GetHeightAt(float x, float z) const {
    if (!m_isLoaded) return 0.0f;

    // 1. חישוב מיקום יחסי (0.0 עד 1.0) בתוך המפה
    float xRel = (x - m_position.x) / m_size.x;
    float zRel = (z - m_position.z) / m_size.z;

    // 2. הגנה מפני יציאה מגבולות המפה
    if (xRel < 0.0f || xRel > 1.0f || zRel < 0.0f || zRel > 1.0f) return m_position.y;

    // 3. המרה לקואורדינטות פיקסלים בתמונת ה-Heightmap
    int px = (int)(xRel * (m_heightmapImage.width - 1));
    int pz = (int)(zRel * (m_heightmapImage.height - 1));

    // 4. דגימת הגובה (לפי ערוץ ה-Red כמו ב-GenMeshHeightmap)
    Color col = GetImageColor(m_heightmapImage, px, pz);
    float heightFraction = (float)col.r / 255.0f;

    // 5. חישוב הגובה הסופי ב-World Space
    // (מיקום המפה ב-Y + גובה יחסי מתוך גודל המפה)
    return m_position.y + (heightFraction * m_size.y);
}