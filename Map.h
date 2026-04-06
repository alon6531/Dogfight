#ifndef DOGFIGHT_MAP_H
#define DOGFIGHT_MAP_H

#include "raylib.h"
#include <vector>

class Map {
public:
    Map() = default;
    ~Map();

    // טעינת המפה מתמונה (Heightmap)
    void Load(const char* heightmapPath, Vector3 size, const char* texturePath);

    void UpdateFog(Vector3 cameraPos);

    // ציור המפה בעולם
    void Draw() const;

    // פונקציות גישה (Getters)
    [[nodiscard]] Model GetModel() const { return m_model; }
    [[nodiscard]] Vector3 GetSize() const { return m_size; }
    [[nodiscard]] Image GetHeightmapImage() const { return m_heightmapImage; }

    // בדיקה: האם נקודה מסוימת נמצאת מתחת לפני השטח?
    [[nodiscard]] bool IsBelowGround(Vector3 position) const;

private:
    Model m_model;
    Texture2D m_texture;
    Image m_heightmapImage; // נשמור בזיכרון כדי לבדוק גבהים עבור ה-AI
    Vector3 m_size;
    Vector3 m_position;
    bool m_isLoaded = false;

    Shader m_shader;
    int m_fogDensityLoc;
    float m_fogDensity = 0.015f; // עוצמת הערפל
};

#endif //DOGFIGHT_MAP_H