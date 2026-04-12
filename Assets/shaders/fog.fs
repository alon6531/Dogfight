#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
in float fragDepth;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec4 fogColor;
uniform float fogDensity;

out vec4 finalColor;

void main() {
    vec4 texelColor = texture(texture0, fragTexCoord) * colDiffuse * fragColor;

    // חישוב פקטור הערפל (Exponential Fog)
    float dist = fragDepth;
    float fogFactor = 1.0 / exp(pow(dist * fogDensity, 2.0));
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    // ערבוב צבע הטקסטורה עם צבע הערפל
    finalColor = mix(fogColor, texelColor, fogFactor);
}