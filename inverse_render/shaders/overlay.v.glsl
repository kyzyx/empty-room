#version 400
out vec4 computed_color;
out ivec3 id;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;
uniform ivec3 auxdata;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;

const int NUM_KELLY_COLORS = 20;
uniform vec4 KellyColors[NUM_KELLY_COLORS] = vec4[NUM_KELLY_COLORS](
    vec4(255, 179, 0, 128)/255,
    vec4(128, 62, 117, 128)/255,
    vec4(255, 104, 0, 128)/255,
    vec4(166, 189, 215, 128)/255,
    vec4(193, 0, 32, 128)/255,
    vec4(206, 162, 98, 128)/255,
    vec4(129, 112, 102, 128)/255,
    vec4(0, 125, 52, 128)/255,
    vec4(246, 118, 142, 128)/255,
    vec4(0, 83, 138, 128)/255,
    vec4(255, 122, 92, 128)/255,
    vec4(83, 55, 122, 128)/255,
    vec4(255, 142, 0, 128)/255,
    vec4(179, 40, 81, 128)/255,
    vec4(244, 200, 0, 128)/255,
    vec4(127, 24, 13, 128)/255,
    vec4(147, 170, 0, 128)/255,
    vec4(89, 51, 21, 128)/255,
    vec4(241, 58, 19, 128)/255,
    vec4(35, 44, 22, 128)/255
);

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    ivec3 tmp = auxdata*vertexaux;
    id = ivec3(tmp.x+tmp.y+tmp.z,0,0);
    computed_color = KellyColors[id.r%NUM_KELLY_COLORS];
}
