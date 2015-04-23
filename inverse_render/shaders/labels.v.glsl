#version 400
flat out vec4 computed_color;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    computed_color = vec4(intBitsToFloat(vertexaux), 1);
}
