#version 400
out vec4 computed_color;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    computed_color = vec4((normal+1)/2, 1);
}