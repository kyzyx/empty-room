#version 400
out vec4 computed_color;
out uint selected;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;
uniform ivec3 auxdata;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;
layout(location=3) in vec3 colors;
layout(location=4) in uint status;

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    selected = status;
}
