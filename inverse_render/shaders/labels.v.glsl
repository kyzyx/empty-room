#version 400
out vec4 computed_color;
out ivec3 id;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;

const int FLOAT_SIG_BITS = 23;
const int FLOAT_EXP_MASK = 1 + (1 << FLOAT_SIG_BITS);

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    computed_color = vec4(intBitsToFloat(vertexaux+FLOAT_EXP_MASK), 1);
    id = vertexaux;
}
