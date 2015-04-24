#version 400
out vec4 computed_color;
out ivec3 id;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;
uniform ivec3 auxdata;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;

void main(void) {
    vec4 pos = modelviewmatrix*vec4(position,1) + vec4(0,0,0.01,0);
    gl_Position = projectionmatrix*pos;
    ivec3 tmp = auxdata*vertexaux;
    id = ivec3(tmp.x+tmp.y+tmp.z,0,0);
    computed_color = vec4(float(id%3)/2.,float((id/3)%3)/2.,float((id/9)%3)/2., 1);
}
