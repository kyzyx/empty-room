#version 400
out vec4 computed_color;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;

uniform sampler2D colors;
uniform sampler2D angles;
uniform sampler2D aux;

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

const int LIM = 8;

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    computed_color = vec4(0,0,0,1);
    float weight = 0;
    int start = (gl_VertexID%256)*32;
    int row = gl_VertexID/256;

    vec4 avg = vec4(0,0,0,1);
    for (int i = 0; i < LIM; ++i) {
        ivec2 coord = ivec2(start+i, row);
        vec4 a = texelFetch(aux, coord, 0);
        //float currweight = abs(a.r*a.g*dot(gl_Normal,texture2D(angles,coord).rgb));
        float currweight = abs(a.r*a.g);
        avg += texelFetch(colors, coord, 0)*currweight;
        weight += currweight;
    }
    if (weight > 0) avg /= weight;
    avg.a = 1;
    for (int i = 0; i < LIM; i++) {
        ivec2 coord = ivec2(start+i, row);
        vec4 a = texelFetch(aux, coord, 0);
        //float currweight = abs(a.r*a.g*dot(gl_Normal,texture2D(angles,coord).rgb));
        float currweight = abs(a.r*a.g);
        vec4 dev = texelFetch(colors, coord, 0) - avg;
        computed_color += dot(dev, dev)*currweight*vec4(1,1,1,0);
    }
    if (weight > 0) computed_color /= weight;
    avg.a = 1;
}
