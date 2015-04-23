#version 400
out vec4 computed_color;

uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;

uniform sampler2D colors;
uniform sampler2D angles;
uniform sampler2D aux;

uniform int auxdata;

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

void main(void) {
    gl_Position = projectionmatrix*modelviewmatrix*vec4(position,1);
    computed_color = vec4(0,0,0,1);
    float weight = 0;
    int start = (gl_VertexID%256)*32;
    int row = gl_VertexID/256;

    for (int i = 0; i < 16; ++i) {
        ivec2 coord = ivec2(start+i, row);
        vec4 a = texelFetch(aux, coord, 0);
        uint data = floatBitsToUint(a.b);
        uint id = bitfieldExtract(data, 0, 16);
        uint label = bitfieldExtract(data, 16, 16);
        float currweight = abs(a.r*a.g);
        computed_color += texelFetch(colors, coord, 0)*float(id==auxdata);
    }
    computed_color.a = 1;
}