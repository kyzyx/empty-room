#version 130
out vec4 computed_color;
uniform sampler2D colors;
uniform sampler2D angles;
uniform sampler2D aux;

void main(void) {
    gl_Position = ftransform();
    computed_color = vec4(0,0,0,1);
    float weight = 0;
    int start = (gl_VertexID%256)*32;
    int row = gl_VertexID/256;

    for (int i = 0; i < 16; ++i) {
        ivec2 coord = ivec2(start+i, row);
        vec4 a = texelFetch(aux, coord, 0);
        //float currweight = abs(a.r*a.g*dot(gl_Normal,texture2D(angles,coord).rgb));
        float currweight = abs(a.r*a.g);
        computed_color += texelFetch(colors, coord, 0)*currweight;
        weight += currweight;
    }
    if (weight > 0) computed_color /= weight;
    computed_color.a = 1;
}
