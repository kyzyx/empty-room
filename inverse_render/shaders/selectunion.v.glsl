#version 400
uniform mat4 modelviewmatrix;
uniform mat4 projectionmatrix;
uniform ivec3 auxdata;

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in ivec3 vertexaux;
layout(location=3) in vec3 colors;
layout(location=4) in uint prevstatus;

out int status;

const int SHIFT = 16;
const int MASK = (1<<SHIFT) - 1;
ivec2 unpackInt(int n) {
    return ivec2(n >> SHIFT, n & MASK);
}

void main(void) {
    vec4 camv = modelviewmatrix*vec4(position,1);
    vec4 v = projectionmatrix*camv;
    vec4 n = modelviewmatrix*vec4(normal, 0);
    vec2 dims = vec2(unpackInt(auxdata.y));
    vec2 r = vec2(unpackInt(auxdata.x)) - dims*(1+(v.xy/v.w))/2;
    bool inselection = dot(r,r) <= auxdata.z*auxdata.z && n.z > 0 && camv.z < 0;
    status = int(prevstatus>0 || inselection);
    gl_Position = v;
}
