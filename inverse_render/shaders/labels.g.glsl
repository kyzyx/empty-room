#version 400

layout(triangles) in;
layout(triangle_strip,max_vertices=3) out;

flat out vec4 flatcolor;

in vec4 computed_color[];
in ivec3 id[];

uniform ivec3 auxdata;

void main(void) {
    bvec3 mask = equal(id[0], id[1]) && equal(id[1], id[2]);
    flatcolor = mix(vec4(0,0,0,1), computed_color[0], bvec4(mask,true));
    for (int i = 0; i < 3; i++) {
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
