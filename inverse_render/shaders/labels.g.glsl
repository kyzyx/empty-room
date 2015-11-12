#version 400

layout(triangles) in;
layout(triangle_strip,max_vertices=3) out;

flat out vec4 flatcolor;

in vec4 computed_color[];
in ivec3 id[];

uniform ivec3 auxdata;

void main(void) {
    ivec3 zero = ivec3(0,0,0);
    bvec3 mask = equal(id[0], zero) || equal(id[1], zero) || equal(id[2], zero);
    flatcolor = mix(computed_color[0], vec4(0,0,0,1), bvec4(mask,false));
    for (int i = 0; i < 3; i++) {
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
