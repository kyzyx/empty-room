#version 430
layout(triangles) in;
layout(triangle_strip,max_vertices=3) out;

flat out vec4 flatcolor;

in vec4 computed_color[];
in ivec3 id[];

uniform ivec3 auxdata;

void main(void) {
    if (id[0].r > 0 && id[0].r == id[1].r && id[1].r == id[2].r) {
        flatcolor = computed_color[0];
        for (int i = 0; i < 3; i++) {
            gl_Position = gl_in[i].gl_Position;
            EmitVertex();
        }
        EndPrimitive();
    }
}
