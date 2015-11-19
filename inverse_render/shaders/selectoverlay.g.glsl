#version 400
layout(triangles) in;
layout(triangle_strip,max_vertices=3) out;

flat out vec4 flatcolor;

in uint selected[];

uniform ivec3 auxdata;

void main(void) {
    if (selected[0]>0 || selected[1]>0 || selected[2]>0) {
        for (int i = 0; i < 3; i++) {
            gl_Position = gl_in[i].gl_Position - vec4(0,0,0.05,0);
            flatcolor = vec4(1,0,0,1);
            EmitVertex();
        }
        EndPrimitive();
    }
}
