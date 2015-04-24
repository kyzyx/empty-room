#version 400
layout(triangles) in;
layout(triangle_strip,max_vertices=3) out;

flat out vec4 flatcolor;

in vec4 computed_color[];

uniform ivec3 auxdata;

bool inrange(vec3 v) {
    return any(greaterThan(v, vec3(auxdata).xxx)) && all(lessThan(v, vec3(auxdata).yyy));
}
void main(void) {
    if (inrange(computed_color[0].rgb)
     && inrange(computed_color[1].rgb) 
     && inrange(computed_color[2].rgb))
    {
        flatcolor = vec4(0,0,1,0.5);
        for (int i = 0; i < 3; i++) {
            gl_Position = gl_in[i].gl_Position;
            EmitVertex();
        }
        EndPrimitive();
    }
}
