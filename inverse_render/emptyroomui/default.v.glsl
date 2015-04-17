#version 130
out vec4 computed_color;

void main(void) {
    gl_Position = ftransform();
    computed_color = vec4((gl_Normal+1)/2, 1);
}
