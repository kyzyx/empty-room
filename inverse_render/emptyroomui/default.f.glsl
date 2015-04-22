#version 400
in vec4 computed_color;
uniform sampler2D colors;
uniform sampler2D angles;
uniform sampler2D aux;

out vec4 fragcolor;

void main(void) {
    fragcolor = computed_color;
}
