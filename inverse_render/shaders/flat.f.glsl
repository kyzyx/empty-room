#version 400
flat in vec4 flatcolor;
uniform sampler2D colors;
uniform sampler2D angles;
uniform sampler2D aux;

out vec4 fragcolor;

void main(void) {
    fragcolor = flatcolor;
}
