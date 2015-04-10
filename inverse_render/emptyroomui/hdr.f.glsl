#version 400
uniform sampler2D rendered_image;
in vec2 f_texcoord;
uniform vec3 hdr_bounds;

out vec4 color;
// linear shader
void main(void) {
    vec4 f = texture2D(rendered_image, f_texcoord);
    color = clamp((f-hdr_bounds[0])/(hdr_bounds[1]-hdr_bounds[0]), 0, 1);
}
