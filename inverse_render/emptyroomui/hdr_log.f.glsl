#version 400
uniform sampler2D rendered_image;
in vec2 f_texcoord;
uniform vec3 hdr_bounds;

out vec4 color;
// logarithmic shader
void main(void) {
    vec4 f = log(texture2D(rendered_image, f_texcoord));
    vec3 lb = log(hdr_bounds);
    color = clamp((f-lb[0])/(lb[1]-lb[0]), 0, 1);
}
