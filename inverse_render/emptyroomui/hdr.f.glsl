#version 400
uniform sampler2D rendered_image;
in vec2 f_texcoord;

out vec4 color;
// identity shader
void main(void) {
  color = clamp(texture2D(rendered_image, f_texcoord), 0, 1);
}
